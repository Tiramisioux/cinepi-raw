/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * dualHdmiPreviewStage.cpp - side-by-side HDMI preview for a two-sensor rig.
 *
 * Background: the on-camera HDMI preview is drawn by libcamera's DRM preview,
 * and DRM *master* is exclusive per GPU. Two independent cinepi-raw processes
 * (one per sensor) therefore cannot both draw to the display - the second is
 * forced to --nopreview. This stage works around that without any core change:
 *
 *   - The SECONDARY instance publishes its lores YUV420 frame (tightly packed)
 *     into a small System-V shared-memory segment. It never touches DRM.
 *   - The PRIMARY instance owns the DRM preview directly (both cores run with
 *     --nopreview in dual mode, so nothing races for master), reads the latest
 *     secondary frame from shared memory, composites the two lores images
 *     side-by-side into one YUV420 canvas, and shows that canvas via DRM.
 *
 * Single-sensor operation is unaffected: cinemate only wires this stage in when
 * it detects two sensors; with one sensor the normal core `-p` preview runs.
 *
 * This is a first, hardware-untested cut (the 2-sensor beam-splitter rig is
 * future hardware); it is deliberately isolated to this file.
 */

#include <chrono>
#include <cstring>
#include <memory>
#include <string>

#include <linux/dma-buf.h>
#include <sys/ioctl.h>
#include <sys/ipc.h>
#include <sys/mman.h>
#include <sys/shm.h>

#include <libcamera/formats.h>
#include <libcamera/stream.h>

#include "core/buffer_sync.hpp"
#include "core/dma_heaps.hpp"
#include "core/frame_info.hpp"
#include "core/rpicam_app.hpp"
#include "core/stream_info.hpp"
#include "post_processing_stages/post_processing_stage.hpp"
#include "preview/preview.hpp"

#include <sw/redis++/redis++.h>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

// The DRM preview factory is defined in preview/drm_preview.cpp. We call it
// directly (rather than make_preview()) because in dual mode the core preview
// is --nopreview, which would otherwise hand us a null preview.
Preview *make_drm_preview(Options const *options);

using Stream = libcamera::Stream;

namespace
{
// Upper bound for a single lores pane (covers anamorphic 720p-tall panes).
constexpr unsigned int kMaxPaneW = 1920;
constexpr unsigned int kMaxPaneH = 1088;
constexpr size_t kMaxPaneBytes = static_cast<size_t>(kMaxPaneW) * kMaxPaneH * 3 / 2;

// Fixed key so primary and secondary meet on the same segment. "CIND".
constexpr key_t kShareKey = 0x43494E44;

// Live preview source, settable at runtime via cinemate's `set preview`
// command (Redis key hdmi_preview_source): both feeds, cam0 only, cam1 only.
enum class PreviewMode { Both, Cam0, Cam1 };

// Only the primary reads Redis, and only for a preview toggle, so re-reading
// every frame is wasteful; poll every N frames instead (~0.5 s at 30 fps).
constexpr unsigned int kModePollFrames = 15;
constexpr char const *kRedisUrl = "redis://127.0.0.1:6379/0";
constexpr char const *kModeKey = "hdmi_preview_source";

// Tightly packed (stride == width) YUV420 frame published by the secondary.
struct DualPreviewShare
{
	volatile uint32_t seq;   // bumped after each full write; 0 == no frame yet
	uint32_t width;
	uint32_t height;
	uint8_t data[kMaxPaneBytes];
};
} // namespace

#define NAME "dualHdmiPreview"

class dualHdmiPreviewStage : public PostProcessingStage
{
public:
	dualHdmiPreviewStage(RPiCamApp *app);
	~dualHdmiPreviewStage();

	char const *Name() const override;
	void Read(boost::property_tree::ptree const &params) override;
	void Configure() override;
	bool Process(CompletedRequestPtr &completed_request) override;
	void Teardown() override;

private:
	bool attachShare();
	Stream *previewStream(StreamInfo &info) const;
	void publishSecondary(uint8_t const *y, StreamInfo const &info);
	void composeAndShow(uint8_t const *y, StreamInfo const &info);
	bool ensureCanvas(unsigned int pane_w, unsigned int pane_h, unsigned int panes,
					  std::optional<libcamera::ColorSpace> const &cs);
	void refreshMode();

	std::shared_ptr<spdlog::logger> console;

	bool primary_ = true;

	// Live preview-source selection (primary only).
	std::unique_ptr<sw::redis::Redis> redis_;
	PreviewMode mode_ = PreviewMode::Both;
	unsigned int frame_count_ = 0;

	// Shared-memory publish channel (both roles attach; primary reads,
	// secondary writes).
	int shm_id_ = -1;
	DualPreviewShare *share_ = nullptr;
	uint32_t last_seen_seq_ = 0;

	// Primary-only DRM output + double-buffered composite canvas. We compose
	// into the back buffer while the display scans out the front one, then swap
	// on Show() — otherwise the display catches a half-composed buffer (drifting
	// black bands).
	std::unique_ptr<Preview> drm_;
	DmaHeap dma_heap_;
	libcamera::UniqueFD canvas_fd_[2];
	libcamera::Span<uint8_t> canvas_span_[2];
	unsigned int canvas_index_ = 0;
	StreamInfo canvas_info_;
};

char const *dualHdmiPreviewStage::Name() const
{
	return NAME;
}

dualHdmiPreviewStage::dualHdmiPreviewStage(RPiCamApp *app) : PostProcessingStage(app)
{
	console = spdlog::get(NAME);
	if (!console)
		console = spdlog::stdout_color_mt(NAME);
}

dualHdmiPreviewStage::~dualHdmiPreviewStage()
{
	if (share_)
		shmdt(share_);
}

void dualHdmiPreviewStage::Read(boost::property_tree::ptree const &params)
{
	std::string role = params.get<std::string>("role", "primary");
	primary_ = (role != "secondary");
}

// Attach (creating if needed) the shared publish segment. Idempotent.
bool dualHdmiPreviewStage::attachShare()
{
	if (share_)
		return true;

	shm_id_ = shmget(kShareKey, sizeof(DualPreviewShare), IPC_CREAT | 0600);
	if (shm_id_ < 0)
	{
		console->error("shmget failed for dual-preview share");
		return false;
	}
	void *p = shmat(shm_id_, nullptr, 0);
	if (p == reinterpret_cast<void *>(-1))
	{
		console->error("shmat failed for dual-preview share");
		share_ = nullptr;
		return false;
	}
	share_ = static_cast<DualPreviewShare *>(p);
	return true;
}

Stream *dualHdmiPreviewStage::previewStream(StreamInfo &info) const
{
	// Prefer the low-res stream for a cheap composite; fall back to main.
	Stream *s = app_->LoresStream(&info);
	if (!s)
		s = app_->GetMainStream();
	if (s)
		info = app_->GetStreamInfo(s);
	return s;
}

void dualHdmiPreviewStage::Configure()
{
	attachShare();

	if (primary_)
	{
		if (share_)
		{
			// Reset the channel so a stale frame from a previous run isn't
			// composited before the secondary has published anything new.
			share_->seq = 0;
			last_seen_seq_ = 0;
		}
		try
		{
			drm_.reset(make_drm_preview(app_->GetOptions()));
			// make_drm_preview() calls done_callback_ on the previous fd; give
			// it a no-op so the persistent canvas fd doesn't trip bad_function.
			if (drm_)
				drm_->SetDoneCallback([](int) {});
		}
		catch (std::exception const &e)
		{
			console->error("dual HDMI preview: DRM unavailable ({})", e.what());
			drm_.reset();
		}
		try
		{
			redis_ = std::make_unique<sw::redis::Redis>(kRedisUrl);
			refreshMode();
		}
		catch (std::exception const &e)
		{
			console->warn("dual HDMI preview: Redis unavailable ({}); defaulting to both", e.what());
			redis_.reset();
			mode_ = PreviewMode::Both;
		}
	}
	else if (!share_)
	{
		console->warn("dual HDMI preview secondary has no share segment; right pane will be blank");
	}
}

// Poll the live preview-source key. Accepts cam0/cam1/both plus a few aliases.
void dualHdmiPreviewStage::refreshMode()
{
	if (!redis_)
		return;
	PreviewMode next = mode_;
	try
	{
		auto v = redis_->get(kModeKey);
		if (v && !v->empty())
		{
			std::string s = *v;
			if (s == "cam0" || s == "0" || s == "a")
				next = PreviewMode::Cam0;
			else if (s == "cam1" || s == "1" || s == "b")
				next = PreviewMode::Cam1;
			else
				next = PreviewMode::Both; // both / cam0+cam1 / anything else
		}
	}
	catch (std::exception const &)
	{
		return; // transient Redis error: keep the current mode
	}
	if (next != mode_)
	{
		mode_ = next;
		console->info("dual HDMI preview source -> {}",
					  mode_ == PreviewMode::Cam0 ? "cam0" : mode_ == PreviewMode::Cam1 ? "cam1" : "both");
	}
}

bool dualHdmiPreviewStage::ensureCanvas(unsigned int pane_w, unsigned int pane_h, unsigned int panes,
										std::optional<libcamera::ColorSpace> const &cs)
{
	unsigned int cw = pane_w * panes, ch = pane_h;
	if (canvas_fd_[0].isValid() && canvas_info_.width == cw && canvas_info_.height == ch)
		return true;

	// Dimensions changed (mode toggle or first frame). DrmPreview caches
	// imported buffers by fd, and the dma-heap can hand back an fd number we
	// are about to free, so drop its cache first to force a clean re-import of
	// the new canvas rather than reusing a stale framebuffer mapping.
	if (canvas_span_[0].data() && drm_)
		drm_->Reset();

	// (Re)allocate both canvas buffers as dmabufs DRM can import.
	size_t size = static_cast<size_t>(cw) * ch * 3 / 2;
	for (unsigned int i = 0; i < 2; ++i)
	{
		if (canvas_span_[i].data())
		{
			munmap(canvas_span_[i].data(), canvas_span_[i].size());
			canvas_span_[i] = {};
		}
		canvas_fd_[i] = {};

		libcamera::UniqueFD fd = dma_heap_.alloc("dual-hdmi-canvas", size);
		if (!fd.isValid())
		{
			console->error("dual HDMI preview: canvas dma-heap alloc failed");
			return false;
		}
		void *mem = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd.get(), 0);
		if (mem == MAP_FAILED)
		{
			console->error("dual HDMI preview: canvas mmap failed");
			return false;
		}
		canvas_fd_[i] = std::move(fd);
		canvas_span_[i] = libcamera::Span<uint8_t>(static_cast<uint8_t *>(mem), size);
	}
	canvas_index_ = 0;

	canvas_info_ = StreamInfo();
	canvas_info_.width = cw;
	canvas_info_.height = ch;
	canvas_info_.stride = cw; // tightly packed
	canvas_info_.colour_space = cs;
	return true;
}

// Copy a padded lores YUV420 pane into a tightly packed destination laid out as
// [Y(dst_stride*h)][U(dst_stride/2*h/2)][V...], writing at column x_off.
static void blitYUV420(uint8_t const *src, unsigned int src_stride, unsigned int w, unsigned int h,
					   uint8_t *dst, unsigned int dst_stride, unsigned int x_off)
{
	uint8_t const *sY = src;
	uint8_t const *sU = sY + static_cast<size_t>(src_stride) * h;
	uint8_t const *sV = sU + static_cast<size_t>(src_stride / 2) * (h / 2);

	uint8_t *dY = dst;
	uint8_t *dU = dY + static_cast<size_t>(dst_stride) * h;
	uint8_t *dV = dU + static_cast<size_t>(dst_stride / 2) * (h / 2);

	for (unsigned int r = 0; r < h; ++r)
		std::memcpy(dY + static_cast<size_t>(r) * dst_stride + x_off, sY + static_cast<size_t>(r) * src_stride, w);

	unsigned int cw = w / 2, ch = h / 2;
	unsigned int cx = x_off / 2;
	for (unsigned int r = 0; r < ch; ++r)
	{
		std::memcpy(dU + static_cast<size_t>(r) * (dst_stride / 2) + cx,
					sU + static_cast<size_t>(r) * (src_stride / 2), cw);
		std::memcpy(dV + static_cast<size_t>(r) * (dst_stride / 2) + cx,
					sV + static_cast<size_t>(r) * (src_stride / 2), cw);
	}
}

// Fill a rect of the tightly packed YUV420 canvas with a solid luma value and
// neutral chroma (128). All coordinates/sizes must be even. canvas_h is the
// full canvas height (Y rows), needed to locate the U/V planes.
static void fillLumaRect(uint8_t *dst, unsigned int ds, unsigned int canvas_h,
						 unsigned int x, unsigned int y, unsigned int w, unsigned int h, uint8_t yval)
{
	uint8_t *Y = dst;
	uint8_t *U = dst + static_cast<size_t>(ds) * canvas_h;
	uint8_t *V = U + static_cast<size_t>(ds / 2) * (canvas_h / 2);
	for (unsigned int r = 0; r < h; ++r)
		std::memset(Y + static_cast<size_t>(y + r) * ds + x, yval, w);
	unsigned int cx = x / 2, cy = y / 2, cw = w / 2, chh = h / 2;
	for (unsigned int r = 0; r < chh; ++r)
	{
		std::memset(U + static_cast<size_t>(cy + r) * (ds / 2) + cx, 128, cw);
		std::memset(V + static_cast<size_t>(cy + r) * (ds / 2) + cx, 128, cw);
	}
}

// Draw a hollow white rectangle (frame) of thickness t around a pane. Adjacent
// pane frames in `both` mode share the centre edge, which reads as the divider.
static void drawWhiteFrame(uint8_t *dst, unsigned int ds, unsigned int canvas_h,
						   unsigned int x0, unsigned int w, unsigned int h, unsigned int t)
{
	constexpr uint8_t kWhite = 235; // broadcast white, avoids full-range clipping
	if (t == 0 || 2 * t >= h || 2 * t >= w)
		return;
	fillLumaRect(dst, ds, canvas_h, x0, 0, w, t, kWhite);          // top
	fillLumaRect(dst, ds, canvas_h, x0, h - t, w, t, kWhite);      // bottom
	fillLumaRect(dst, ds, canvas_h, x0, 0, t, h, kWhite);          // left
	fillLumaRect(dst, ds, canvas_h, x0 + w - t, 0, t, h, kWhite);  // right
}

void dualHdmiPreviewStage::publishSecondary(uint8_t const *y, StreamInfo const &info)
{
	if (!share_)
		return;
	if (static_cast<size_t>(info.width) * info.height * 3 / 2 > kMaxPaneBytes)
		return; // too large for the channel; drop rather than overrun

	// Write tightly packed (stride == width) so the primary can blit directly.
	blitYUV420(y, info.stride, info.width, info.height, share_->data, info.width, 0);
	share_->width = info.width;
	share_->height = info.height;
	__sync_synchronize();
	share_->seq = share_->seq + 1;
}

void dualHdmiPreviewStage::composeAndShow(uint8_t const *y, StreamInfo const &info)
{
	if (!drm_)
		return;

	if (++frame_count_ % kModePollFrames == 0)
		refreshMode();

	unsigned int pane_w = info.width, pane_h = info.height;
	bool have_secondary =
		share_ && share_->seq != 0 && share_->width == pane_w && share_->height == pane_h;

	// Resolve the requested source; fall back to cam0 if cam1 is requested but
	// the secondary hasn't published a frame yet, so the monitor isn't blank.
	PreviewMode mode = mode_;
	if (mode == PreviewMode::Cam1 && !have_secondary)
		mode = PreviewMode::Cam0;

	unsigned int panes = (mode == PreviewMode::Both) ? 2 : 1;
	if (!ensureCanvas(pane_w, pane_h, panes, info.colour_space))
		return;

	// Compose into the back buffer; the display is scanning out the other one.
	unsigned int idx = canvas_index_;
	uint8_t *dst = canvas_span_[idx].data();
	unsigned int ds = canvas_info_.stride;

	// The canvas is a cached dma-heap buffer, so bracket the CPU writes with
	// DMA_BUF_IOCTL_SYNC — otherwise the display controller reads stale cache
	// lines and the preview is pure static. Mirrors core BufferWriteSync.
	struct dma_buf_sync sync = {};
	sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE;
	::ioctl(canvas_fd_[idx].get(), DMA_BUF_IOCTL_SYNC, &sync);

	// Neutral black (Y=16, Cb/Cr=128) so an unfilled pane looks blank.
	std::memset(dst, 16, static_cast<size_t>(ds) * pane_h);
	std::memset(dst + static_cast<size_t>(ds) * pane_h, 128, canvas_span_[idx].size() - static_cast<size_t>(ds) * pane_h);

	if (mode == PreviewMode::Cam1)
	{
		// cam1 fullscreen: the secondary's frame (tightly packed) fills the pane.
		last_seen_seq_ = share_->seq;
		blitYUV420(share_->data, pane_w, pane_w, pane_h, dst, ds, 0);
	}
	else
	{
		// cam0 fullscreen or both: our own frame occupies the first pane.
		blitYUV420(y, info.stride, pane_w, pane_h, dst, ds, 0);
		if (mode == PreviewMode::Both && have_secondary)
		{
			last_seen_seq_ = share_->seq;
			blitYUV420(share_->data, pane_w /*tightly packed*/, pane_w, pane_h, dst, ds, pane_w);
		}
	}

	// White frame around each pane. In `both` mode the two inner edges meet at
	// the centre, forming the dividing line between the feeds. Thickness matches
	// the single-sensor GUI outline (PREVIEW_GUIDE_OUTLINE_WIDTH = 2); kept even
	// for chroma subsampling.
	constexpr unsigned int t = 2;
	unsigned int ch = canvas_info_.height;
	drawWhiteFrame(dst, ds, ch, 0, pane_w, pane_h, t);
	if (panes == 2)
		drawWhiteFrame(dst, ds, ch, pane_w, pane_w, pane_h, t);

	sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE;
	::ioctl(canvas_fd_[idx].get(), DMA_BUF_IOCTL_SYNC, &sync);

	drm_->Show(canvas_fd_[idx].get(), canvas_span_[idx], canvas_info_);
	canvas_index_ ^= 1; // next frame composes into the other buffer
}

bool dualHdmiPreviewStage::Process(CompletedRequestPtr &completed_request)
{
	StreamInfo info;
	Stream *stream = previewStream(info);
	if (!stream)
		return false;

	// We composite planar YUV420 directly; cinemate always configures the
	// lores stream as YUV420, but guard anyway rather than render garbage.
	if (info.pixel_format != libcamera::formats::YUV420)
	{
		static bool warned = false;
		if (!warned)
		{
			warned = true;
			console->warn("dual HDMI preview: preview stream is not YUV420; disabled");
		}
		return false;
	}

	auto it = completed_request->buffers.find(stream);
	if (it == completed_request->buffers.end() || !it->second)
		return false;

	BufferReadSync r(app_, it->second);
	libcamera::Span<uint8_t> buffer = r.Get()[0];
	uint8_t const *y = buffer.data();

	if (primary_)
		composeAndShow(y, info);
	else
		publishSecondary(y, info);

	return false;
}

void dualHdmiPreviewStage::Teardown()
{
	drm_.reset();
	for (unsigned int i = 0; i < 2; ++i)
	{
		if (canvas_span_[i].data())
		{
			munmap(canvas_span_[i].data(), canvas_span_[i].size());
			canvas_span_[i] = {};
		}
		canvas_fd_[i] = {};
	}
}

static PostProcessingStage *Create(RPiCamApp *app)
{
	return new dualHdmiPreviewStage(app);
}

static RegisterStage reg(NAME, &Create);
