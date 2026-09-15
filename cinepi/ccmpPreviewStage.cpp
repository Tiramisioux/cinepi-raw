/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * ccmpPreviewStage.cpp - decompand the lores preview for 12-bit ClearHDR.
 *
 * The CCMP12 fix in dng_encoder.cpp writes a LinearizationTable into the DNG.
 * That is metadata: no pixel changes, and only a DNG reader ever applies it.
 * The previews are the PiSP back end's YUV output, and the back end was fed the
 * companded codes as if they were linear — so HDMI and MJPEG still render the
 * "before" image with the mid-tones crushed magenta.
 *
 * This stage re-renders the lores frame from the raw Bayer with the decompand
 * applied first (ccmp_preview.hpp), and writes the result back into the lores
 * buffer IN PLACE. Everything downstream then gets corrected pixels for free:
 *
 *   - mjpegPreview           (stage, runs after this one)
 *   - dualHdmiPreview        (stage, runs after this one)
 *   - RPiCamApp::ShowPreview (main loop, after all stages)
 *   - the DNG thumbnail      (main loop; EncodeBuffer takes the lores stream)
 *
 * That last one is a deliberate consequence and not a side effect to be sorry
 * about: the thumbnail was magenta for the same reason the preview was.
 *
 * WHY IN PLACE RATHER THAN A NEW BUFFER. Four consumers, three of which are not
 * ours, and two of which are reached through paths (ShowPreview, EncodeBuffer)
 * that take a Stream* rather than pixels. Correcting the one buffer they all
 * read is the only change that reaches all four without touching any of them.
 *
 * ORDERING. This has to run before the two preview stages. Rather than depend
 * on key order in a post-process JSON that is written by the Cinemate installer
 * on each Pi, cinepi-raw inserts this stage at the FRONT of the chain itself
 * (PostProcessor::EnsureFirstStage, called from RPiCamApp::OpenCamera). An
 * explicit entry in the JSON still wins, so the file can still tune it.
 *
 * SCOPE — the same gate as dng_encoder.cpp, and for the same reason. ClearHDR
 * ON *and* a 12-bit sensor mode. 16-bit ClearHDR is delivered linear with no
 * compander in the path, and a 12-bit SDR mode never companded either;
 * decompanding those would be the same defect with the sign flipped. Anything
 * unmeasured falls through and leaves the ISP's preview alone.
 */

#include <algorithm>
#include <cmath>
#include <cstring>
#include <mutex>
#include <string>

#include <libcamera/control_ids.h>
#include <libcamera/formats.h>
#include <libcamera/stream.h>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include "core/buffer_sync.hpp"
#include "core/rpicam_app.hpp"
#include "core/stream_info.hpp"
#include "post_processing_stages/post_processing_stage.hpp"

#include "ccmp_lut.hpp"
#include "ccmp_preview.hpp"
#include "clip_ceiling.hpp"
#include "cinepi_recorder.hpp"

using Stream = libcamera::Stream;

#define NAME "ccmpPreview"

namespace {

struct RawFormat
{
	unsigned cfa[4];      /* quad order (0,0) (1,0) (0,1) (1,1); unused when mono */
	unsigned container;   /* bits in the storage word */
	bool mono;            /* no CFA — a mono sensor's R16/R12, see below */
};

/* Only the unpacked containers. A CSI2P-packed raw is 12 bits straddling byte
 * boundaries and cannot be indexed as uint16; the imx585 ClearHDR modes run on
 * Pi 5 / PiSP, which delivers unpacked, so rather than carry an unpacked-on-
 * the-fly path that nothing exercises, an unexpected format disables the stage
 * loudly and the ISP preview stands.
 *
 * R16/R12 are the mono sensor's unpacked 12-bit-in-16 / 12-bit containers —
 * the same physical layout as SBGGR16/SBGGR12, but with no CFA: the imx585
 * mono tuning has no AWB/CCM, so faking a Bayer pattern here would put the
 * whole signal in one channel instead of rendering it as luminance. */
bool lookupRawFormat(const libcamera::PixelFormat &fmt, RawFormat &out)
{
	using namespace libcamera::formats;
	static const unsigned RGGB[4] = { CCMP_PREVIEW_R, CCMP_PREVIEW_G, CCMP_PREVIEW_G, CCMP_PREVIEW_B };
	static const unsigned GRBG[4] = { CCMP_PREVIEW_G, CCMP_PREVIEW_R, CCMP_PREVIEW_B, CCMP_PREVIEW_G };
	static const unsigned BGGR[4] = { CCMP_PREVIEW_B, CCMP_PREVIEW_G, CCMP_PREVIEW_G, CCMP_PREVIEW_R };
	static const unsigned GBRG[4] = { CCMP_PREVIEW_G, CCMP_PREVIEW_B, CCMP_PREVIEW_R, CCMP_PREVIEW_G };
	static const unsigned MONO[4] = { CCMP_PREVIEW_R, CCMP_PREVIEW_R, CCMP_PREVIEW_R, CCMP_PREVIEW_R };

	const unsigned *cfa = nullptr;
	unsigned container = 0;
	bool mono = false;

	if (fmt == SRGGB16) { cfa = RGGB; container = 16; }
	else if (fmt == SGRBG16) { cfa = GRBG; container = 16; }
	else if (fmt == SBGGR16) { cfa = BGGR; container = 16; }
	else if (fmt == SGBRG16) { cfa = GBRG; container = 16; }
	else if (fmt == SRGGB12) { cfa = RGGB; container = 12; }
	else if (fmt == SGRBG12) { cfa = GRBG; container = 12; }
	else if (fmt == SBGGR12) { cfa = BGGR; container = 12; }
	else if (fmt == SGBRG12) { cfa = GBRG; container = 12; }
	else if (fmt == R16) { cfa = MONO; container = 16; mono = true; }
	else if (fmt == R12) { cfa = MONO; container = 12; mono = true; }

	if (!cfa)
		return false;

	std::memcpy(out.cfa, cfa, sizeof(out.cfa));
	out.container = container;
	out.mono = mono;
	return true;
}

} // namespace

class ccmpPreviewStage : public PostProcessingStage
{
public:
	ccmpPreviewStage(RPiCamApp *app) : PostProcessingStage(app)
	{
		console = spdlog::get(NAME);
		if (!console)
			console = spdlog::stdout_color_mt(NAME);
	}

	char const *Name() const override { return NAME; }

	void Read(boost::property_tree::ptree const &params) override
	{
		colour_.exposure = params.get<double>("exposure", colour_.exposure);
		colour_.gamma = params.get<double>("gamma", colour_.gamma);
		/* 0 puts the clipped-channel magenta back, which is the A/B. */
		colour_.highlight_rolloff = params.get<double>("highlightRolloff", colour_.highlight_rolloff);
		/* The raw code the rolloff is a fraction OF -- see
		 * CcmpPreviewColour::sensor_clip_code. Exposed so a rig whose ClearHDR
		 * clamp lands somewhere else can be corrected without a rebuild; the
		 * "max code seen" line below is how that value gets checked. */
		colour_.sensor_clip_code = params.get<unsigned>("sensorClipCode", colour_.sensor_clip_code);
	}

	void Configure() override;
	bool Process(CompletedRequestPtr &completed_request) override;
	void measureClamp(const uint8_t *raw);
	void Teardown() override { enabled_ = false; }

private:
	std::shared_ptr<spdlog::logger> console;

	/* PostProcessor::Process() spawns a DETACHED THREAD PER REQUEST, so two
	 * frames can be inside Process() at once. The buffers are per-request and
	 * safe, but the renderer's baked matrix and the gains it is baked from are
	 * one instance shared between them — without this a frame renders with the
	 * next frame's white balance half-applied. Held across setColour+render;
	 * the work is ~3 ms against a 20-40 ms frame period, so serialising here
	 * costs nothing real. */
	std::mutex mutex_;

	bool enabled_ = false;
	Stream *raw_stream_ = nullptr;
	Stream *lores_stream_ = nullptr;
	size_t raw_bytes_ = 0;
	size_t lores_bytes_ = 0;
	CcmpPreviewRenderer renderer_;
	CcmpPreviewColour colour_;

	static constexpr unsigned kMaxCodeReportFrames = 120;
	unsigned frames_since_report_ = 0;

	/* ── the live clamp measurement (16-bit ClearHDR only) ─────────────────
	 *
	 * COST. The renderer already reads every raw sample, so this is the second
	 * pass over the frame and the only new per-frame work in the stage. It is
	 * kept off the critical path by measuring 1 frame in 8 and every 4th row of
	 * that frame — about 1/32 of the samples the render itself touches — which
	 * at 25 fps still puts a fresh answer in front of the operator roughly
	 * every 1.3 s. kRowStride 4 is inside the range the encoder's own stride of
	 * 3 was validated over (all eleven fixtures re-run at 1/3, 1/6 and 1/12
	 * gave bit-identical verdicts). */
	static constexpr unsigned kMeasureEveryNth = 8;
	static constexpr unsigned kAccumulations = 4;
	static constexpr unsigned kRowStride = 4;
	/* Below this the answer is noise, not a clamp: a frame with no blown
	 * highlight in it has nothing to measure and must leave the reference where
	 * it is rather than invent one. The detector refuses those itself; this is
	 * only the hysteresis that stops a converged answer jittering the whole
	 * picture's exposure by a fraction of a stop every second. */
	static constexpr double kAdoptFraction = 0.005;
	/* imx585 16-bit linear pedestal, the value SensorBlackLevels reports and
	 * dng_encoder.cpp writes into the file's own BlackLevel tag. Refined per
	 * frame from metadata below; this is only what the first frame uses. */
	static constexpr double kLinearBlackLevel = 256.0;

	bool measure_ = false;
	ClipCeilingDetector ceiling_det_;
	unsigned accumulations_ = 0;
	unsigned measure_phase_ = 0;
};

void ccmpPreviewStage::Configure()
{
	enabled_ = false;
	raw_stream_ = nullptr;
	lores_stream_ = nullptr;

	Options const *options = app_->GetOptions();
	if (!options)
		return;

	/* Frozen here, once, rather than read again below: options->mode is a
	 * live pointer into the controller's redis-mutable state, and raw_info
	 * (fetched a few lines down) is the actual validated stream. Deriving
	 * gate 1 and geom.raw_shift from two reads of the same mutable field taken
	 * at different times is exactly the race cinepi_raw.cpp's encoder
	 * snapshot exists to avoid — see its comment for the full reasoning. */
	const Mode requested_mode = options->mode;

	/* Gate 1 — the scope. Silent, because every SDR mode lands here on every
	 * reconfigure and none of them is a problem.
	 *
	 * 16-BIT USED TO BE TURNED AWAY HERE, on the reasoning that it is linear
	 * with no compander in the path, so there was nothing to decompand. That
	 * reasoning was right and the conclusion was wrong: the magenta was never
	 * the compander. It is the highlight desaturation being referenced to a
	 * white level the sensor cannot reach — see setClipCeiling() in
	 * ccmp_preview.hpp. 12-bit had that fixed with a per-binning anchor; this
	 * mode was simply never in scope to be fixed. */
	const bool linear = requested_mode.bit_depth == 16;
	if (!(options->hdr == "sensor" || options->hdr == "auto") ||
		(requested_mode.bit_depth != 12 && !linear))
		return;

	StreamInfo raw_info, lores_info;
	raw_stream_ = app_->RawStream(&raw_info);
	lores_stream_ = app_->LoresStream(&lores_info);
	if (!raw_stream_ || !lores_stream_)
	{
		console->warn("ccmpPreview: {}-bit ClearHDR but no {} stream; preview stays magenta",
					  requested_mode.bit_depth, raw_stream_ ? "lores" : "raw");
		return;
	}

	if (requested_mode.width != raw_info.width || requested_mode.height != raw_info.height)
	{
		/* Refuse rather than decompand data the requested mode doesn't
		 * actually describe — same reasoning as ccmp_gate.hpp on the encoder
		 * side: requested_mode.bit_depth (12 or 16, checked above) is a
		 * snapshot of the REQUEST, and on a mismatch it cannot be trusted to
		 * describe what raw_info actually is. Both branches have something to
		 * lose by it — decompanding a stream that is genuinely linear renders
		 * worse than the ISP preview it replaces, and taking a 12-bit stream
		 * for a 16-bit one puts the measurement three orders of magnitude off
		 * and normalises the picture into black. */
		console->warn("ccmpPreview: requested mode {}x{} does not match the configured raw "
					   "stream {}x{}; refusing to decompand — preview stays magenta.",
					   requested_mode.width, requested_mode.height, raw_info.width, raw_info.height);
		return;
	}

	if (lores_info.pixel_format != libcamera::formats::YUV420)
	{
		console->warn("ccmpPreview: lores stream is {} not YUV420; preview stays magenta",
					  lores_info.pixel_format.toString());
		return;
	}

	RawFormat raw_format;
	if (!lookupRawFormat(raw_info.pixel_format, raw_format))
	{
		console->warn("ccmpPreview: raw stream is {}, which this stage cannot address as "
					  "16-bit samples; preview stays magenta",
					  raw_info.pixel_format.toString());
		return;
	}

	/* Gate 2 — a MEASURED decompand table for this mode's binning. Same call
	 * and same refusal as the encoder: a binning factor with no measured anchor
	 * is an unvalidated mode, and the register-only curve is wrong by 21 L
	 * through the mid-tones. Keyed on the actual configured raw stream
	 * (raw_info), not the requested mode — see SensorBinning()'s comment.
	 *
	 * Skipped entirely when linear: 16-bit ClearHDR never went through the
	 * compander, so there is no table to be missing and nothing for a binning
	 * factor to key. */
	const double binning = static_cast<CinePIRecorder *>(app_)->SensorBinning(raw_info.width, raw_info.height);
	std::string err;
	const CcmpLut *lut = nullptr;
	if (!linear)
	{
		lut = get_ccmp_lut(binning, err);
		if (!lut)
		{
			console->warn("ccmpPreview: no CCMP decompand table: {}. Preview stays magenta.", err);
			return;
		}
	}

	CcmpPreviewGeometry geom;
	geom.raw_width = raw_info.width;
	geom.raw_height = raw_info.height;
	geom.raw_stride = raw_info.stride;
	/* Detail 2 in ccmp_preview.hpp: PiSP hands the 12-bit mode over MSB-aligned
	 * in a 16-bit word, so the sensor's own code is `px >> 4`. Derived from the
	 * container rather than hardcoded, so a 12-in-12 stream reads correctly too. */
	geom.raw_shift = raw_format.container - requested_mode.bit_depth;
	std::memcpy(geom.cfa, raw_format.cfa, sizeof(geom.cfa));
	geom.mono = raw_format.mono;
	geom.out_width = lores_info.width;
	geom.out_height = lores_info.height;
	geom.out_stride = lores_info.stride;

	const bool configured = linear
								? renderer_.configureLinear(geom, kLinearBlackLevel,
															static_cast<double>((1u << requested_mode.bit_depth) - 1u),
															&err)
								: renderer_.configure(geom, *lut, &err);
	if (!configured)
	{
		console->warn("ccmpPreview: {}. Preview stays magenta.", err);
		return;
	}

	/* The measurement that replaces the anchor. clip_ceiling.hpp is the same
	 * header dng_encoder.cpp measures WhiteLevel with, unmodified — it is pure,
	 * it takes raw Bayer, and the preview has raw Bayer. Running it here rather
	 * than sharing the encoder's answer is deliberate: the encoder only has one
	 * while a take is rolling, and the monitor has to be right BEFORE the
	 * operator presses record, which is the whole point of a monitor.
	 *
	 * No curve goes in for the linear path, because there is none — the raw
	 * code IS the linearised value, so the detector's answer comes back in the
	 * domain the renderer normalises in, exactly as it comes back in the tag's
	 * domain on the encoder side. */
	measure_ = linear;
	if (measure_)
	{
		uint8_t cfa8[4];
		for (int i = 0; i < 4; ++i)
			cfa8[i] = static_cast<uint8_t>(raw_format.cfa[i]);
		ceiling_det_.reset(static_cast<unsigned>((1u << requested_mode.bit_depth) - 1u),
						   static_cast<unsigned>(kLinearBlackLevel), cfa8);
		accumulations_ = 0;
		measure_phase_ = 0;
	}

	/* The display's matrix, read off the stream rather than assumed. */
	colour_.rec709 = lores_info.colour_space &&
					 lores_info.colour_space->ycbcrEncoding == libcamera::ColorSpace::YcbcrEncoding::Rec709;

	raw_bytes_ = static_cast<size_t>(raw_info.stride) * raw_info.height;
	lores_bytes_ = static_cast<size_t>(lores_info.stride) * lores_info.height * 3 / 2;
	enabled_ = true;

	/* setColour() so highlightReference() below reports the level this take will
	 * actually use, rather than the previous configure's. Process() sets it
	 * again per frame for the live AWB gains. */
	renderer_.setColour(colour_);
	renderer_.resetMaxCode();
	frames_since_report_ = 0;

	console->info("ccmpPreview: {} -> {}x{} preview, b={}, exposure {:.2f} gamma {:.2f} "
				  "highlightRolloff {:.3f} clip anchor {} (desaturation from {:.4f} of "
				  "full scale) {}",
				  linear ? "16-bit ClearHDR linear (no decompand)" : lut->params().describe(),
				  geom.out_width, geom.out_height,
				  static_cast<long long>(binning), colour_.exposure, colour_.gamma,
				  colour_.highlight_rolloff, renderer_.resolvedClipCode(),
				  renderer_.highlightReference() * (1.0 - colour_.highlight_rolloff),
				  colour_.rec709 ? "Rec709" : "Rec601");
	if (measure_)
		console->info("ccmpPreview: measuring the clamp live; until it converges the "
					  "reference is nominal white {} and blown highlights stay magenta",
					  renderer_.nominalWhite());
}

/* Accumulates a strided sample of one frame, and every kAccumulations frames
 * asks the detector for a verdict.
 *
 * WHAT IS ADOPTED AND WHAT IS NOT. The detector refuses far more often than it
 * answers, and every refusal is correct: a frame with no blown highlight has no
 * clamp in it, and a frame that runs to full scale is an SDR-shaped histogram
 * where a "ceiling" would be invented rather than found. A refusal therefore
 * leaves the reference exactly where it was — which for a fresh configure means
 * nominal white, i.e. today's behaviour, magenta and all. The stage never
 * guesses; it either has the number or it renders the way it always did.
 *
 * Called with the stage mutex held, from Process(), which is also the only
 * place the renderer is touched. */
void ccmpPreviewStage::measureClamp(const uint8_t *raw)
{
	if (++measure_phase_ % kMeasureEveryNth)
		return;

	const CcmpPreviewGeometry &g = renderer_.geometry();
	for (unsigned y = 0; y < g.raw_height; y += kRowStride)
		ceiling_det_.add_row(reinterpret_cast<const uint16_t *>(raw + static_cast<size_t>(y) * g.raw_stride),
							 g.raw_width, y, g.raw_shift);

	if (++accumulations_ < kAccumulations)
		return;
	accumulations_ = 0;

	const ClipCeilingDetector::Result r = ceiling_det_.detect();
	if (r.found)
	{
		const double now = renderer_.clipCeiling();
		const double want = static_cast<double>(r.ceiling);
		if (std::fabs(want - now) > now * kAdoptFraction)
		{
			renderer_.setClipCeiling(want);
			console->info("ccmpPreview: clamp measured at {} (was referencing {:.0f}); "
						  "blown highlights now desaturate from {:.4f} of it",
						  r.ceiling, now, 1.0 - colour_.highlight_rolloff);
		}
	}
	/* A refusal deliberately does nothing. The last good answer is kept rather
	 * than snapping back to nominal white the moment the lamp leaves frame: the
	 * clamp is a property of the sensor at this gain, not of what happens to be
	 * in shot, and a monitor that jumps a third of a stop every time you pan off
	 * the highlight is worse than one that is slightly stale. Configure() is
	 * what clears it, which is right — that is where the gain and the mode can
	 * actually have changed. */

	/* Same window discipline as the peak-code report below: a fresh histogram
	 * each time, so one old frame's highlight cannot hold the answer up. */
	uint8_t cfa8[4];
	for (int i = 0; i < 4; ++i)
		cfa8[i] = static_cast<uint8_t>(g.cfa[i]);
	ceiling_det_.reset(static_cast<unsigned>(renderer_.nominalWhite()),
					   static_cast<unsigned>(renderer_.blackLevel()), cfa8);
}

bool ccmpPreviewStage::Process(CompletedRequestPtr &completed_request)
{
	std::lock_guard<std::mutex> lock(mutex_);

	if (!enabled_)
		return false;

	auto raw_it = completed_request->buffers.find(raw_stream_);
	auto lores_it = completed_request->buffers.find(lores_stream_);
	if (raw_it == completed_request->buffers.end() || !raw_it->second ||
		lores_it == completed_request->buffers.end() || !lores_it->second)
		return false;

	/* The gains the pipeline is actually using. Per frame, because AWB moves. */
	if (auto cg = completed_request->metadata.get(libcamera::controls::ColourGains); cg)
	{
		colour_.r_gain = (*cg)[0];
		colour_.b_gain = (*cg)[1];
	}
	if (auto m = completed_request->metadata.get(libcamera::controls::ColourCorrectionMatrix); m)
	{
		for (int i = 0; i < 9; ++i)
			colour_.ccm[i] = (*m)[i];
	}
	renderer_.setColour(colour_);

	BufferReadSync rr(app_, raw_it->second);
	BufferWriteSync lw(app_, lores_it->second);
	libcamera::Span<uint8_t> raw = rr.Get()[0];
	libcamera::Span<uint8_t> lores = lw.Get()[0];

	/* Short buffers would be read or written past the end, and this runs on the
	 * post-processing thread where that is a silent heap corruption rather than
	 * a visible glitch. Configure() derived both sizes from the stream info; if
	 * the actual plane disagrees, the stream is not what it said it was and
	 * rendering from it is guesswork. */
	if (raw.size() < raw_bytes_ || lores.size() < lores_bytes_)
	{
		console->warn("ccmpPreview: buffers are raw {} / lores {} bytes, need {} / {}; disabled",
					  raw.size(), lores.size(), raw_bytes_, lores_bytes_);
		enabled_ = false;
		return false;
	}

	/* SensorBlackLevels is per-channel in the 16-bit linear domain, which is
	 * the domain this path normalises in — so it goes straight in. The four
	 * values are within a code of each other on this sensor and the renderer
	 * carries one black, so the smallest is used: erring low lifts the shadows
	 * a hair, erring high crushes them, and only one of those is recoverable by
	 * looking harder at the monitor. */
	if (measure_)
	{
		auto bl = completed_request->metadata.get(libcamera::controls::SensorBlackLevels);
		if (bl && bl->size() >= 4)
		{
			auto lo = (*bl)[0];
			for (size_t i = 1; i < 4; ++i)
				lo = std::min(lo, (*bl)[i]);
			if (lo >= 0 && static_cast<double>(lo) < renderer_.clipCeiling())
				renderer_.setBlackLevel(static_cast<double>(lo));
		}
		measureClamp(raw.data());
	}

	renderer_.render(raw.data(), lores.data());

	/* The one number that says whether sensor_clip_code matches this sensor:
	 * point the camera at a blown highlight and the peak should settle at (not
	 * above) the configured clip code. Every ~120 frames so it is a few seconds
	 * apart at any frame rate, and the window is reset each time so a stale
	 * maximum from an earlier shot cannot linger. */
	if (++frames_since_report_ >= kMaxCodeReportFrames)
	{
		/* highestUncorrected is the number to read: with the anchor in the right
		 * place it sits just under it, and when it tracks the peak instead the
		 * anchor is too high and the blown area is still magenta. */
		/* For the linear path resolvedClipCode() is 0 by construction — there
		 * is no per-binning anchor and no table to carry one — so what has to
		 * be reported is the measured ceiling, which IS the reference. Read it
		 * the same way either way: with a blown highlight in frame the peak
		 * should settle at it, not above it. */
		if (measure_)
			console->info("ccmpPreview: peak raw code {}, highest uncorrected {}, {} quads fully "
						  "desaturated, over the last {} frames (clamp {:.0f}, {})",
						  renderer_.maxCodeSeen(), renderer_.maxUndesaturatedCode(),
						  renderer_.fullyDesaturated(), frames_since_report_,
						  renderer_.clipCeiling(),
						  renderer_.usingMeasuredCeiling() ? "measured" : "nominal, not yet measured");
		else
			console->info("ccmpPreview: peak raw code {}, highest uncorrected {}, {} quads fully "
						  "desaturated, over the last {} frames (clip anchor {})",
						  renderer_.maxCodeSeen(), renderer_.maxUndesaturatedCode(),
						  renderer_.fullyDesaturated(), frames_since_report_,
						  renderer_.resolvedClipCode());
		renderer_.resetMaxCode();
		frames_since_report_ = 0;
	}
	return false;
}

static PostProcessingStage *Create(RPiCamApp *app)
{
	return new ccmpPreviewStage(app);
}

static RegisterStage reg(NAME, &Create);
