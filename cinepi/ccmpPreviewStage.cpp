/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * ccmpPreviewStage.cpp - clean up the ClearHDR preview: decompand 12-bit,
 * neutralise the merge-clamp zone in both 12-bit and 16-bit.
 *
 * Two unrelated defects share one gate (ClearHDR) and one buffer (the lores
 * plane), which is why they live in one stage.
 *
 * 12-BIT: THE COMPANDER. The CCMP12 fix in dng_encoder.cpp writes a
 * LinearizationTable into the DNG. That is metadata: no pixel changes, and
 * only a DNG reader ever applies it. The previews are the PiSP back end's YUV
 * output, and the back end was fed the companded codes as if they were linear
 * -- so HDMI and MJPEG still render the "before" image with the mid-tones
 * crushed magenta. This stage re-renders the lores frame from the raw Bayer
 * with the decompand applied first (ccmp_preview.hpp).
 *
 * 16-BIT: THE MERGE CLAMP, NO COMPANDER INVOLVED. 16-bit ClearHDR is
 * delivered linear -- there is nothing to decompand -- but the imx585's HG/LG
 * merge still clamps digitally, and the clamp still makes R, G and B converge
 * on one code. The ISP's own render is otherwise correct (denoise, sharpening,
 * the tuned CCM and gamma), so this stage keeps it and neutralises only the
 * clamp zone, in place, over the ISP's YUV output (clip_neutralise.hpp). The
 * anchor cannot be a constant the way the 12-bit table's is: the same take
 * plateaued at raw code 54100 at analogue gain code 71 and at 48600 at code
 * 80, thirteen minutes apart -- so it is measured off the raw every frame
 * instead (clip_plateau.hpp), from the signature a clamp always leaves: a
 * bright Bayer quad whose four samples agree. See
 * cinemate-handbook/architecture/cinepi-raw.md, "The clamp zone", for the
 * full picture, including the 12-bit shadow measurement this file also logs.
 *
 * BOTH write the result back into the lores buffer IN PLACE. Everything
 * downstream then gets corrected pixels for free:
 *
 *   - mjpegPreview           (stage, runs after this one)
 *   - dualHdmiPreview        (stage, runs after this one)
 *   - RPiCamApp::ShowPreview (main loop, after all stages)
 *   - the DNG thumbnail      (main loop; EncodeBuffer takes the lores stream)
 *
 * That last one is a deliberate consequence and not a side effect to be sorry
 * about: the thumbnail was pink/magenta for the same reason the preview was.
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
 * SCOPE. Gate 1 is ClearHDR on, 12-bit or 16-bit. Both defects need ClearHDR
 * engaged -- SDR never companded and its clamp behaviour, if any, is untested
 * and out of scope. 16-BIT IS IN SCOPE FOR THE CLAMP ZONE ONLY, NOT FOR
 * DECOMPANDING -- it was never companded, and decompanding a mode that did
 * not compand is the same defect with the sign flipped (ccmp_lut.hpp, and
 * dng_encoder.cpp's own scope comment). A 16-bit mono mode has no CFA, no
 * measured gains and therefore no cast to correct, so it is left on the ISP
 * preview and this stage says so in the log rather than staying silent about
 * why nothing happened. Anything else unmeasured (a raw container this stage
 * cannot address, a non-YUV420 lores) falls through and leaves the ISP's
 * preview alone.
 */

#include <algorithm>
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
#include "cinepi_recorder.hpp"
#include "clip_neutralise.hpp"
#include "clip_plateau.hpp"

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

/* 16-bit's hysteresis band: an adopted anchor only moves for a measurement
 * that differs by more than this many codes. 65536 >> 9 = 128, i.e. ~0.2% of
 * full scale — the measured floor jitters by single codes between frames of
 * the same take (2026-09-13 evidence: 54032/54037 ten frames apart), well
 * inside one 64-code histogram bin, so this absorbs that jitter without
 * absorbing a genuine gain-driven move (11% of full scale between the two
 * measured takes). */
constexpr unsigned kFullScale16 = 1u << 16;
constexpr unsigned kAnchorHysteresis = kFullScale16 >> 9;

/* Below this max(r_gain, b_gain), a neutral subject is itself equal-code —
 * the clamp's whole signature stops meaning "clamp". See
 * CcmpPreviewColour::r_gain/b_gain's comment: the shipping neutral patch sits
 * at ~0.55 of green under real gains (2.5/2.2), which is what makes an
 * equal-code quad distinctive in the first place. */
constexpr double kMinGainForAutoDetect = 1.15;

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
		/* 0 puts the clipped-channel pink/magenta back in either mode — the
		 * 12-bit renderer's own correction and the 16-bit neutraliser both
		 * gate on this the same way, since it is the same ramp shape applied
		 * to different buffers. */
		colour_.highlight_rolloff = params.get<double>("highlightRolloff", colour_.highlight_rolloff);
		/* The raw code the rolloff is a fraction OF. In 12-bit this overrides
		 * the per-binning table anchor (CcmpAnchor::clip_code). In 16-bit
		 * there is no table: 0 (the default) means "measure the clamp from
		 * the frame, every frame" (clip_plateau.hpp); non-zero PINS the
		 * anchor and switches auto-detection off entirely, for a rig whose
		 * clamp has been independently measured and should not chase a scene
		 * change. Either way, the periodic log line names which mode (auto,
		 * override or off) is in force and reports the code actually used. */
		colour_.sensor_clip_code = params.get<unsigned>("sensorClipCode", colour_.sensor_clip_code);
	}

	void Configure() override;
	bool Process(CompletedRequestPtr &completed_request) override;
	void Teardown() override { enabled_ = false; }

private:
	enum class ClearHdrMode { kOff, k12Bit, k16Bit };

	void configure12Bit(const StreamInfo &raw_info, const StreamInfo &lores_info, const RawFormat &raw_format,
						 const Mode &requested_mode);
	void configure16Bit(const StreamInfo &raw_info, const StreamInfo &lores_info, const RawFormat &raw_format);

	bool process12Bit(const uint8_t *raw, uint8_t *lores, const libcamera::ControlList &metadata);
	bool process16Bit(const uint8_t *raw, uint8_t *lores, const libcamera::ControlList &metadata);

	std::shared_ptr<spdlog::logger> console;

	/* PostProcessor::Process() spawns a DETACHED THREAD PER REQUEST, so two
	 * frames can be inside Process() at once. The buffers are per-request and
	 * safe, but the renderer/neutraliser's baked state (colour_, the anchor,
	 * the detector) is one instance shared between them — without this a
	 * frame renders with the next frame's gains or anchor half-applied. Held
	 * across setColour/setAnchor + render/apply + detect; the work is a few
	 * ms against a 20-40 ms frame period, so serialising here costs nothing
	 * real. */
	std::mutex mutex_;

	bool enabled_ = false;
	ClearHdrMode mode_ = ClearHdrMode::kOff;
	Stream *raw_stream_ = nullptr;
	Stream *lores_stream_ = nullptr;
	size_t raw_bytes_ = 0;
	size_t lores_bytes_ = 0;

	/* 12-bit only. */
	CcmpPreviewRenderer renderer_;

	/* 16-bit only. */
	HighlightNeutraliser neutraliser_;
	uint8_t white_ = 235;
	unsigned auto_anchor_ = 0; /* 0 = none measured/adopted yet ("off") */
	bool have_last_gain_ = false;
	float last_analogue_gain_ = 0.f;
	bool low_gain_logged_ = false;
	ClipPlateauDetector::Result last_result_{};

	/* Shared: the colour/geometry knobs (Read()) and the shadow/real
	 * detector, configured for whichever mode is active. */
	CcmpPreviewColour colour_;
	ClipPlateauDetector detector_;

	static constexpr unsigned kMaxCodeReportFrames = 120;
	unsigned frames_since_report_ = 0;
};

void ccmpPreviewStage::Configure()
{
	enabled_ = false;
	mode_ = ClearHdrMode::kOff;
	raw_stream_ = nullptr;
	lores_stream_ = nullptr;
	renderer_.setPlateauDetector(nullptr);

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

	/* Gate 1 — the scope. ClearHDR AND a 12-bit or 16-bit sensor mode.
	 * Silent, because every SDR mode lands here on every reconfigure and none
	 * of them is a problem. */
	const bool clear_hdr = (options->hdr == "sensor" || options->hdr == "auto");
	if (!clear_hdr || (requested_mode.bit_depth != 12 && requested_mode.bit_depth != 16))
		return;

	StreamInfo raw_info, lores_info;
	raw_stream_ = app_->RawStream(&raw_info);
	lores_stream_ = app_->LoresStream(&lores_info);
	if (!raw_stream_ || !lores_stream_)
	{
		console->warn("ccmpPreview: ClearHDR but no {} stream; preview stays uncorrected",
					  raw_stream_ ? "lores" : "raw");
		return;
	}

	if (requested_mode.width != raw_info.width || requested_mode.height != raw_info.height)
	{
		/* Refuse rather than correct a stream the requested mode doesn't
		 * actually describe — same reasoning as ccmp_gate.hpp on the encoder
		 * side: requested_mode.bit_depth (already checked above) is a
		 * snapshot of the REQUEST, and on a mismatch it cannot be trusted to
		 * describe what raw_info actually is. */
		console->warn("ccmpPreview: requested mode {}x{} does not match the configured raw "
					   "stream {}x{}; refusing — preview stays uncorrected.",
					   requested_mode.width, requested_mode.height, raw_info.width, raw_info.height);
		return;
	}

	if (lores_info.pixel_format != libcamera::formats::YUV420)
	{
		console->warn("ccmpPreview: lores stream is {} not YUV420; preview stays uncorrected",
					  lores_info.pixel_format.toString());
		return;
	}

	RawFormat raw_format;
	if (!lookupRawFormat(raw_info.pixel_format, raw_format))
	{
		console->warn("ccmpPreview: raw stream is {}, which this stage cannot address as "
					  "16-bit samples; preview stays uncorrected",
					  raw_info.pixel_format.toString());
		return;
	}

	if (requested_mode.bit_depth == 12)
		configure12Bit(raw_info, lores_info, raw_format, requested_mode);
	else
		configure16Bit(raw_info, lores_info, raw_format);
}

void ccmpPreviewStage::configure12Bit(const StreamInfo &raw_info, const StreamInfo &lores_info,
									   const RawFormat &raw_format, const Mode &requested_mode)
{
	/* Gate 2 — a MEASURED decompand table for this mode's binning. Same call
	 * and same refusal as the encoder: a binning factor with no measured anchor
	 * is an unvalidated mode, and the register-only curve is wrong by 21 L
	 * through the mid-tones. Keyed on the actual configured raw stream
	 * (raw_info), not the requested mode — see SensorBinning()'s comment. */
	const double binning = static_cast<CinePIRecorder *>(app_)->SensorBinning(raw_info.width, raw_info.height);
	std::string err;
	const CcmpLut *lut = get_ccmp_lut(binning, err);
	if (!lut)
	{
		console->warn("ccmpPreview: no CCMP decompand table: {}. Preview stays magenta.", err);
		return;
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

	if (!renderer_.configure(geom, *lut, &err))
	{
		console->warn("ccmpPreview: {}. Preview stays magenta.", err);
		return;
	}

	/* The display's matrix, read off the stream rather than assumed. */
	colour_.rec709 = lores_info.colour_space &&
					 lores_info.colour_space->ycbcrEncoding == libcamera::ColorSpace::YcbcrEncoding::Rec709;

	raw_bytes_ = static_cast<size_t>(raw_info.stride) * raw_info.height;
	lores_bytes_ = static_cast<size_t>(lores_info.stride) * lores_info.height * 3 / 2;

	/* Shadow mode: every quad quadRgb() touches is also fed to detector_, but
	 * nothing here reads the result back into the render — see
	 * CcmpPreviewRenderer::setPlateauDetector()'s comment. Answers, without
	 * risking the picture, whether the fixed per-binning anchor still matches
	 * the hardware at gains other than the one it was measured at. */
	detector_.configure(12);
	renderer_.setPlateauDetector(&detector_);

	mode_ = ClearHdrMode::k12Bit;
	enabled_ = true;

	/* setColour() so highlightReference() below reports the level this take will
	 * actually use, rather than the previous configure's. Process() sets it
	 * again per frame for the live AWB gains. */
	renderer_.setColour(colour_);
	renderer_.resetMaxCode();
	frames_since_report_ = 0;

	console->info("ccmpPreview: 12-bit {} -> {}x{} preview, b={}, exposure {:.2f} gamma {:.2f} "
				  "highlightRolloff {:.3f} clip anchor {} (desaturation from {:.4f} of "
				  "full scale) {}",
				  lut->params().describe(), geom.out_width, geom.out_height,
				  static_cast<long long>(binning), colour_.exposure, colour_.gamma,
				  colour_.highlight_rolloff, renderer_.resolvedClipCode(),
				  renderer_.highlightReference() * (1.0 - colour_.highlight_rolloff),
				  colour_.rec709 ? "Rec709" : "Rec601");
}

void ccmpPreviewStage::configure16Bit(const StreamInfo &raw_info, const StreamInfo &lores_info,
									   const RawFormat &raw_format)
{
	if (raw_format.mono)
	{
		console->info("ccmpPreview: 16-bit ClearHDR mono has no CFA, no measured gains and no "
					  "colour cast to neutralise; ISP preview kept as is");
		return;
	}
	if (raw_format.container != 16)
	{
		/* requested_mode.bit_depth == 16 got us into this branch, but the
		 * ACTUAL raw stream reporting a 12-bit container means the request
		 * and the configured stream disagree — the same "invalid combo"
		 * shape as the width/height check above, just on bit depth instead
		 * of dimensions. raw_shift has to be exactly 0 for the neutraliser
		 * (detail: the container IS the sensor code, nothing to shift off),
		 * so this is refused rather than guessed at. */
		console->warn("ccmpPreview: 16-bit ClearHDR raw stream reports a {}-bit container, not "
					  "16; refusing to assume raw_shift 0 — preview stays uncorrected",
					  raw_format.container);
		return;
	}

	CcmpPreviewGeometry geom;
	geom.raw_width = raw_info.width;
	geom.raw_height = raw_info.height;
	geom.raw_stride = raw_info.stride;
	geom.raw_shift = 0; /* container == 16, mode == 16: the container IS the code */
	std::memcpy(geom.cfa, raw_format.cfa, sizeof(geom.cfa));
	geom.mono = raw_format.mono; /* false — checked above */
	geom.out_width = lores_info.width;
	geom.out_height = lores_info.height;
	geom.out_stride = lores_info.stride;

	std::string err;
	if (!neutraliser_.configure(geom, &err))
	{
		console->warn("ccmpPreview: {}. Preview stays uncorrected.", err);
		return;
	}

	/* 65535 is white to the ISP only when the lores colour space says so;
	 * otherwise studio range's 235 is. Read off the stream, not assumed —
	 * same reasoning as colour_.rec709 in the 12-bit path. */
	const bool full_range =
		lores_info.colour_space && lores_info.colour_space->range == libcamera::ColorSpace::Range::Full;
	white_ = full_range ? 255 : 235;
	neutraliser_.setWhite(white_);

	detector_.configure(16);
	auto_anchor_ = 0;
	have_last_gain_ = false;
	low_gain_logged_ = false;
	last_result_ = ClipPlateauDetector::Result{};

	raw_bytes_ = static_cast<size_t>(raw_info.stride) * raw_info.height;
	lores_bytes_ = static_cast<size_t>(lores_info.stride) * lores_info.height * 3 / 2;

	mode_ = ClearHdrMode::k16Bit;
	enabled_ = true;
	frames_since_report_ = 0;

	console->info("ccmpPreview: 16-bit ClearHDR {}x{} -> {}x{}, keeping the ISP render and "
				  "neutralising the merge-clamp zone in place; white {} ({}), highlightRolloff "
				  "{:.3f}, clip anchor {} ({})",
				  raw_info.width, raw_info.height, geom.out_width, geom.out_height, white_,
				  full_range ? "full range" : "limited range", colour_.highlight_rolloff,
				  colour_.sensor_clip_code, colour_.sensor_clip_code ? "override" : "auto, not yet measured");
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

	/* The gains the pipeline is actually using. Per frame, because AWB moves.
	 * Needed by both paths: the 12-bit renderer's matrix, and 16-bit's
	 * low-gain auto-detection skip. */
	if (auto cg = completed_request->metadata.get(libcamera::controls::ColourGains); cg)
	{
		colour_.r_gain = (*cg)[0];
		colour_.b_gain = (*cg)[1];
	}

	BufferReadSync rr(app_, raw_it->second);
	BufferWriteSync lw(app_, lores_it->second);
	libcamera::Span<uint8_t> raw = rr.Get()[0];
	libcamera::Span<uint8_t> lores = lw.Get()[0];

	/* Short buffers would be read or written past the end, and this runs on the
	 * post-processing thread where that is a silent heap corruption rather than
	 * a visible glitch. Configure() derived both sizes from the stream info; if
	 * the actual plane disagrees, the stream is not what it said it was and
	 * rendering from it is guesswork. Shared by both paths — 16-bit reads and
	 * writes exactly the same two buffers the 12-bit renderer does. */
	if (raw.size() < raw_bytes_ || lores.size() < lores_bytes_)
	{
		console->warn("ccmpPreview: buffers are raw {} / lores {} bytes, need {} / {}; disabled",
					  raw.size(), lores.size(), raw_bytes_, lores_bytes_);
		enabled_ = false;
		return false;
	}

	if (mode_ == ClearHdrMode::k12Bit)
		return process12Bit(raw.data(), lores.data(), completed_request->metadata);
	else
		return process16Bit(raw.data(), lores.data(), completed_request->metadata);
}

bool ccmpPreviewStage::process12Bit(const uint8_t *raw, uint8_t *lores, const libcamera::ControlList &metadata)
{
	/* The CCM the pipeline is actually using. Per frame, same reasoning as
	 * colour_.r_gain/b_gain above: AWB and the CCM solve together and both
	 * move. 16-bit does not need this — the neutraliser blends the ISP's own
	 * already-CCM'd YUV toward neutral, it never rebuilds RGB from raw. */
	if (auto m = metadata.get(libcamera::controls::ColourCorrectionMatrix); m)
		for (int i = 0; i < 9; ++i)
			colour_.ccm[i] = (*m)[i];

	renderer_.setColour(colour_);
	renderer_.render(raw, lores);

	if (++frames_since_report_ >= kMaxCodeReportFrames)
	{
		/* Shadow measurement: never adopted, never gates the render above —
		 * see configure12Bit()'s comment. detect() leaves shadow at zeros
		 * when the frame had nothing blown in it, which is itself the
		 * correct thing to print (not a stale number from the last one that
		 * was). */
		ClipPlateauDetector::Result shadow;
		detector_.detect(shadow);

		/* The two lines the hardware log quotes verbatim
		 * ("peak raw code ... highest uncorrected ... quads fully
		 * desaturated ... (clip anchor N)") are APPENDED to, never reworded —
		 * only the ", auto floor N, would anchor M" clause is new. */
		console->info("ccmpPreview: peak raw code {}, highest uncorrected {}, {} quads fully "
					  "desaturated, over the last {} frames (clip anchor {}), auto floor {}, "
					  "would anchor {}",
					  renderer_.maxCodeSeen(), renderer_.maxUndesaturatedCode(),
					  renderer_.fullyDesaturated(), frames_since_report_, renderer_.resolvedClipCode(),
					  shadow.floor, shadow.anchor);
		renderer_.resetMaxCode();
		detector_.reset();
		frames_since_report_ = 0;
	}
	return false;
}

bool ccmpPreviewStage::process16Bit(const uint8_t *raw, uint8_t *lores, const libcamera::ControlList &metadata)
{
	const unsigned override_code = colour_.sensor_clip_code;

	if (override_code != 0)
	{
		/* Override: no auto-detection at all, so no gain-change reset and no
		 * low-gain skip either — there is nothing for either to protect. */
		neutraliser_.setAnchor(override_code, colour_.highlight_rolloff);
		neutraliser_.apply(raw, lores, nullptr);
	}
	else
	{
		/* The clamp moves with gain (2026-09-13: 54100 at code 71, 48600 at
		 * code 80). A stale auto-adopted anchor after a gain change would
		 * whiten highlights that are no longer clipped until the next blown
		 * area re-measures it; "off" costs nothing when nothing is clipped,
		 * so reset rather than try to guess the new anchor from the old one. */
		if (auto ag = metadata.get(libcamera::controls::AnalogueGain); ag)
		{
			const float g = *ag;
			if (have_last_gain_ && g != last_analogue_gain_ && auto_anchor_ != 0)
			{
				console->info("ccmpPreview: AnalogueGain {:.3f} -> {:.3f}; clamp anchor reset to off",
							  last_analogue_gain_, g);
				auto_anchor_ = 0;
			}
			last_analogue_gain_ = g;
			have_last_gain_ = true;
		}

		const double max_gain = std::max(colour_.r_gain, colour_.b_gain);
		const bool skip_auto = max_gain < kMinGainForAutoDetect;
		if (skip_auto)
		{
			if (!low_gain_logged_)
			{
				console->info("ccmpPreview: gains r={:.2f} b={:.2f} below {:.2f}; a neutral "
							  "subject is equal-code too at near-unity gain, skipping clamp "
							  "auto-detection until they rise",
							  colour_.r_gain, colour_.b_gain, kMinGainForAutoDetect);
				low_gain_logged_ = true;
			}
		}
		else
			low_gain_logged_ = false;

		neutraliser_.setAnchor(auto_anchor_, colour_.highlight_rolloff);

		if (skip_auto)
			neutraliser_.apply(raw, lores, nullptr);
		else
		{
			/* Per frame: reset, apply with the PREVIOUS frame's anchor
			 * (setAnchor() above already used auto_anchor_ as it stood before
			 * this block), then measure THIS frame for the NEXT one. One
			 * frame of latency is invisible; the thumbnail is written long
			 * after the preview has settled. */
			detector_.reset();
			neutraliser_.apply(raw, lores, &detector_);

			ClipPlateauDetector::Result result;
			if (detector_.detect(result))
			{
				last_result_ = result;
				const unsigned delta = auto_anchor_ > result.anchor ? auto_anchor_ - result.anchor
																	 : result.anchor - auto_anchor_;
				if (auto_anchor_ == 0 || delta > kAnchorHysteresis)
					auto_anchor_ = result.anchor;
			}
			else
				last_result_ = result; /* converged/sampled still meaningful at 0 */
		}
	}

	if (++frames_since_report_ >= kMaxCodeReportFrames)
	{
		const unsigned anchor_in_use = override_code ? override_code : auto_anchor_;
		const char *anchor_mode = override_code ? "override" : (auto_anchor_ ? "auto" : "off");
		console->info("ccmpPreview: clamp anchor {} ({}), plateau floor {} from {} converged quads "
					  "of {}, peak raw code {}, highest uncorrected {}, {} quads fully desaturated, "
					  "over the last {} frames",
					  anchor_in_use, anchor_mode, last_result_.floor, last_result_.converged,
					  last_result_.sampled, neutraliser_.maxCodeSeen(), neutraliser_.maxUndesaturatedCode(),
					  neutraliser_.fullyDesaturated(), frames_since_report_);
		neutraliser_.resetMaxCode();
		frames_since_report_ = 0;
	}
	return false;
}

static PostProcessingStage *Create(RPiCamApp *app)
{
	return new ccmpPreviewStage(app);
}

static RegisterStage reg(NAME, &Create);
