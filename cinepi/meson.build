hiredis_dep = dependency('hiredis', required : false) # If not found, it might not be in pkg-config; you can use cc.find_library() instead.
redis_plus_plus_dep = dependency('redis++', required : false) # Same note as hiredis.


# If hiredis or redis++ aren't found by pkg-config, use find_library and include_directories.
if not hiredis_dep.found()
  hiredis_lib = meson.get_compiler('cpp').find_library('hiredis', required: true)
  hiredis_header = include_directories('/path/to/hiredis/includes') # Replace with the correct path if it's not system-wide.
else
  hiredis_lib = hiredis_dep
  hiredis_header = []
endif

if not redis_plus_plus_dep.found()
  redis_plus_plus_lib = meson.get_compiler('cpp').find_library('redis++', required: true)
  redis_plus_plus_header = include_directories('/path/to/redis++/includes') # Replace with the correct path if it's not system-wide.
else
  redis_plus_plus_lib = redis_plus_plus_dep
  redis_plus_plus_header = []
endif

# 'mjpegStreamStage.cpp',
# Sources
sources = [
  'cinepi_raw.cpp',
  'dng_encoder.cpp',
  'ccmp_lut.cpp',
  'sharedContextStage.cpp',
  'ccmpPreviewStage.cpp',
  'mjpegPreviewStage.cpp',
  'dualHdmiPreviewStage.cpp',
  'cinepi_controller.cpp',
  'cinepi_sound.cpp',
  'cinepi_state.cpp',
  'utils.cpp',
  'cinepi_options.cpp',
  'log_lut.cpp',
]


spdlog_dep = dependency('spdlog')
json_dep = dependency('jsoncpp')

cinepi_raw_dep = [tiff_dep, hiredis_lib, redis_plus_plus_lib, boost_dep, thread_dep, libcamera_dep, rpicam_app_dep, spdlog_dep, json_dep]

alsa_dep = dependency('alsa')
udev_dep = dependency('libudev')
# nadjieb's cpp-mjpeg-streamer used to be an external dependency, built by
# the installer's build_cpp_mjpeg_streamer step and found here as
# dependency('nadjieb_mjpeg_streamer'). It is now vendored (and patched) at
# cinepi/third_party/nadjieb/mjpeg_streamer.hpp -- see that file's own
# "Vendored into cinepi-raw" header comment for the upstream commit and the
# patch -- so no pkg-config dependency remains; mjpegPreviewStage.cpp finds
# it via the include directory added below. thread_dep (already in
# cinepi_raw_dep) covers the header's std::thread usage.
cinepi_raw_dep += alsa_dep
cinepi_raw_dep += udev_dep
# The DNG thumbnail's JPEG mode (dng_encoder.cpp, thumbnail=3): jpeg_dep is
# declared once, in image/meson.build (subdir('image') runs before
# subdir('cinepi') in the root meson.build), where it is also folded into
# rpicam_app_dep for encoder/mjpeg_encoder.cpp. dng_encoder.cpp is compiled
# as one of THIS executable's own sources (not through rpicam_app), so its
# #include <jpeglib.h> needs jpeg_dep on cinepi_raw_dep directly too.
cinepi_raw_dep += jpeg_dep

cinepi_raw = executable('cinepi-raw', sources,
             include_directories : [include_directories('..'), include_directories('third_party')],
             dependencies: cinepi_raw_dep,
             link_with : rpicam_app,
             install : true)

executable('cinepi-audio-capture',
           'cinepi_audio_capture.cpp',
           dependencies : [alsa_dep],
           install : true)

# Unit tests for the pure frame-rate phase-lock control core. No libcamera / redis
# deps — it includes only cinepi/phase_lock_core.hpp. Run with `meson test`.
phase_lock_core_test = executable('phase_lock_core_test',
           '../tests/phase_lock_core_test.cpp',
           include_directories : [include_directories('..')],
           install : false)
test('phase_lock_core', phase_lock_core_test)

# Unit tests for the pure DNG pixel pack/unpack helpers. No libcamera deps — it
# includes only cinepi/dng_pack.hpp. Run with `meson test`.
dng_pack_test = executable('dng_pack_test',
           '../tests/dng_pack_test.cpp',
           include_directories : [include_directories('..')],
           install : false)
test('dng_pack', dng_pack_test)

# Unit tests for the linear DNG output-depth rule (which code depth the raw
# strip is packed at, and the shift that gets there). No libcamera deps — it
# includes only cinepi/dng_output_depth.hpp. Run with `meson test`.
dng_output_depth_test = executable('dng_output_depth_test',
           '../tests/dng_output_depth_test.cpp',
           include_directories : [include_directories('..')],
           install : false)
test('dng_output_depth', dng_output_depth_test)

# Unit tests for the binning-source decision (WP-CPR-2 / finding C2): whether
# the sensor driver's own "Mode Binning" control or the active-area/delivered
# -size ratio is trusted. No libcamera deps — it includes only
# cinepi/sensor_binning_source.hpp. Run with `meson test`.
sensor_binning_source_test = executable('sensor_binning_source_test',
           '../tests/sensor_binning_source_test.cpp',
           include_directories : [include_directories('..')],
           install : false)
test('sensor_binning_source', sensor_binning_source_test)

# Unit tests for IFDBuilder's multi-IFD chaining contract (C9 Phase 0's
# embedded DNG thumbnail). No libcamera deps — it includes only
# cinepi/ifd_builder.hpp. Run with `meson test`.
ifd_builder_test = executable('ifd_builder_test',
           '../tests/ifd_builder_test.cpp',
           include_directories : [include_directories('..')],
           install : false)
test('ifd_builder', ifd_builder_test)

# Unit tests for the DNG embedded-thumbnail geometry formula -- the single
# source setup_encoder()'s buffer reservation and dng_save()'s IFD1 write
# both call. No libcamera deps — it includes only cinepi/dng_thumbnail.hpp.
# Run with `meson test`.
dng_thumbnail_test = executable('dng_thumbnail_test',
           '../tests/dng_thumbnail_test.cpp',
           include_directories : [include_directories('..')],
           install : false)
test('dng_thumbnail', dng_thumbnail_test)

# Unit tests for the CineMate Log curve engine. No libcamera / jsoncpp deps — it
# includes only cinepi/log_lut.hpp (the spec loader in log_lut.cpp is what needs
# jsoncpp, and it is not part of the tested core). Run with `meson test`.
log_lut_test = executable('log_lut_test',
           '../tests/log_lut_test.cpp',
           include_directories : [include_directories('..')],
           install : false)
test('log_lut', log_lut_test)

# Unit tests for --log-encode argv parsing. No libcamera / boost deps — it
# includes only cinepi/log_encode_arg.hpp, which is where the bare-flag lookahead
# rules live precisely so they can be tested off-device. Run with `meson test`.
log_encode_arg_test = executable('log_encode_arg_test',
           '../tests/log_encode_arg_test.cpp',
           include_directories : [include_directories('..')],
           install : false)
test('log_encode_arg', log_encode_arg_test)

# Unit tests for the CCMP12 decompand curve. No libcamera deps — it includes only
# cinepi/ccmp_lut.hpp. The test compares the built table against the golden ones
# emitted by innomaker585/ccmp12-lut/tools/ccmp_decode.py, so it needs their path
# at compile time. Run with `meson test`.
ccmp_lut_test = executable('ccmp_lut_test',
           '../tests/ccmp_lut_test.cpp',
           include_directories : [include_directories('..')],
           cpp_args : ['-DCCMP_GOLDEN_DIR="' + meson.current_source_dir() / '../resources/ccmp_luts' + '"'],
           install : false)
test('ccmp_lut', ccmp_lut_test)

# Unit tests for the CCMP12 decompand composed with CineMate Log (12-bit
# ClearHDR log-encode, P3). No libcamera / jsoncpp deps — it includes only
# cinepi/log_lut.hpp (which pulls in cinepi/ccmp_lut.hpp). The test compares
# the composed table against the golden ones emitted by
# innomaker585/ccmp12-lut/tools/compose_log.py, so it needs their path at
# compile time. Run with `meson test`.
ccmp_log_compose_test = executable('ccmp_log_compose_test',
           '../tests/ccmp_log_compose_test.cpp',
           include_directories : [include_directories('..')],
           cpp_args : ['-DCCMP_LOG_GOLDEN_DIR="' + meson.current_source_dir() / '../resources/ccmp_log_luts' + '"'],
           install : false)
test('ccmp_log_compose', ccmp_log_compose_test)

# Unit tests for the CCMP12 preview renderer — the raw-Bayer path that keeps the
# HDMI/MJPEG previews out of the magenta the DNG's LinearizationTable already
# fixes. No libcamera deps: ccmp_preview.hpp is pure by construction, which is
# what lets the decompand-before-gains ordering be asserted off-device.
ccmp_preview_test = executable('ccmp_preview_test',
           '../tests/ccmp_preview_test.cpp',
           include_directories : [include_directories('..')],
           install : false)
test('ccmp_preview', ccmp_preview_test)

# Unit tests for the CCMP12 attach gate (cinepi/ccmp_gate.hpp) -- whether
# dng_encoder.cpp's setup_encoder() should even consider a decompand table,
# including the round-2 sensor_mode_trusted refusal. No libcamera deps.
ccmp_gate_test = executable('ccmp_gate_test',
           '../tests/ccmp_gate_test.cpp',
           include_directories : [include_directories('..')],
           install : false)
test('ccmp_gate', ccmp_gate_test)

# Unit tests for the ClearHDR clamp detector (cinepi/clip_ceiling.hpp) -- the
# measurement dng_save() writes into WhiteLevel so a blown ClearHDR highlight
# renders white instead of magenta. No libcamera deps. The fixtures are the
# per-channel code histograms of four recorded DNGs, so it needs their path at
# compile time. Run with `meson test`.
clip_ceiling_test = executable('clip_ceiling_test',
           '../tests/clip_ceiling_test.cpp',
           include_directories : [include_directories('..')],
           cpp_args : ['-DCLIP_CEILING_DATA_DIR="' + meson.current_source_dir() / '../tests/data' + '"'],
           install : false)
test('clip_ceiling', clip_ceiling_test)

# Unit tests for the static-response branch patched into the vendored MJPEG
# streamer (cinepi/third_party/nadjieb/mjpeg_streamer.hpp) -- the clean
# preview's black index page at "/". No libcamera / Redis deps and no
# sockets opened; it includes only the vendored header. Run with
# `meson test`.
mjpeg_static_response_test = executable('mjpeg_static_response_test',
           '../tests/mjpeg_static_response_test.cpp',
           include_directories : [include_directories('..'), include_directories('third_party')],
           install : false)
test('mjpeg_static_response', mjpeg_static_response_test)

# Unit test for MJPEGStreamer::clientCount(), added on
# fix/mjpeg-worker-exhaustion so mjpegPreviewStage.cpp can log the
# registered-client count periodically. Pure / self-contained, same shape as
# mjpeg_static_response_test above -- see tests/mjpeg_client_count_test.cpp
# for what it does and does not cover (socket lifecycle is not unit-testable
# here; that part was checked with a standalone desk harness instead).
mjpeg_client_count_test = executable('mjpeg_client_count_test',
           '../tests/mjpeg_client_count_test.cpp',
           include_directories : [include_directories('..'), include_directories('third_party')],
           install : false)
test('mjpeg_client_count', mjpeg_client_count_test)
