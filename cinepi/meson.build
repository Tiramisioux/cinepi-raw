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
mhd_dep = dependency('nadjieb_mjpeg_streamer', required : true)
cinepi_raw_dep += mhd_dep
cinepi_raw_dep += alsa_dep
cinepi_raw_dep += udev_dep

cinepi_raw = executable('cinepi-raw', sources,
             include_directories : [include_directories('..')],
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
