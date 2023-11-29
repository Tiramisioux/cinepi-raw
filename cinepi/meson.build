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

# Sources
sources = [
  'cinepi_raw.cpp',
  'dng_encoder.cpp',
  'mjpegStreamStage.cpp',
  'cinepi_controller.cpp',
  'cinepi_sound.cpp',
  'cinepi_state.cpp',
  'utils.cpp'
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



# Executable
cinepi_raw = executable('cinepi-raw', sources,
             include_directories : include_directories('..'),
             dependencies: cinepi_raw_dep,
             link_with : rpicam_app,
             install : true)

