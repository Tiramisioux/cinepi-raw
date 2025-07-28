# cinepi/meson.build  ---------------------------------------------------
# This file is included from the top-level rpicam-apps project, so **do
# NOT** call project() again.

# ---------------------------------------------------------------
# Optional lgpio support.  We detect it, then add the macro and
# include path to every target that needs it, without touching the
# global project state (no add_project_arguments()).
# ---------------------------------------------------------------
lgpio_dep = dependency('lgpio', required : false)
have_lgpio = lgpio_dep.found()

if not have_lgpio
  cpp = meson.get_compiler('cpp')
  lgpio_dep = cpp.find_library('lgpio',
                               dirs : ['/usr/local/lib', '/usr/lib', '/usr/lib64'],
                               required : false)
  have_lgpio = lgpio_dep.found()
endif

common_cpp_args     = []
common_includes     = []
if have_lgpio
  common_cpp_args  += ['-DHAVE_LGPIO']
  # Joan’s “make install” puts the header in /usr/local/include
  common_includes  += include_directories('/usr/local/include')
endif

# ---------------------------------------------------------------
# Optional Redis support (already used elsewhere in the project)
# ---------------------------------------------------------------
hiredis_dep         = dependency('hiredis',  required : false)
redis_plus_plus_dep = dependency('redis++', required : false)

if not hiredis_dep.found()
  hiredis_dep = meson.get_compiler('cpp').find_library('hiredis', required : true)
  common_includes += include_directories('/usr/include')   # adjust if custom path
endif

if not redis_plus_plus_dep.found()
  redis_plus_plus_dep = meson.get_compiler('cpp').find_library('redis++', required : true)
  common_includes += include_directories('/usr/include')
endif

# ---------------------------------------------------------------
# Third-party deps provided by the parent meson.build
# (tiff_dep, boost_dep, thread_dep, libcamera_dep, rpicam_app_dep, etc.)
# ---------------------------------------------------------------
spdlog_dep = dependency('spdlog')
json_dep   = dependency('jsoncpp')
alsa_dep   = dependency('alsa')
udev_dep   = dependency('libudev')
mhd_dep    = dependency('nadjieb_mjpeg_streamer', required : true)

cinepi_raw_dep = [
  tiff_dep,
  hiredis_dep,
  redis_plus_plus_dep,
  boost_dep,
  thread_dep,
  libcamera_dep,
  rpicam_app_dep,
  spdlog_dep,
  json_dep,
  mhd_dep,
  alsa_dep,
  udev_dep,
  lgpio_dep,
]

# ---------------------------------------------------------------
# Source files
# ---------------------------------------------------------------
sources = [
  'cinepi_raw.cpp',
  'dng_encoder.cpp',
  'sharedContextStage.cpp',
  'mjpegPreviewStage.cpp',
  'cinepi_controller.cpp',
  'cinepi_sound.cpp',
  'cinepi_state.cpp',
  'utils.cpp',
  'cinepi_options.cpp',
]

cinepi_raw = executable(
  'cinepi-raw',
  sources,
  include_directories : [include_directories('..')] + common_includes,
  cpp_args            : common_cpp_args,
  dependencies        : cinepi_raw_dep,
  link_with           : rpicam_app,
  install             : true,
)

# ---------------------------------------------------------------
# Stand-alone hardware-sync helper
# ---------------------------------------------------------------
cinepi_hw_sync = executable(
  'cinepi-hw-sync',
  'cinepi_hw_sync.cpp',
  include_directories : [include_directories('..')] + common_includes,
  cpp_args            : common_cpp_args,
  dependencies        : [lgpio_dep, dependency('threads'), spdlog_dep],
  install             : true,
)
# ----------------------------------------------------------------
