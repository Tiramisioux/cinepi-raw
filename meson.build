project('rpicam-apps', 'c', 'cpp',
        meson_version : '>= 0.64.0',
        version : '1.7.0',
        default_options : [
            'werror=false',
            'warning_level=3',
            'cpp_std=c++17',
            'c_std=c11',
            'buildtype=release',
        ],
        license : 'BSD-2-Clause')

meson.add_dist_script('utils' / 'gen-dist.sh')

cpp_arguments = ['-pedantic', '-Wno-unused-parameter', '-faligned-new']

# Needed for file sizes > 32-bits.
cpp_arguments += '-D_FILE_OFFSET_BITS=64'

cxx = meson.get_compiler('cpp')
cpu = host_machine.cpu()
neon = get_option('neon_flags')

if cxx.get_id() == 'gcc'
    cpp_arguments += '-Wno-psabi'
endif

if cpu == 'aarch64' or neon == 'arm64'
    cpp_arguments += '-ftree-vectorize'
elif neon == 'armv8-neon'
    cpp_arguments += ['-mfpu=neon-fp-armv8', '-ftree-vectorize']
endif

libcamera_dep = dependency('libcamera', required : true)

summary({
            'location' : libcamera_dep.get_variable('libdir'),
            'version' : libcamera_dep.version()
        },
        section : 'libcamera')

rpicam_app_src = []
rpicam_app_dep = [libcamera_dep]     

subdir('core')
subdir('encoder')
subdir('image')
subdir('output')
subdir('preview')
subdir('post_processing_stages')
subdir('utils')

add_project_arguments(cpp_arguments, language : 'cpp')

rpicam_app = library(
    'rpicam_app',
    rpicam_app_src,
    soversion : meson.project_version(),
    include_directories : include_directories('.'),
    install : true,
    name_prefix : '',
    dependencies : rpicam_app_dep,
)

# Install a symlink to the old library name for legacy purposes.
install_symlink('libcamera_app.so',
                install_dir: get_option('libdir'),
                pointing_to: 'rpicam_app.so')

subdir('apps')
subdir('cinepi')

# CineMate Log curve specs, loaded at startup by cinepi/log_lut.cpp. Installed
# where find_log_lut_spec() looks first; running uninstalled out of the repo clone
# still works via its /home/pi and ./resources fallbacks.
install_data(['resources/log_luts/cinemate_log_16to12.json',
              'resources/log_luts/cinemate_log_16to10.json',
              'resources/log_luts/cinemate_log_12to10.json'],
             install_dir : get_option('datadir') / 'cinepi-raw' / 'log_luts')

summary({
            'libav encoder' : enable_libav,
            'drm preview' : enable_drm,
            'egl preview' : enable_egl,
            'qt preview' : enable_qt,
            'OpenCV postprocessing' : enable_opencv,
            'TFLite postprocessing' : enable_tflite,
        },
        bool_yn : true, section : 'Build configuration')
