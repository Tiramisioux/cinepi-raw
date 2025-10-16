# Tests in this subdir need libcamera headers (included via core/options.hpp).
# Declare the deps locally (harmless even if top-level already did).
dep_libcamera       = dependency('libcamera', required: true)
dep_libcamera_base  = dependency('libcamera-base', required: true)
dep_threads         = dependency('threads')                 # std::thread if needed
dep_boost_progopts  = dependency('boost', modules : ['program_options'], required : false)

startup_gate_test = executable('startup_gate_test',
  ['test_startup_gate.cpp', '../startup_gate.cpp'],
  include_directories : [ include_directories('..', '../..') ],
  dependencies : [ spdlog_dep, dep_threads ]  # spdlog already defined by parent meson
)

test('startup_gate', startup_gate_test)

sync_policy_test = executable('sync_policy_test',
  'test_sync_policy.cpp',
  include_directories : [ include_directories('..', '../..') ],
  dependencies : [
    spdlog_dep,               # for your logging in the test
    dep_libcamera,            # pulls in -I for <libcamera/...>
    dep_libcamera_base,       # some headers/types come from base
    dep_threads,
    dep_boost_progopts        # only for headers; linking usually not required for this test
  ]
)

test('sync_policy', sync_policy_test)

dng_encoder_test = executable('dng_encoder_test',
    ['test_dng_encoder.cpp', '../dng_encoder.cpp', '../startup_gate.cpp'],
    include_directories : [include_directories('..', '../..')],
    dependencies : cinepi_raw_dep,
    link_with : rpicam_app)

test('dng_encoder', dng_encoder_test)
