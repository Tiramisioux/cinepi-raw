startup_gate_test = executable('startup_gate_test',
    ['test_startup_gate.cpp', '../startup_gate.cpp'],
    include_directories : [include_directories('..')],
    dependencies : [spdlog_dep])

test('startup_gate', startup_gate_test)

sync_policy_test = executable('sync_policy_test',
    'test_sync_policy.cpp',
    include_directories : [include_directories('..', '../..')])

test('sync_policy', sync_policy_test)
