add_executable(standalone_tests standalone_tests.cpp)
target_link_libraries(standalone_tests PRIVATE pin_ik)
add_test(NAME cpp_standalone COMMAND standalone_tests ${CMAKE_CURRENT_SOURCE_DIR}/robot.urdf)

# Hooks are absent from the installed library and production hot path.
add_library(pin_ik_worker_test_support STATIC
  ../src/pin_ik.cpp ../src/pinocchio_tl.cpp ../src/nlopt_ik.cpp ../src/urdf.cpp)
target_compile_definitions(pin_ik_worker_test_support PUBLIC PIN_IK_TESTING)
target_compile_features(pin_ik_worker_test_support PUBLIC cxx_std_17)
target_include_directories(pin_ik_worker_test_support PUBLIC ../include)
target_link_libraries(pin_ik_worker_test_support PUBLIC
  Eigen3::Eigen pinocchio::pinocchio ${NLOPT_LIBRARIES} Threads::Threads)
add_executable(worker_tests worker_tests.cpp)
target_link_libraries(worker_tests PRIVATE pin_ik_worker_test_support)
add_test(NAME persistent_workers COMMAND worker_tests ${CMAKE_CURRENT_SOURCE_DIR}/robot.urdf)
set_tests_properties(persistent_workers PROPERTIES TIMEOUT 180)
if(PIN_IK_BUILD_EXAMPLES)
  add_test(NAME example_random_ik COMMAND ik_tests ${CMAKE_CURRENT_SOURCE_DIR}/robot.urdf base tip 100)
endif()

# Diagnostic tools are built explicitly and are not automated regression tests.
add_subdirectory(manual)
