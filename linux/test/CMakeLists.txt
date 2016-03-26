# build gtest framework
add_subdirectory(gtest_1_7_0)
enable_testing()

################
#  Test Files  #
################

SET(TEST_FILES unittests/sample_test.cpp
               unittests/locked_queue_test.cpp)

##########################################
#  Include directories relative to test  #
##########################################

include_directories(${PROJECT_SOURCE_DIR}/test/gtest_1_7_0/include)

add_executable(unit_tests ${TEST_FILES})

target_link_libraries(unit_tests gtest gtest_main)

add_test(unit_tests unit_tests)

# custom target for running the tests
add_custom_target(run_tests ALL COMMAND "./unit_tests")
add_dependencies(run_tests unit_tests)

