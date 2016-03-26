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

add_executable(UnitTests ${TEST_FILES})

target_link_libraries(UnitTests gtest gtest_main)

add_test(UnitTests UnitTests)

# custom target for running the tests
add_custom_target(run_tests ALL COMMAND "./UnitTests")
