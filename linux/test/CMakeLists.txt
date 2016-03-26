# build gtest framework
add_subdirectory(gtest_1_7_0)

SET(TEST_FILES unittests/sample_test.cpp
               unittests/locked_queue_test.cpp)

#include folder contains current project's header files
include_directories(${PROJECT_SOURCE_DIR}/test/gtest_1_7_0/include)
include_directories(${PROJECT_SOURCE_DIR}/src)

# set flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++0x")

enable_testing()

message("Unit test files: ")
message("" ${TEST_FILES})

add_executable(UnitTests ${TEST_FILES})

target_link_libraries(UnitTests gtest gtest_main ${PROJECT_SOURCE_DIR}/third-party/libserialport/.libs/libserialport.a)

add_test(UnitTests UnitTests)

# custom target for running the tests
add_custom_target(run_tests ALL COMMAND "./UnitTests")
