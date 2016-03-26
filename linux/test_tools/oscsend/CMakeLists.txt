cmake_minimum_required(VERSION 2.8)
project(sensor_reader)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

# Build all included third party dependecies
#add_subdirectory("${PROJECT_SOURCE_DIR}/../../third-party")

# Add to here if you need to include say third-party/xxx/include
set(INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/../../include "
                 "${PROJECT_SOURCE_DIR}/../../third-party/install/liblo/include ")

set(LINK_LIBRARIES "${PROJECT_SOURCE_DIR}../../third-party/install/liblo/lib/liblo.a ")

# List all source files here
set(SOURCE_FILES osc_send_test.cpp)

message("Source files: ")
message("" ${SOURCE_FILES})
message("" ${INCLUDE_DIRS})

include_directories(${INCLUDE_DIRS})
add_executable(osc_send_test ${SOURCE_FILES})
target_link_libraries(osc_send_test  ${LINK_LIBRARIES})
