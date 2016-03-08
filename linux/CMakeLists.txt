cmake_minimum_required(VERSION 2.8)
project(sensor_reader)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

# Build all included third party dependecies
add_subdirectory(third-party)

# Add to here if you need to include say third-party/xxx/include
set(INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/src "
                 "${PROJECT_SOURCE_DIR}/include "
                 "${PROJECT_SOURCE_DIR}/third-party/libserialport ")

set(LINK_LIBRARIES "${PROJECT_SOURCE_DIR}/third-party/libserialport/.libs/libserialport.a ")

# List all source files here
set(SOURCE_FILES src/main.cpp)

message("Source files: ")
message("" ${SOURCE_FILES})

include_directories(${INCLUDE_DIRS})
add_executable(sensor_reader ${SOURCE_FILES})
target_link_libraries(sensor_reader  ${PROJECT_SOURCE_DIR}/third-party/libserialport/.libs/libserialport.a)

add_subdirectory(test)
