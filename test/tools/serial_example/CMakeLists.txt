cmake_minimum_required(VERSION 2.8)
project(sensor_reader)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++14")
# Platform specific stuff
if (APPLE)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -framework IOKit -framework CoreFoundation")
endif()

set(INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/../../../src"
                 "${PROJECT_SOURCE_DIR}/../../../include"
                 "${PROJECT_SOURCE_DIR}/../../../third-party/install/libserialport/include"
                 "${PROJECT_SOURCE_DIR}/../../../third-party/install/liblo/include")

include_directories(${INCLUDE_DIRS})

#################################
#  Statically linked libraries  #
#################################

set(LINK_LIBRARIES "${PROJECT_SOURCE_DIR}/../../../third-party/install/libserialport/lib/libserialport.a"
                   "${PROJECT_SOURCE_DIR}/../../../third-party/install/liblo/lib/liblo.a"
                   pthread)

# List all source files here
set(SOURCE_FILES serial_example.cpp
                 Serial.cpp
                 Serial.h
                 types.h)

add_executable(serial_example ${SOURCE_FILES})
target_link_libraries(serial_example ${LINK_LIBRARIES})

