cmake_minimum_required(VERSION 2.8)
project(sensor_reader)

message("Building third party dependecies")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

include(ExternalProject)
ExternalProject_Add(libserialport
        SOURCE_DIR ${PROJECT_SOURCE_DIR}/libserialport
        BUILD_IN_SOURCE 1
        UPDATE_COMMAND ./autogen.sh
        CONFIGURE_COMMAND ./configure
        BUILD_COMMAND make
        INSTALL_DIR ${PROJECT_SOURCE_DIR}/install
        INSTALL_COMMAND ""
        )
