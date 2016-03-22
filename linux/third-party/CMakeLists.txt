cmake_minimum_required(VERSION 2.8)
project(sensor_reader)

message("Building third party dependecies")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

include(ExternalProject)
ExternalProject_Add(libserialport
        SOURCE_DIR ${PROJECT_SOURCE_DIR}/libserialport
        BUILD_IN_SOURCE 1
        CONFIGURE_COMMAND ./autogen.sh COMMAND ./configure
        BUILD_COMMAND make
        INSTALL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/install
        INSTALL_COMMAND ""
        )

ExternalProject_Add(liblo
        SOURCE_DIR ${PROJECT_SOURCE_DIR}/liblo
        BUILD_IN_SOURCE 1
        CONFIGURE_COMMAND "./autogen.sh --enable-static" COMMAND ./configure
        BUILD_COMMAND make
        INSTALL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/install
        INSTALL_COMMAND ""
        )

