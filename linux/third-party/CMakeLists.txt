cmake_minimum_required(VERSION 2.8)
include(ExternalProject)

#####################################################
#  External Projects definitions (non-Cmake based)  #
#####################################################

ExternalProject_Add(libserialport
        SOURCE_DIR ${PROJECT_SOURCE_DIR}/third-party/libserialport
        BUILD_IN_SOURCE 1
        CONFIGURE_COMMAND ./autogen.sh COMMAND ./configure --prefix=${CMAKE_CURRENT_SOURCE_DIR}/install/libserialport
        BUILD_COMMAND make
        INSTALL_COMMAND make install
)

ExternalProject_Add(liblo
        SOURCE_DIR ${PROJECT_SOURCE_DIR}/third-party/liblo
        BUILD_IN_SOURCE 1
        CONFIGURE_COMMAND ./autogen.sh --enable-static --prefix=${CMAKE_CURRENT_SOURCE_DIR}/install/liblo
        BUILD_COMMAND make
        INSTALL_COMMAND make install
)

