cmake_minimum_required(VERSION 3.8)
project(rpi_shiftreg_gpio)

#######################
#  Cross Compilation  #
#######################

set(XENOMAI_BASE_DIR "/usr/xenomai" CACHE STRING "xenomai base dir path")

if(NOT "$ENV{CMAKE_SYSROOT}" STREQUAL "")
    set(CMAKE_SYSROOT "$ENV{CMAKE_SYSROOT}")
    message("ENV_CMAKE_SYSROOT = " $ENV{CMAKE_SYSROOT})
endif()
if(NOT "${CMAKE_SYSROOT}" STREQUAL "")
    set(XENOMAI_BASE_DIR "${CMAKE_SYSROOT}/usr/xenomai")
    message("XENOMAI_BASE_DIR is " ${XENOMAI_BASE_DIR})
endif()

#######################
#  Utility functions  #
#######################

# Set Xenomai build options at local scope
function(add_xenomai_to_target target)
    set(XENOMAI_C_FLAGS "-D_GNU_SOURCE -D_REENTRANT -D__COBALT__ -D__COBALT_WRAP__")

    set(XENOMAI_INCLUDE_DIRS ${XENOMAI_BASE_DIR}/include
                             ${XENOMAI_BASE_DIR}/include/cobalt)

    find_library(COBALT_LIB cobalt HINTS ${XENOMAI_BASE_DIR}/lib)
    target_compile_options(${target} PRIVATE ${XENOMAI_C_FLAGS})
    target_include_directories(${target} PRIVATE ${XENOMAI_INCLUDE_DIRS})
    target_link_libraries(${target} PRIVATE ${COBALT_LIB} rt m)
endfunction()

#######################
# Include Directories #
#######################
set(ELK_PI_GPIO_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/include"
                             "${PROJECT_SOURCE_DIR}/src"
                             "${CMAKE_SOURCE_DIR}/src"
                             "${CMAKE_SOURCE_DIR}/third-party/spdlog/include")

####################
# Lib Source Files #
####################
set(ELK_PI_GPIO_LIB_SOURCES src/rpi_shiftreg_gpio.cpp)

###########
# Targets #
###########
add_library(rpi_shiftreg_gpio STATIC ${ELK_PI_GPIO_LIB_SOURCES})
add_xenomai_to_target(rpi_shiftreg_gpio)
target_include_directories(rpi_shiftreg_gpio PRIVATE ${ELK_PI_GPIO_INCLUDE_DIRS})
target_link_libraries(rpi_shiftreg_gpio PRIVATE pthread fifo gpio_protocol gpio_protocol_client)
target_compile_definitions(rpi_shiftreg_gpio PRIVATE -DWITH_ELK_PI_GPIO)