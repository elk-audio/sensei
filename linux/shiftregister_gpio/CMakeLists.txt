cmake_minimum_required(VERSION 3.8)
project(shiftreg_gpio)

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
set(SHIFTREG_GPIO_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/include"
                               "${PROJECT_SOURCE_DIR}/src"
                               "${CMAKE_SOURCE_DIR}/src"
                               "${CMAKE_SOURCE_DIR}/third-party/spdlog/include")

####################
# Lib Source Files #
####################
set(SHIFTREG_GPIO_LIB_SOURCES src/shiftreg_gpio.cpp)

###########
# Targets #
###########
add_library(shiftreg_gpio STATIC ${SHIFTREG_GPIO_LIB_SOURCES})
add_xenomai_to_target(shiftreg_gpio)
target_include_directories(shiftreg_gpio PRIVATE ${SHIFTREG_GPIO_INCLUDE_DIRS})
target_link_libraries(shiftreg_gpio PRIVATE pthread fifo gpio_protocol gpio_protocol_client)
target_compile_definitions(shiftreg_gpio PRIVATE -DWITH_GPIO_LOGIC)

# Board test
add_executable(elk_pi_hw_test test/elk_pi_hw_test.cpp)
add_xenomai_to_target(elk_pi_hw_test)
target_link_libraries(elk_pi_hw_test PRIVATE pthread)
target_include_directories(elk_pi_hw_test PRIVATE "${PROJECT_SOURCE_DIR}/src")