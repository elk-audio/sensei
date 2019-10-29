cmake_minimum_required(VERSION 3.8)
project(sensei)

##################################
#  Build  information            #
##################################

option(WITH_UNIT_TESTS "Build and run unittests" ON)

# Get the latest commit hash of the working branch
execute_process(
        COMMAND git log -1 --format=%H
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_COMMIT_HASH
        OUTPUT_STRIP_TRAILING_WHITESPACE
)

string(TIMESTAMP BUILD_TIMESTAMP "%Y-%m-%d %H:%M")

configure_file(
        ${CMAKE_SOURCE_DIR}/include/version.h.in
        ${CMAKE_BINARY_DIR}/generated/version.h
)

############################
#  Main executable target  #
############################

set(COMPILATION_UNITS src/config_backend/json_configuration.cpp
                      src/mapping/sensor_mappers.cpp
                      src/mapping/mapping_processor.cpp
                      src/event_handler.cpp
                      src/output_backend/std_stream_backend.cpp
                      src/output_backend/osc_backend.cpp
                      src/hardware_frontend/raspa_frontend.cpp
                      src/hardware_frontend/message_tracker.cpp
                      src/hardware_frontend/gpio_command_creator.cpp
                      src/main.cpp
                      src/logging.cpp
                      src/user_frontend/user_frontend.cpp src/user_frontend/osc_user_frontend.cpp)


# Enumerate all the headers separately so that CLion can index them
set(EXTRA_CLION_SOURCES src/config_backend/base_configuration.h
                        src/config_backend/json_configuration.h
                        src/message/base_message.h
                        src/message/base_value.h
                        src/message/base_command.h
                        src/message/base_error.h
                        src/message/error_defs.h
                        src/message/command_defs.h
                        src/message/value_defs.h
                        src/message/message_factory.h
                        src/mapping/mapping_processor.h
                        src/mapping/sensor_mappers.h
                        src/output_backend/output_backend.h
                        src/output_backend/std_stream_backend.h
                        src/output_backend/osc_backend.h
                        src/hardware_frontend/hw_frontend.h
                        src/hardware_frontend/message_tracker.h
                        src/hardware_frontend/raspa_frontend.h
                        src/hardware_frontend/gpio_command_creator.h
                        src/locked_queue.h
                        src/synchronized_queue.h
                        src/event_handler.h
                        src/utils.h
                        src/logging.h
                        src/user_frontend/user_frontend.h
                        src/user_frontend/osc_user_frontend.h)

set(SOURCE_FILES "${COMPILATION_UNITS}" "${EXTRA_CLION_SOURCES}")

add_executable(sensei ${SOURCE_FILES})

#########################
#  Include Directories  #
#########################

set(INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/src"
                 "${PROJECT_SOURCE_DIR}/include"
                 "${CMAKE_BINARY_DIR}"
                 "${PROJECT_SOURCE_DIR}/third-party/optionparser/"
                 "${PROJECT_SOURCE_DIR}/third-party/spdlog/include")

# /usr/local doesn't get added by default in Mac OS X
if (APPLE)
    set(INCLUDE_DIRS "${INCLUDE_DIRS}" /usr/local/include)
endif()

target_include_directories(sensei PRIVATE ${INCLUDE_DIRS})
if (APPLE)
    target_include_directories(sensei PRIVATE /usr/local/lib)
endif()

####################################
#  Compiler Flags and definitions  #
####################################

target_compile_features(sensei PRIVATE cxx_std_17)
target_compile_options(sensei PRIVATE -Wall -Wextra -Wno-psabi -fno-rtti -ffast-math)

if (APPLE)
    set_target_properties(sensei LINK_FLAGS -framework IOKit -framework CoreFoundation)
endif()

#################################
#  Statically linked libraries  #
#################################
add_subdirectory(gpio_protocol)

set(LINK_LIBRARIES gpio_protocol pthread jsoncpp lo)
target_link_libraries(sensei PRIVATE ${LINK_LIBRARIES})

####################
#  Add unit tests  #
####################

if (${WITH_UNIT_TESTS})
    add_subdirectory(test)
endif()

###########
#  Tools  #
###########

add_subdirectory(test/tools/socket_example EXCLUDE_FROM_ALL)