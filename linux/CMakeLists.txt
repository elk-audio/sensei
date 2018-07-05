cmake_minimum_required(VERSION 3.8)
project(sensei)

##################################
#  Build  information            #
##################################

set(WITH_UNIT_TESTS true CACHE BOOL "Build and run unittests")

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

####################
#  Compiler Flags  #
####################

# Global flags
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(PROJECT_CXX_FLAGS "-Wall -Wextra -fno-rtti")
set(PROJECT_LINK_FLAGS "")

# Platform specific stuff
if (APPLE)
    set(PROJECT_LINK_FLAGS "${PROJECT_LINK_FLAGS} -framework IOKit -framework CoreFoundation")
endif()

# Build optimization flags
set(CMAKE_CXX_FLAGS_RELEASE "-DNDEBUG -O3")
set(CMAKE_CXX_FLAGS_DEBUG   "-DDEBUG -O0 -g")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${PROJECT_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${PROJECT_LINK_FLAGS}")

##################
#  Source Files  #
##################

set(COMPILATION_UNITS src/config_backend/json_configuration.cpp
                      src/mapping/sensor_mappers.cpp
                      src/mapping/mapping_processor.cpp
                      src/event_handler.cpp
                      src/output_backend/std_stream_backend.cpp
                      src/output_backend/osc_backend.cpp
                      src/hardware_frontend/serial_frontend.cpp
                      src/hardware_frontend/serial_command_creator.cpp
                      src/hardware_frontend/raspa_frontend.cpp
                      src/hardware_frontend/message_tracker.cpp
                      src/hardware_frontend/xmos_command_creator.cpp
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
                        src/hardware_frontend/serial_frontend.h
                        src/hardware_frontend/serial_frontend_internal.h
                        src/hardware_frontend/serial_command_creator.h
                        src/hardware_frontend/message_tracker.h
                        src/hardware_frontend/raspa_frontend.h
                        src/hardware_frontend/xmos_command_creator.h
                        src/locked_queue.h
                        src/synchronized_queue.h
                        src/event_handler.h
                        src/utils.h
                        src/logging.h
                        src/user_frontend/user_frontend.h
                        src/user_frontend/osc_user_frontend.h)

set(SOURCE_FILES "${COMPILATION_UNITS}" "${EXTRA_CLION_SOURCES}")

#########################
#  Include Directories  #
#########################

set(INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/src"
                 "${PROJECT_SOURCE_DIR}/include"
                 "${PROJECT_SOURCE_DIR}/xmos_protocol"
                 "${PROJECT_SOURCE_DIR}/../common"
                 "${CMAKE_BINARY_DIR}"
                 "${PROJECT_SOURCE_DIR}/third-party/optionparser/"
                 "${PROJECT_SOURCE_DIR}/third-party/spdlog/include")

# /usr/local doesn't get added by default in Mac OS X
if (APPLE)
    set(INCLUDE_DIRS "${INCLUDE_DIRS}" /usr/local/include)
endif()

include_directories(${INCLUDE_DIRS})

#################################
#  Statically linked libraries  #
#################################

set(LINK_LIBRARIES serialport pthread jsoncpp lo)

if (APPLE)
    link_directories(/usr/local/lib)
endif()

#############################################
#  Subdirectory projects                    #
#############################################
if (${WITH_UNIT_TESTS})
    add_subdirectory(test)
endif()
####################
#  Target Objects  #
####################

add_executable(sensei ${SOURCE_FILES})

# Explicitly add external statically linked dependencies here
target_link_libraries(sensei "${LINK_LIBRARIES}")
