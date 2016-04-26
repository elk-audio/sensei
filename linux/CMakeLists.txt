cmake_minimum_required(VERSION 2.8)
project(sensei)

####################
#  Compiler Flags  #
####################

# Global flags
set(PROJECT_CXX_FLAGS "-Wall -Wextra -std=c++14 -fno-rtti")
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
                      src/serial_frontend/serial_frontend.cpp
                      src/serial_frontend/serial_command_creator.cpp
                      src/serial_frontend/message_tracker.cpp
                      src/main.cpp)


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
                        src/serial_frontend/serial_frontend.h
                        src/serial_frontend/serial_frontend_internal.h
                        src/serial_frontend/serial_command_creator.h
                        src/serial_frontend/message_tracker.h
                        src/locked_queue.h
                        src/synchronized_queue.h
                        src/event_handler.h
                        src/utils.h)

set(SOURCE_FILES "${COMPILATION_UNITS}" "${EXTRA_CLION_SOURCES}")

#########################
#  Include Directories  #
#########################

set(INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/src"
                 "${PROJECT_SOURCE_DIR}/include"
                 "${PROJECT_SOURCE_DIR}/../common"
                 "${PROJECT_SOURCE_DIR}/third-party/install/libserialport/include"
                 "${PROJECT_SOURCE_DIR}/third-party/install/liblo/include"
                 "${PROJECT_SOURCE_DIR}/third-party/optionparser/")

# /usr/local doesn't get added by default in Mac OS X
if (APPLE)
    set(INCLUDE_DIRS "${INCLUDE_DIRS}" /usr/local/include)
endif()

include_directories(${INCLUDE_DIRS})

#################################
#  Statically linked libraries  #
#################################

set(LINK_LIBRARIES "${PROJECT_SOURCE_DIR}/third-party/install/libserialport/lib/libserialport.a"
                   "${PROJECT_SOURCE_DIR}/third-party/install/liblo/lib/liblo.a"
                   pthread jsoncpp)

if (APPLE)
    link_directories(/usr/local/lib)
endif()

#############################################
#  Subdirectory projects                    #
#############################################

add_subdirectory(third-party)
add_subdirectory(test)

####################
#  Target Objects  #
####################

add_executable(sensei ${SOURCE_FILES})

# Explicitly add external statically linked dependencies here
add_dependencies(sensei libserialport)
add_dependencies(sensei liblo)
target_link_libraries(sensei "${LINK_LIBRARIES}")