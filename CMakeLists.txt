cmake_minimum_required(VERSION 3.15..3.23)
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

# Enable
option(WITH_GPIO_LOGIC "Perform gpio logic instead of a micro controller" OFF)

option(SENSEI_USE_GRPC "Use the gRPC backend, if disabled will use OSC" OFF)

################################
#  gRPC / Protobuf generation  #
################################
if (${SENSEI_USE_GRPC})
    find_package(gRPC REQUIRED)

    set(PROTOS pin-proxy/pin_events.proto)

    # In yocto environments, yocto grpc does not include gRPCPluginTargets.cmake
    # when find_package(gRPC REQUIRED) is called.
    if(CMAKE_CROSSCOMPILING)
        find_program(Protobuf_PROTOC_EXECUTABLE REQUIRED NAMES protoc)
        if(NOT TARGET protobuf::protoc)
            add_executable(protobuf::protoc IMPORTED)
        endif()
        set_target_properties(protobuf::protoc PROPERTIES
            IMPORTED_LOCATION "${Protobuf_PROTOC_EXECUTABLE}")
        find_program(_gRPC_CPP_PLUGIN grpc_cpp_plugin)
    else()
        set(_gRPC_CPP_PLUGIN \$<TARGET_FILE:gRPC::grpc_cpp_plugin>)
    endif()

    # Intermediate library target with the generated protobuf objects
    add_library(protos-lib OBJECT)
    target_sources(protos-lib PRIVATE ${PROTOS})
    target_link_libraries(protos-lib PUBLIC gRPC::grpc++)
    target_include_directories(protos-lib PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
    protobuf_generate(TARGET protos-lib LANGUAGE cpp)
    protobuf_generate(
        TARGET protos-lib
        LANGUAGE grpc
        GENERATE_EXTENSIONS .grpc.pb.h .grpc.pb.cc
        PLUGIN
        "protoc-gen-grpc=${_gRPC_CPP_PLUGIN}"
    )

    set(BACKFRONTEND_SOURCE_FILES
        src/output_backend/grpc_backend.cpp
        src/output_backend/grpc_backend.h
        src/user_frontend/grpc_user_frontend.cpp
        src/user_frontend/grpc_user_frontend.h
        src/user_frontend/async_call_handlers.cpp
        src/user_frontend/async_call_handlers.h
    )
    set(BACKFRONTEND_LIBRARIES protos-lib)
else()
    set(BACKFRONTEND_SOURCE_FILES
        src/output_backend/osc_backend.cpp
        src/output_backend/osc_backend.h
        src/user_frontend/osc_user_frontend.cpp
        src/user_frontend/osc_user_frontend.h
    )
    if (CMAKE_CROSSCOMPILING)
        set(BACKFRONTEND_LIBRARIES lo)
    else()
        find_package(liblo CONFIG REQUIRED)
        set(BACKFRONTEND_LIBRARIES liblo::liblo)
    endif()
endif()

############################
#  Main executable target  #
############################

set(COMPILATION_UNITS src/config_backend/json_configuration.cpp
                      src/mapping/sensor_mappers.cpp
                      src/mapping/mapping_processor.cpp
                      src/event_handler.cpp
                      src/output_backend/std_stream_backend.cpp
                      src/hardware_frontend/hw_frontend.cpp
                      src/hardware_frontend/message_tracker.cpp
                      src/hardware_frontend/gpio_command_creator.cpp
                      src/hardware_backend/gpio_hw_socket.cpp
                      src/main.cpp
                      src/logging.cpp
                      src/user_frontend/user_frontend.cpp)


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
                        src/hardware_frontend/base_hw_frontend.h
                        src/hardware_frontend/message_tracker.h
                        src/hardware_frontend/hw_frontend.h
                        src/hardware_frontend/gpio_command_creator.h
                        src/hardware_backend/base_hw_backend.h
                        src/hardware_backend/gpio_hw_socket.h
                        src/locked_queue.h
                        src/synchronized_queue.h
                        src/event_handler.h
                        src/utils.h
                        src/logging.h
                        src/user_frontend/user_frontend.h)

set(SOURCE_FILES "${COMPILATION_UNITS}" "${EXTRA_CLION_SOURCES}" "${BACKFRONTEND_SOURCE_FILES}")

add_executable(sensei ${SOURCE_FILES})

if (${SENSEI_USE_GRPC})
    target_compile_definitions(sensei PUBLIC -DSENSEI_USE_GRPC)
endif()

#########################
#  Include Directories  #
#########################

set(INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/src"
                 "${PROJECT_SOURCE_DIR}/include"
                 "${PROJECT_SOURCE_DIR}/shiftregister_gpio/include"
                 "${PROJECT_SOURCE_DIR}/elk-warning-suppressor/include"
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
    set_target_properties(sensei PROPERTIES LINK_FLAGS "-framework IOKit -framework CoreFoundation")
endif()

#################################
#  Statically linked libraries  #
#################################
add_subdirectory(elk-gpio-protocol)
add_subdirectory(shiftregister_gpio)

# these libraries are taken from vcpkg when compiling locally, but are provided
# in the sysroot when cross-compiling
if (CMAKE_CROSSCOMPILING)
    set(EXTRA_LIBRARIES jsoncpp)
else()
    find_package(jsoncpp CONFIG REQUIRED)
    set(EXTRA_LIBRARIES JsonCpp::JsonCpp)
endif()

set(LINK_LIBRARIES gpio_protocol shiftreg_gpio pthread ${BACKFRONTEND_LIBRARIES} ${EXTRA_LIBRARIES})

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
