cmake_minimum_required(VERSION 2.8)
project(sensei)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

# Build all included third party dependecies
add_subdirectory(third-party)

# List all source files here
set(SOURCE_FILES src/main.cpp)

# Add to here if you need to include say third-party/xxx/include
set(INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/src "
                 "${PROJECT_SOURCE_DIR}/include "
                 "${PROJECT_SOURCE_DIR}/third-party/libserialport "
                 "${PROJECT_SOURCE_DIR}/third-party/liblo/include ")

# Add any libraries that should be linked here
set(LINK_LIBRARIES "${PROJECT_SOURCE_DIR}/third-party/libserialport/.libs/libserialport.a "
                   "${PROJECT_SOURCE_DIR}/third-party/liblo/lib/liblo.a")

message("Source files: ")
message("" ${SOURCE_FILES})

include_directories(${INCLUDE_DIRS})
add_executable(sensei ${SOURCE_FILES})

# libserialport must be added as a depency, otherwise it might not get built
add_dependencies(sensei libserialport)
add_dependencies(sensei liblo)
target_link_libraries(sensei "${LINK_LIBRARIES}")

# Build tests
add_subdirectory(test)
