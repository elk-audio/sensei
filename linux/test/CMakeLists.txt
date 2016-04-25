# build gtest framework
add_subdirectory(gtest_1_7_0)
enable_testing()

################
#  Test Files  #
################

SET(TEST_FILES unittests/sample_test.cpp
               unittests/locked_queue_test.cpp
               unittests/synchronized_queue_test.cpp
               unittests/configuration/json_configuration_test.cpp
               unittests/serial_frontend/serial_command_creator_test.cpp
               unittests/serial_frontend/serial_frontend_test.cpp
               unittests/serial_frontend/serial_fronted_internal_test.cpp
               unittests/serial_frontend/message_tracker_test.cpp
               unittests/message/message_test.cpp
               unittests/mapping/sensor_mappers_test.cpp
               unittests/mapping/mapping_processor_test.cpp
               unittests/mapping/output_backend_mockup.h
               unittests/test_utils.h
               unittests/output_backend/osc_backend_test.cpp)


##########################################
#  Include directories relative to test  #
##########################################

include_directories(${PROJECT_SOURCE_DIR}/test/gtest_1_7_0/include)

#################################
#  Statically linked libraries  #
#################################

set(LINK_LIBRARIES "${PROJECT_SOURCE_DIR}/third-party/install/libserialport/lib/libserialport.a"
                   "${PROJECT_SOURCE_DIR}/third-party/install/liblo/lib/liblo.a"
                   jsoncpp
                   gtest
                   gtest_main)

add_executable(unit_tests ${TEST_FILES})

target_link_libraries(unit_tests "${LINK_LIBRARIES}")
add_test(unit_tests unit_tests)

# custom target for running the tests
add_custom_target(run_tests ALL COMMAND "./unit_tests")
add_dependencies(run_tests unit_tests)

