add_subdirectory(gtest_1_7_0)
enable_testing()

######################
#  Unit Test target  #
######################

SET(TEST_FILES unittests/sample_test.cpp
               unittests/locked_queue_test.cpp
               unittests/synchronized_queue_test.cpp
               unittests/configuration/json_configuration_test.cpp
               unittests/message/message_test.cpp
               unittests/mapping/sensor_mappers_test.cpp
               unittests/mapping/mapping_processor_test.cpp
               unittests/mapping/output_backend_mockup.h
               unittests/test_utils.h
               unittests/output_backend/osc_backend_test.cpp
               unittests/user_frontend/osc_user_frontend_test.cpp)

add_executable(unit_tests ${TEST_FILES})
target_compile_definitions(unit_tests PRIVATE -DDISABLE_LOGGING)

#########################
#  Include directories  #
#########################

set(INCLUDE_DIRS "${INCLUDE_DIRS}"
                 ${PROJECT_SOURCE_DIR}/test/gtest_1_7_0/include)

target_include_directories(unit_tests PRIVATE ${INCLUDE_DIRS})

#################################
#  Statically linked libraries  #
#################################

set(LINK_LIBRARIES "${LINK_LIBRARIES}" gtest gtest_main)

target_link_libraries(unit_tests "${LINK_LIBRARIES}" )
add_test(unit_tests unit_tests)

# custom target for running the tests
add_custom_target(run_tests ALL COMMAND "./unit_tests")
add_dependencies(run_tests unit_tests)

