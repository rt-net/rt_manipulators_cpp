cmake_minimum_required(VERSION 3.10)

# set the project name and version
project(rt_manipulators_cpp_test VERSION 1.0)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

enable_testing()

find_package(GTest REQUIRED)

include(GoogleTest)

# サンプルのビルド
set(list_tests
    test_joint
    test_kinematics_utils
    test_kinematics
    test_config_file_parser
    test_dynamixel_x
    test_dynamixel_xh
    test_dynamixel_p
    test_hardware
)

# Download FakeIt
Set(FETCHCONTENT_QUIET FALSE)
include(FetchContent)
FetchContent_Declare(
    fakeit
    GIT_REPOSITORY https://github.com/eranpeer/FakeIt
    GIT_TAG 2.4.0
    GIT_PROGRESS TRUE)
FetchContent_MakeAvailable(fakeit)

foreach(test_executable IN LISTS list_tests)
    message("${test_executable}")
    add_executable(${test_executable}
        ${test_executable}.cpp
    )
    target_link_libraries(${test_executable}
        GTest::GTest
        GTest::Main
        rt_manipulators_cpp
    )
    target_include_directories(${test_executable} PRIVATE
        ${fakeit_SOURCE_DIR}/single_header/gtest
    )
    gtest_discover_tests(${test_executable})
endforeach()
