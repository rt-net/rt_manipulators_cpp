cmake_minimum_required(VERSION 3.10)

# set the project name and version
project(rt_manipulators_cpp_test VERSION 1.0)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

enable_testing()

find_package(GTest REQUIRED)

include(GoogleTest)

add_executable(test_joint test_joint.cpp)
target_link_libraries(test_joint
    GTest::GTest
    GTest::Main
    rt_manipulators_cpp
)

gtest_discover_tests(test_joint)