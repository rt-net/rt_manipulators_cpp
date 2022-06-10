cmake_minimum_required(VERSION 3.10)

# set the project name and version
project(rt_manipulators_cpp VERSION 1.0)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# add_subdirectory(src)
find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
find_package(yaml_cpp_vendor REQUIRED)

set(library_name ${CMAKE_PROJECT_NAME})

add_library(${library_name}
    SHARED
    src/hardware.cpp
    src/joint.cpp
    src/hardware_joints.cpp
    src/hardware_communicator.cpp
    src/kinematics.cpp
    src/kinematics_utils.cpp
    src/config_file_parser.cpp
    src/dynamixel_x.cpp
    src/dynamixel_xm430.cpp
    src/dynamixel_xm540.cpp
    src/dynamixel_xh430.cpp
    src/dynamixel_xh540.cpp
    src/dynamixel_p.cpp
    src/dynamixel_ph42.cpp
)
target_include_directories(${library_name}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${library_name}
    dynamixel_sdk
    Eigen3
    yaml_cpp_vendor
)

ament_export_targets(export_${library_name} HAS_LIBRARY_TARGET)
ament_export_dependencies(
    dynamixel_sdk
    eigen3_cmake_module
    Eigen3
    yaml_cpp_vendor
)
install(
  DIRECTORY include/
  DESTINATION include/${library_name}
)

install(
  TARGETS ${library_name}
  EXPORT export_${library_name}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
