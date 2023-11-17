// Copyright 2023 RT Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RT_MANIPULATORS_LIB_INCLUDE_KINEMATICS_ROS_UTILS_HPP_
#define RT_MANIPULATORS_LIB_INCLUDE_KINEMATICS_ROS_UTILS_HPP_

#include <string>

#include "kinematics_utils.hpp"

namespace kinematics_ros_utils {

kinematics_utils::links_t parse_urdf_file(const std::string & path);

}  // kinematics_ros_utils

#endif  // RT_MANIPULATORS_LIB_INCLUDE_KINEMATICS_ROS_UTILS_HPP_
