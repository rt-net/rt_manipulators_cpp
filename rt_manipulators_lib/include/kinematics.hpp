// Copyright 2021 RT Corporation
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

#ifndef RT_MANIPULATORS_LIB_INCLUDE_KINEMATICS_HPP_
#define RT_MANIPULATORS_LIB_INCLUDE_KINEMATICS_HPP_

#include <eigen3/Eigen/Dense>
#include <vector>

#include "kinematics_utils.hpp"
#include "link.hpp"

namespace kinematics {

void forward_kinematics(std::vector<manipulators_link::Link> & links, const int & start_id);
bool inverse_kinematics_LM(
  const kinematics_utils::links_t & links, const kinematics_utils::link_id_t & target_id,
  const Eigen::Vector3d & target_p, const Eigen::Matrix3d & target_R,
  kinematics_utils::q_list_t & result_q_list);
}  // namespace kinematics

#endif  // RT_MANIPULATORS_LIB_INCLUDE_KINEMATICS_HPP_
