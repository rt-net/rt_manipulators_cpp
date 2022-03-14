// Copyright 2022 RT Corporation
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

#ifndef SAMPLES_SAMPLES03_INCLUDE_RT_MANIPULATORS_IK_HPP_
#define SAMPLES_SAMPLES03_INCLUDE_RT_MANIPULATORS_IK_HPP_

#include "rt_manipulators_cpp/kinematics_utils.hpp"

namespace samples03 {

bool x7_3dof_inverse_kinematics(const kinematics_utils::links_t & links,
                                const Eigen::Vector3d & target_p,
                                kinematics_utils::q_list_t & q_list);
bool x7_3dof_picking_inverse_kinematics(const kinematics_utils::links_t & links,
                                        const Eigen::Vector3d & target_p,
                                        kinematics_utils::q_list_t & q_list);
bool s17_3dof_right_arm_inverse_kinematics(const kinematics_utils::links_t & links,
                                           const Eigen::Vector3d & target_p,
                                           kinematics_utils::q_list_t & q_list);
bool s17_3dof_right_arm_picking_inverse_kinematics(const kinematics_utils::links_t & links,
                                                   const Eigen::Vector3d & target_p,
                                                   kinematics_utils::q_list_t & q_list);

}  // namespace samples03

#endif  // SAMPLES_SAMPLES03_INCLUDE_RT_MANIPULATORS_IK_HPP_
