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

#ifndef SAMPLES_SAMPLES03_INCLUDE_RT_MANIPULATORS_DYNAMICS_HPP_
#define SAMPLES_SAMPLES03_INCLUDE_RT_MANIPULATORS_DYNAMICS_HPP_

#include <map>

#include "rt_manipulators_cpp/kinematics_utils.hpp"
#include "rt_manipulators_cpp/link.hpp"

namespace samples03_dynamics {

using torque_to_current_t = std::map<kinematics_utils::link_id_t, double>;

bool gravity_compensation(const kinematics_utils::links_t & links,
                          const kinematics_utils::link_id_t & target_id,
                          const torque_to_current_t & torque_to_current,
                          kinematics_utils::q_list_t & q_list);

}  // namespace samples03_dynamics

#endif  // SAMPLES_SAMPLES03_INCLUDE_RT_MANIPULATORS_DYNAMICS_HPP_
