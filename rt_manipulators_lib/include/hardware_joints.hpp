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

#ifndef RT_MANIPULATORS_LIB_INCLUDE_HARDWARE_JOINTS_HPP_
#define RT_MANIPULATORS_LIB_INCLUDE_HARDWARE_JOINTS_HPP_

#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "joint.hpp"

namespace hardware {

using group_name_t = std::string;
using joint_name_t = std::string;
using group_map_t = std::map<group_name_t, std::shared_ptr<joint::JointGroup>>;
using dxl_id_t = uint8_t;

// ハードウェアのジョイント情報を持つクラス
class Joints{
 public:
  Joints() {}
  ~Joints() {}
  const group_map_t groups() const;
  void append_group(const group_name_t & group_name, const joint::JointGroup & group);
  void append_joint(const joint_name_t & joint_name, const joint::Joint & joint);
  std::shared_ptr<joint::JointGroup> group(const group_name_t & name);
  std::shared_ptr<joint::Joint> joint(const joint_name_t & name);
  std::shared_ptr<joint::Joint> joint(const dxl_id_t & id);
  bool has_group(const group_name_t & name);
  bool has_joint(const joint_name_t & name);
  bool has_joint(const dxl_id_t & id);

 private:
  group_map_t joint_groups_;
  std::map<joint_name_t, std::shared_ptr<joint::Joint>> all_joints_;
  std::map<dxl_id_t, std::shared_ptr<joint::Joint>> all_joints_ref_from_id_;
};

}  // namespace hardware

#endif  // RT_MANIPULATORS_LIB_INCLUDE_HARDWARE_JOINTS_HPP_
