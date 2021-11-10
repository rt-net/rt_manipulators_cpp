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

using JointGroupName = std::string;
using JointName = std::string;
using JointGroupMap = std::map<JointGroupName, std::shared_ptr<joint::JointGroup>>;

// ハードウェアのジョイント情報を持つクラス
class Joints{
 public:
  Joints() {}
  ~Joints() {}
  const JointGroupMap groups() const;
  void append_group(const JointGroupName & group_name, const joint::JointGroup & group);
  void append_joint(const JointName & joint_name, const joint::Joint & joint);
  std::shared_ptr<joint::JointGroup> group(const JointGroupName & name);
  std::shared_ptr<joint::Joint> joint(const JointName & name);
  std::shared_ptr<joint::Joint> joint(const uint8_t & id);
  bool has_group(const JointGroupName & name);
  bool has_joint(const JointName & name);
  bool has_joint(const uint8_t & id);

 private:
  JointGroupMap joint_groups_;
  std::map<JointName, std::shared_ptr<joint::Joint>> all_joints_;
  std::map<uint8_t, std::shared_ptr<joint::Joint>> all_joints_ref_from_id_;
};

}  // namespace hardware

#endif  // RT_MANIPULATORS_LIB_INCLUDE_HARDWARE_JOINTS_HPP_
