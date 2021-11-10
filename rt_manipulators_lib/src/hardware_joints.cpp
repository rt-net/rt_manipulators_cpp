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

#include "hardware_joints.hpp"


namespace hardware {


const JointGroupMap Joints::groups() const {
  return joint_groups_;
}

void Joints::append_group(const JointGroupName & group_name, const joint::JointGroup & group) {
  auto joint_group_ptr = std::make_shared<joint::JointGroup>(group);
  joint_groups_.emplace(group_name, joint_group_ptr);
}

void Joints::append_joint(const JointName & joint_name, const joint::Joint & joint) {
  auto joint_ptr = std::make_shared<joint::Joint>(joint);
  all_joints_.emplace(joint_name, joint_ptr);
  // IDからもJointにアクセスできる
  all_joints_ref_from_id_.emplace(joint.id(), joint_ptr);
}

std::shared_ptr<joint::JointGroup> Joints::group(const JointGroupName & name) {
  return joint_groups_.at(name);
}

std::shared_ptr<joint::Joint> Joints::joint(const JointName & name) {
  return all_joints_.at(name);
}

std::shared_ptr<joint::Joint> Joints::joint(const uint8_t & id) {
  return all_joints_ref_from_id_.at(id);
}

bool Joints::has_group(const JointGroupName & name) {
  return joint_groups_.find(name) != joint_groups_.end();
}

bool Joints::has_joint(const JointName & name) {
  return all_joints_.find(name) != all_joints_.end();
}

bool Joints::has_joint(const uint8_t & id) {
  return all_joints_ref_from_id_.find(id) != all_joints_ref_from_id_.end();
}

}  // namespace hardware
