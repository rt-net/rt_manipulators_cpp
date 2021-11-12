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

#include <iostream>

#include "hardware_joints.hpp"


namespace hardware_joints {


const group_map_t Joints::groups() const {
  return joint_groups_;
}

void Joints::append_group(const group_name_t & group_name, const joint::JointGroup & group) {
  auto joint_group_ptr = std::make_shared<joint::JointGroup>(group);
  joint_groups_.emplace(group_name, joint_group_ptr);
}

void Joints::append_joint(const joint_name_t & joint_name, const joint::Joint & joint) {
  auto joint_ptr = std::make_shared<joint::Joint>(joint);
  all_joints_.emplace(joint_name, joint_ptr);
  // IDからもJointにアクセスできる
  all_joints_ref_from_id_.emplace(joint.id(), joint_ptr);
}

std::shared_ptr<joint::JointGroup> Joints::group(const group_name_t & name) {
  return joint_groups_.at(name);
}

std::shared_ptr<joint::Joint> Joints::joint(const joint_name_t & name) {
  return all_joints_.at(name);
}

std::shared_ptr<joint::Joint> Joints::joint(const dxl_id_t & id) {
  return all_joints_ref_from_id_.at(id);
}

bool Joints::has_group(const group_name_t & name) {
  return joint_groups_.find(name) != joint_groups_.end();
}

bool Joints::has_joint(const joint_name_t & name) {
  return all_joints_.find(name) != all_joints_.end();
}

bool Joints::has_joint(const dxl_id_t & id) {
  return all_joints_ref_from_id_.find(id) != all_joints_ref_from_id_.end();
}

bool Joints::get_position(const dxl_id_t & id, position_t & position) {
  if (!has_joint(id)) {
    std::cerr << "ID:" << std::to_string(id) << "のジョイントは存在しません." << std::endl;
    return false;
  }
  position = joint(id)->get_present_position();
  return true;
}

bool Joints::get_position(const joint_name_t & joint_name, position_t & position) {
  if (!has_joint(joint_name)) {
    std::cerr << joint_name << "ジョイントは存在しません." << std::endl;
    return false;
  }
  position = joint(joint_name)->get_present_position();
  return true;
}

bool Joints::get_positions(const group_name_t & group_name, std::vector<position_t> & positions) {
  if (!has_group(group_name)) {
    std::cerr << group_name << "はjoint_groupsに存在しません." << std::endl;
    return false;
  }

  for (const auto & joint_name : joint_groups_.at(group_name)->joint_names()) {
    positions.push_back(joint(joint_name)->get_present_position());
  }
  return true;
}

bool Joints::get_velocity(const dxl_id_t & id, velocity_t & velocity) {
  if (!has_joint(id)) {
    std::cerr << "ID:" << std::to_string(id) << "のジョイントは存在しません." << std::endl;
    return false;
  }
  velocity = joint(id)->get_present_velocity();
  return true;
}

bool Joints::get_velocity(const joint_name_t & joint_name, velocity_t & velocity) {
  if (!has_joint(joint_name)) {
    std::cerr << joint_name << "ジョイントは存在しません." << std::endl;
    return false;
  }
  velocity = joint(joint_name)->get_present_velocity();
  return true;
}

bool Joints::get_velocities(const group_name_t & group_name, std::vector<velocity_t> & velocities) {
  if (!has_group(group_name)) {
    std::cerr << group_name << "はjoint_groupsに存在しません." << std::endl;
    return false;
  }

  for (const auto & joint_name : joint_groups_.at(group_name)->joint_names()) {
    velocities.push_back(joint(joint_name)->get_present_velocity());
  }
  return true;
}

bool Joints::get_current(const dxl_id_t & id, current_t & current) {
  if (!has_joint(id)) {
    std::cerr << "ID:" << std::to_string(id) << "のジョイントは存在しません." << std::endl;
    return false;
  }
  current = joint(id)->get_present_current();
  return true;
}

bool Joints::get_current(const joint_name_t & joint_name, current_t & current) {
  if (!has_joint(joint_name)) {
    std::cerr << joint_name << "ジョイントは存在しません." << std::endl;
    return false;
  }
  current = joint(joint_name)->get_present_current();
  return true;
}

bool Joints::get_currents(const group_name_t & group_name, std::vector<current_t>& currents) {
  if (!has_group(group_name)) {
    std::cerr << group_name << "はjoint_groupsに存在しません." << std::endl;
    return false;
  }

  for (const auto & joint_name : joint_groups_.at(group_name)->joint_names()) {
    currents.push_back(joint(joint_name)->get_present_current());
  }
  return true;
}

bool Joints::get_voltage(const dxl_id_t id, voltage_t & voltage) {
  if (!has_joint(id)) {
    std::cerr << "ID:" << std::to_string(id) << "のジョイントは存在しません." << std::endl;
    return false;
  }
  voltage = joint(id)->get_present_voltage();
  return true;
}

bool Joints::get_voltage(const joint_name_t & joint_name, voltage_t & voltage) {
  if (!has_joint(joint_name)) {
    std::cerr << joint_name << "ジョイントは存在しません." << std::endl;
    return false;
  }
  voltage = joint(joint_name)->get_present_voltage();
  return true;
}

bool Joints::get_voltages(const group_name_t & group_name, std::vector<voltage_t>& voltages) {
  if (!has_group(group_name)) {
    std::cerr << group_name << "はjoint_groupsに存在しません." << std::endl;
    return false;
  }

  for (const auto & joint_name : joint_groups_.at(group_name)->joint_names()) {
    voltages.push_back(joint(joint_name)->get_present_voltage());
  }
  return true;
}

bool Joints::get_temperature(const dxl_id_t & id, temperature_t & temperature) {
  if (!has_joint(id)) {
    std::cerr << "ID:" << std::to_string(id) << "のジョイントは存在しません." << std::endl;
    return false;
  }
  temperature = joint(id)->get_present_temperature();
  return true;
}

bool Joints::get_temperature(const joint_name_t & joint_name, temperature_t & temperature) {
  if (!has_joint(joint_name)) {
    std::cerr << joint_name << "ジョイントは存在しません." << std::endl;
    return false;
  }
  temperature = joint(joint_name)->get_present_temperature();
  return true;
}

bool Joints::get_temperatures(
  const group_name_t & group_name, std::vector<temperature_t>& temperatures) {
  if (!has_group(group_name)) {
    std::cerr << group_name << "はjoint_groupsに存在しません." << std::endl;
    return false;
  }

  for (const auto & joint_name : joint_groups_.at(group_name)->joint_names()) {
    temperatures.push_back(joint(joint_name)->get_present_temperature());
  }
  return true;
}

bool Joints::set_position(const dxl_id_t & id, const position_t & position) {
  if (!has_joint(id)) {
    std::cerr << "ID:" << std::to_string(id) << "のジョイントは存在しません." << std::endl;
    return false;
  }
  joint(id)->set_goal_position(position);
  return true;
}

bool Joints::set_position(const joint_name_t & joint_name, const position_t & position) {
  if (!has_joint(joint_name)) {
    std::cerr << joint_name << "ジョイントは存在しません." << std::endl;
    return false;
  }
  joint(joint_name)->set_goal_position(position);
  return true;
}

bool Joints::set_positions(
  const group_name_t & group_name, const std::vector<position_t> & positions) {
  if (!has_group(group_name)) {
    std::cerr << group_name << "はjoint_groupsに存在しません." << std::endl;
    return false;
  }

  if (joint_groups_.at(group_name)->joint_names().size() != positions.size()) {
    std::cerr << "目標値のサイズ:" << positions.size();
    std::cerr << "がジョイント数:" << joint_groups_.at(group_name)->joint_names().size();
    std::cerr << "と一致しません." << std::endl;
    return false;
  }

  for (size_t i = 0; i < positions.size(); i++) {
    auto joint_name = joint_groups_.at(group_name)->joint_names()[i];
    joint(joint_name)->set_goal_position(positions[i]);
  }
  return true;
}

bool Joints::set_velocity(const dxl_id_t & id, const velocity_t & velocity) {
  if (!has_joint(id)) {
    std::cerr << "ID:" << std::to_string(id) << "のジョイントは存在しません." << std::endl;
    return false;
  }

  joint(id)->set_goal_velocity(velocity);
  return true;
}

bool Joints::set_velocity(const joint_name_t & joint_name, const velocity_t & velocity) {
  if (!has_joint(joint_name)) {
    std::cerr << joint_name << "ジョイントは存在しません." << std::endl;
    return false;
  }

  joint(joint_name)->set_goal_velocity(velocity);
  return true;
}

bool Joints::set_velocities(
  const group_name_t & group_name, const std::vector<velocity_t>& velocities) {
  if (!has_group(group_name)) {
    std::cerr << group_name << "はjoint_groupsに存在しません." << std::endl;
    return false;
  }

  if (joint_groups_.at(group_name)->joint_names().size() != velocities.size()) {
    std::cerr << "目標値のサイズ:" << velocities.size();
    std::cerr << "がジョイント数:" << joint_groups_.at(group_name)->joint_names().size();
    std::cerr << "と一致しません." << std::endl;
    return false;
  }

  for (size_t i = 0; i < velocities.size(); i++) {
    auto joint_name = joint_groups_.at(group_name)->joint_names()[i];
    joint(joint_name)->set_goal_velocity(velocities[i]);
  }
  return true;
}

}  // namespace hardware_joints
