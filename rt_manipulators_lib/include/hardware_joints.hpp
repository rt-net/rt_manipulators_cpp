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

namespace hardware_joints {

using group_name_t = std::string;
using joint_name_t = std::string;
using group_map_t = std::map<group_name_t, std::shared_ptr<joint::JointGroup>>;
using dxl_id_t = uint8_t;
using position_t = double;
using velocity_t = double;
using current_t = double;
using voltage_t = double;
using temperature_t = int8_t;

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
  bool get_position(const dxl_id_t & id, position_t & position);
  bool get_position(const joint_name_t & joint_name, position_t & position);
  bool get_positions(const group_name_t & group_name, std::vector<position_t> & positions);
  bool get_velocity(const dxl_id_t & id, velocity_t & velocity);
  bool get_velocity(const joint_name_t & joint_name, velocity_t & velocity);
  bool get_velocities(const group_name_t & group_name, std::vector<velocity_t> & velocities);
  bool get_current(const dxl_id_t & id, current_t & current);
  bool get_current(const joint_name_t & joint_name, current_t & current);
  bool get_currents(const group_name_t & group_name, std::vector<current_t>& currents);
  bool get_voltage(const dxl_id_t id, voltage_t & voltage);
  bool get_voltage(const joint_name_t & joint_name, voltage_t & voltage);
  bool get_voltages(const group_name_t & group_name, std::vector<voltage_t>& voltages);
  bool get_temperature(const dxl_id_t & id, temperature_t & temperature);
  bool get_temperature(const joint_name_t & joint_name, temperature_t & temperature);
  bool get_temperatures(const group_name_t & group_name, std::vector<temperature_t>& temperatures);
  bool get_max_position_limit(const dxl_id_t & id, position_t & max_position_limit);
  bool get_min_position_limit(const dxl_id_t & id, position_t & min_position_limit);
  bool set_position(const dxl_id_t & id, const position_t & position);
  bool set_position(const joint_name_t & joint_name, const position_t & position);
  bool set_positions(const group_name_t & group_name, const std::vector<position_t> & positions);
  bool set_velocity(const dxl_id_t & id, const velocity_t & velocity);
  bool set_velocity(const joint_name_t & joint_name, const velocity_t & velocity);
  bool set_velocities(const group_name_t & group_name, const std::vector<velocity_t>& velocities);
  bool set_current(const dxl_id_t & id, const current_t & current);
  bool set_current(const joint_name_t & joint_name, const current_t & current);
  bool set_currents(const group_name_t & group_name, const std::vector<current_t>& currents);

 private:
  group_map_t joint_groups_;
  std::map<joint_name_t, std::shared_ptr<joint::Joint>> all_joints_;
  std::map<dxl_id_t, std::shared_ptr<joint::Joint>> all_joints_ref_from_id_;
};

}  // namespace hardware_joints

#endif  // RT_MANIPULATORS_LIB_INCLUDE_HARDWARE_JOINTS_HPP_
