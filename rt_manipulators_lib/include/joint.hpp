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

#ifndef RT_MANIPULATORS_LIB_INCLUDE_JOINT_HPP_
#define RT_MANIPULATORS_LIB_INCLUDE_JOINT_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace joint {

class Joint {
 public:
  Joint(const uint8_t id, const uint8_t operating_mode,
        const double max_position_limit, const double min_position_limit,
        const double current_limit_when_position_exceeds_limit);
  uint8_t id() const;
  uint8_t operating_mode() const;
  double max_position_limit() const;
  double min_position_limit() const;
  double current_limit_when_position_exceeds_limit() const;
  void set_present_position(const double position_radian);
  void set_present_velocity(const double velocity_rps);
  void set_present_current(const double current_ampere);
  void set_present_voltage(const double voltage_volt);
  void set_present_temperature(const int8_t temperature_degree);
  double get_present_position() const;
  double get_present_velocity() const;
  double get_present_current() const;
  double get_present_voltage() const;
  int8_t get_present_temperature() const;
  void set_goal_position(const double position_radian);
  void set_goal_velocity(const double velocity_rps);
  void set_goal_current(const double current_ampere);
  double get_goal_position() const;
  double get_goal_velocity() const;
  double get_goal_current() const;

 private:
  uint8_t id_;
  uint8_t operating_mode_;
  double max_position_limit_;
  double min_position_limit_;
  double current_limit_when_position_exceeds_limit_;
  double present_position_;
  double present_velocity_;
  double present_current_;
  double present_voltage_;
  int8_t present_temperature_;
  double goal_position_;
  double goal_velocity_;
  double goal_current_;
};

class JointGroup {
 public:
  JointGroup(const std::vector<std::string>& joint_names,
             const std::vector<std::string>& sync_read_targets,
             const std::vector<std::string>& sync_write_targets);
  std::vector<std::string> joint_names() const;
  bool sync_read_position_enabled() const;
  bool sync_read_velocity_enabled() const;
  bool sync_read_current_enabled() const;
  bool sync_read_voltage_enabled() const;
  bool sync_read_temperature_enabled() const;
  bool sync_write_position_enabled() const;
  bool sync_write_velocity_enabled() const;
  bool sync_write_current_enabled() const;

 private:
  std::vector<std::string> joint_names_;
  bool sync_read_position_enabled_;
  bool sync_read_velocity_enabled_;
  bool sync_read_current_enabled_;
  bool sync_read_voltage_enabled_;
  bool sync_read_temperature_enabled_;
  bool sync_write_position_enabled_;
  bool sync_write_velocity_enabled_;
  bool sync_write_current_enabled_;
};

}  // namespace joint

#endif  // RT_MANIPULATORS_LIB_INCLUDE_JOINT_HPP_
