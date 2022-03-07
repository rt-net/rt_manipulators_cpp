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

#include "joint.hpp"

namespace joint {

Joint::Joint(const uint8_t id, const uint8_t operating_mode)
    : id_(id), operating_mode_(operating_mode),
      position_limit_margin_(0.0),
      max_position_limit_(0.0), min_position_limit_(0.0),
      current_limit_margin_(0.0),
      max_current_limit_(0.0),
      current_limit_when_position_exceeds_limit_(0.0),
      present_position_(0.0), present_velocity_(0.0), present_current_(0.0),
      present_voltage_(0.0), present_temperature_(0), goal_position_(0.0),
      goal_velocity_(0.0), goal_current_(0.0) {}


uint8_t Joint::id() const { return id_; }

uint8_t Joint::operating_mode() const { return operating_mode_; }

void Joint::set_position_limit_margin(const double position_radian) {
  // 0より小さいマージンはセットしない
  position_limit_margin_ = std::max(position_radian, 0.0);
}

void Joint::set_position_limit(const double min_position_radian, const double max_position_radian) {
  min_position_limit_ = min_position_radian;
  max_position_limit_ = max_position_radian;
}

double Joint::max_position_limit() const {
  return max_position_limit_ - position_limit_margin_;
}

double Joint::min_position_limit() const {
  return min_position_limit_ + position_limit_margin_;
}

void Joint::set_current_limit_margin(const double current_ampere) {
  // 0より小さいマージンはセットしない
  current_limit_margin_ = std::max(current_ampere, 0.0);
}

void Joint::set_current_limit(const double max_current_ampere) {
  // 0より小さいリミットはセットしない
  max_current_limit_ = std::max(max_current_ampere, 0.0);
}

double Joint::current_limit_when_position_exceeds_limit() const {
  return max_current_limit_ - current_limit_margin_;
}

void Joint::set_present_position(const double position_radian) {
  present_position_ = position_radian;
}

void Joint::set_present_velocity(const double velocity_rps) {
  present_velocity_ = velocity_rps;
}

void Joint::set_present_current(const double current_ampere) {
  present_current_ = current_ampere;
}

void Joint::set_present_voltage(const double voltage_volt) {
  present_voltage_ = voltage_volt;
}

void Joint::set_present_temperature(const int8_t temperature_degree) {
  present_temperature_ = temperature_degree;
}

double Joint::get_present_position() const { return present_position_; }

double Joint::get_present_velocity() const { return present_velocity_; }

double Joint::get_present_current() const { return present_current_; }

double Joint::get_present_voltage() const { return present_voltage_; }

int8_t Joint::get_present_temperature() const { return present_temperature_; }

void Joint::set_goal_position(const double position_radian) { goal_position_ = position_radian; }

void Joint::set_goal_velocity(const double velocity_rps) { goal_velocity_ = velocity_rps; }

void Joint::set_goal_current(const double current_ampere) { goal_current_ = current_ampere; }

double Joint::get_goal_position() const { return goal_position_; }

double Joint::get_goal_velocity() const { return goal_velocity_; }

double Joint::get_goal_current() const { return goal_current_; }

JointGroup::JointGroup(const std::vector<std::string>& joint_names,
                       const std::vector<std::string>& sync_read_targets,
                       const std::vector<std::string>& sync_write_targets)
    : joint_names_(joint_names),
      sync_read_position_enabled_(false),
      sync_read_velocity_enabled_(false),
      sync_read_current_enabled_(false),
      sync_read_voltage_enabled_(false),
      sync_read_temperature_enabled_(false),
      sync_write_position_enabled_(false),
      sync_write_velocity_enabled_(false),
      sync_write_current_enabled_(false) {
  for (const auto & target : sync_read_targets) {
    if (target == "position") sync_read_position_enabled_ = true;
    if (target == "velocity") sync_read_velocity_enabled_ = true;
    if (target == "current") sync_read_current_enabled_ = true;
    if (target == "voltage") sync_read_voltage_enabled_ = true;
    if (target == "temperature") sync_read_temperature_enabled_ = true;
  }

  for (const auto & target : sync_write_targets) {
    if (target == "position") sync_write_position_enabled_ = true;
    if (target == "velocity") sync_write_velocity_enabled_ = true;
    if (target == "current") sync_write_current_enabled_ = true;
  }
}

std::vector<std::string> JointGroup::joint_names() const { return joint_names_; }

bool JointGroup::sync_read_position_enabled() const { return sync_read_position_enabled_; }

bool JointGroup::sync_read_velocity_enabled() const { return sync_read_velocity_enabled_; }

bool JointGroup::sync_read_current_enabled() const { return sync_read_current_enabled_; }

bool JointGroup::sync_read_voltage_enabled() const { return sync_read_voltage_enabled_; }

bool JointGroup::sync_read_temperature_enabled() const { return sync_read_temperature_enabled_; }

bool JointGroup::sync_write_position_enabled() const { return sync_write_position_enabled_; }

bool JointGroup::sync_write_velocity_enabled() const { return sync_write_velocity_enabled_; }

bool JointGroup::sync_write_current_enabled() const { return sync_write_current_enabled_; }

}  // namespace joint
