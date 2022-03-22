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

#ifndef RT_MANIPULATORS_LIB_INCLUDE_DYNAMIXEL_BASE_HPP_
#define RT_MANIPULATORS_LIB_INCLUDE_DYNAMIXEL_BASE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_communicator.hpp"

namespace dynamixel_base {

using comm_t = std::shared_ptr<hardware_communicator::Communicator>;

class DynamixelBase {
 public:
  explicit DynamixelBase(const uint8_t id) : id_(id), name_("base") {}
  ~DynamixelBase() {}

  uint8_t get_id() const { return id_; }
  std::string get_name() const { return name_; }

  virtual bool read_operating_mode(
    const dynamixel_base::comm_t & comm, uint8_t & mode) { return false; }
  virtual bool write_operating_mode(
    const dynamixel_base::comm_t & comm, const uint8_t mode) { return false; }
  virtual bool read_current_limit(
    const dynamixel_base::comm_t & comm, double & limit_ampere) { return false; }
  virtual bool read_max_position_limit(
    const dynamixel_base::comm_t & comm, double & limit_radian) { return false; }
  virtual bool read_min_position_limit(
    const dynamixel_base::comm_t & comm, double & limit_radian) { return false; }

  virtual bool write_torque_enable(
    const dynamixel_base::comm_t & comm, const bool enable) { return false; }

  virtual bool write_velocity_i_gain(
    const dynamixel_base::comm_t & comm, const unsigned int gain) { return false; }
  virtual bool write_velocity_p_gain(
    const dynamixel_base::comm_t & comm, const unsigned int gain) { return false; }
  virtual bool write_position_d_gain(
    const dynamixel_base::comm_t & comm, const unsigned int gain) { return false; }
  virtual bool write_position_i_gain(
    const dynamixel_base::comm_t & comm, const unsigned int gain) { return false; }
  virtual bool write_position_p_gain(
    const dynamixel_base::comm_t & comm, const unsigned int gain) { return false; }

  virtual bool write_profile_acceleration(
    const dynamixel_base::comm_t & comm, const double acceleration_rpss) { return false; }
  virtual bool write_profile_velocity(
    const dynamixel_base::comm_t & comm, const double velocity_rps) { return false; }

  virtual unsigned int to_profile_acceleration(const double acceleration_rpss) { return 1; }
  virtual unsigned int to_profile_velocity(const double velocity_rps) { return 1; }
  virtual double to_position_radian(const int position) { return 0.0; }
  virtual double to_velocity_rps(const int velocity) { return 0.0; }
  virtual double to_current_ampere(const int current) { return 0.0; }
  virtual double to_voltage_volt(const int voltage) { return 0.0; }
  virtual unsigned int from_position_radian(const double position_rad) { return 0; }
  virtual unsigned int from_velocity_rps(const double velocity_rps) { return 0; }
  virtual unsigned int from_current_ampere(const double current_ampere) { return 0; }

  virtual bool auto_set_indirect_address_of_present_position(
    const dynamixel_base::comm_t & comm) { return false; }
  virtual bool auto_set_indirect_address_of_present_velocity(
    const dynamixel_base::comm_t & comm) { return false; }
  virtual bool auto_set_indirect_address_of_present_current(
    const dynamixel_base::comm_t & comm) { return false; }
  virtual bool auto_set_indirect_address_of_present_input_voltage(
    const dynamixel_base::comm_t & comm) { return false; }
  virtual bool auto_set_indirect_address_of_present_temperature(
    const dynamixel_base::comm_t & comm) { return false; }
  virtual bool auto_set_indirect_address_of_goal_position(
    const dynamixel_base::comm_t & comm) { return false; }
  virtual bool auto_set_indirect_address_of_goal_velocity(
    const dynamixel_base::comm_t & comm) { return false; }
  virtual bool auto_set_indirect_address_of_goal_current(
    const dynamixel_base::comm_t & comm) { return false; }

  virtual unsigned int indirect_addr_of_present_position(void) { return 0; }
  virtual unsigned int indirect_addr_of_present_velocity(void) { return 0; }
  virtual unsigned int indirect_addr_of_present_current(void) { return 0; }
  virtual unsigned int indirect_addr_of_present_input_voltage(void) { return 0; }
  virtual unsigned int indirect_addr_of_present_temperature(void) { return 0; }
  virtual unsigned int indirect_addr_of_goal_position(void) { return 0; }
  virtual unsigned int indirect_addr_of_goal_velocity(void) { return 0; }
  virtual unsigned int indirect_addr_of_goal_current(void) { return 0; }

  virtual unsigned int start_address_for_indirect_read(void) { return 0; }
  virtual unsigned int length_of_indirect_data_read(void) { return 0; }
  virtual unsigned int next_indirect_addr_read(void) const { return 0; }

  virtual unsigned int start_address_for_indirect_write(void) { return 0; }
  virtual unsigned int length_of_indirect_data_write(void) { return 0; }
  virtual unsigned int next_indirect_addr_write(void) const { return 0; }

  virtual bool extract_present_position_from_sync_read(
    const dynamixel_base::comm_t & comm, const std::string & group_name,
    double & position_rad) { return false; }
  virtual bool extract_present_velocity_from_sync_read(
    const dynamixel_base::comm_t & comm, const std::string & group_name,
    double & velocity_rps) { return false; }
  virtual bool extract_present_current_from_sync_read(
    const dynamixel_base::comm_t & comm, const std::string & group_name,
    double & current_ampere) { return false; }
  virtual bool extract_present_input_voltage_from_sync_read(
    const dynamixel_base::comm_t & comm, const std::string & group_name,
    double & voltage_volt) { return false; }
  virtual bool extract_present_temperature_from_sync_read(
    const dynamixel_base::comm_t & comm, const std::string & group_name,
    int & temperature_deg) { return false; }

  virtual void push_back_position_for_sync_write(
    const double position_rad, std::vector<uint8_t> & write_data) {}
  virtual void push_back_velocity_for_sync_write(
    const double velocity_rps, std::vector<uint8_t> & write_data) {}
  virtual void push_back_current_for_sync_write(
    const double current_ampere, std::vector<uint8_t> & write_data) {}

 protected:
  uint8_t id_;
  std::string name_;
};

}  // namespace dynamixel_base

#endif  // RT_MANIPULATORS_LIB_INCLUDE_DYNAMIXEL_BASE_HPP_
