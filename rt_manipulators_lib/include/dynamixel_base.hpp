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

 protected:
  uint8_t id_;
  std::string name_;
};

}  // namespace dynamixel_base

#endif  // RT_MANIPULATORS_LIB_INCLUDE_DYNAMIXEL_BASE_HPP_
