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

#ifndef RT_MANIPULATORS_LIB_INCLUDE_DYNAMIXEL_XM_HPP_
#define RT_MANIPULATORS_LIB_INCLUDE_DYNAMIXEL_XM_HPP_

#include <string>

#include "dynamixel_base.hpp"

namespace dynamixel_xm {

class DynamixelXM : public dynamixel_base::DynamixelBase  {
 public:
  explicit DynamixelXM(const uint8_t id, const int home_position = 2048);

  bool read_operating_mode(const dynamixel_base::comm_t & comm, uint8_t & mode);
  bool write_operating_mode(const dynamixel_base::comm_t & comm, const uint8_t mode);
  bool read_current_limit(const dynamixel_base::comm_t & comm, double & limit_ampere);
  bool read_max_position_limit(const dynamixel_base::comm_t & comm, double & limit_radian);
  bool read_min_position_limit(const dynamixel_base::comm_t & comm, double & limit_radian);

  bool write_torque_enable(const dynamixel_base::comm_t & comm, const bool enable);

  bool write_velocity_i_gain(const dynamixel_base::comm_t & comm, const unsigned int gain);
  bool write_velocity_p_gain(const dynamixel_base::comm_t & comm, const unsigned int gain);
  bool write_position_d_gain(const dynamixel_base::comm_t & comm, const unsigned int gain);
  bool write_position_i_gain(const dynamixel_base::comm_t & comm, const unsigned int gain);
  bool write_position_p_gain(const dynamixel_base::comm_t & comm, const unsigned int gain);

  bool write_profile_acceleration(
    const dynamixel_base::comm_t & comm, const double acceleration_rpss);
  bool write_profile_velocity(
    const dynamixel_base::comm_t & comm, const double velocity_rps);

  unsigned int to_profile_acceleration(const double acceleration_rpss);
  unsigned int to_profile_velocity(const double velocity_rps);
  double to_position_radian(const int position);
  double to_velocity_rps(const int velocity);
  double to_current_ampere(const int current);
  double to_voltage_volt(const int voltage);
  unsigned int from_position_radian(const double position_rad);
  unsigned int from_velocity_rps(const double velocity_rps);
  unsigned int from_current_ampere(const double current_ampere);

  bool auto_set_indirect_address_of_present_position(const dynamixel_base::comm_t & comm);
  bool auto_set_indirect_address_of_present_velocity(const dynamixel_base::comm_t & comm);
  bool auto_set_indirect_address_of_present_current(const dynamixel_base::comm_t & comm);
  bool auto_set_indirect_address_of_present_input_voltage(const dynamixel_base::comm_t & comm);
  bool auto_set_indirect_address_of_present_temperature(const dynamixel_base::comm_t & comm);

  unsigned int indirect_addr_of_present_position(void);
  unsigned int indirect_addr_of_present_velocity(void);
  unsigned int indirect_addr_of_present_current(void);
  unsigned int indirect_addr_of_present_input_voltage(void);
  unsigned int indirect_addr_of_present_temperature(void);

  unsigned int start_address_for_indirect_read(void);
  unsigned int length_of_indirect_data_read(void);
  unsigned int next_indirect_addr_read(void) const;

  bool extract_present_position_from_sync_read(
    const dynamixel_base::comm_t & comm, const std::string & group_name,
    double & position_rad);
  bool extract_present_velocity_from_sync_read(
    const dynamixel_base::comm_t & comm, const std::string & group_name,
    double & velocity_rps);
  bool extract_present_current_from_sync_read(
    const dynamixel_base::comm_t & comm, const std::string & group_name,
    double & current_ampere);
  bool extract_present_input_voltage_from_sync_read(
    const dynamixel_base::comm_t & comm, const std::string & group_name,
    double & voltage_volt);
  bool extract_present_temperature_from_sync_read(
    const dynamixel_base::comm_t & comm, const std::string & group_name,
    int & temperature_deg);

 protected:
  int HOME_POSITION_;
  unsigned int total_length_of_indirect_addr_read_;
  uint16_t indirect_addr_of_present_position_;
  uint16_t indirect_addr_of_present_velocity_;
  uint16_t indirect_addr_of_present_current_;
  uint16_t indirect_addr_of_present_input_voltage_;
  uint16_t indirect_addr_of_present_temperature_;

  bool set_indirect_address_read(
    const dynamixel_base::comm_t & comm, const uint16_t addr, const uint16_t len,
    uint16_t & indirect_addr);
};

}  // namespace dynamixel_xm

#endif  // RT_MANIPULATORS_LIB_INCLUDE_DYNAMIXEL_XM_HPP_
