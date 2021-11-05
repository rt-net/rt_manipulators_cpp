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

#ifndef RT_MANIPULATORS_LIB_INCLUDE_HARDWARE_HPP_
#define RT_MANIPULATORS_LIB_INCLUDE_HARDWARE_HPP_

#include <dynamixel_sdk/dynamixel_sdk.h>
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "joint.hpp"

namespace rt_manipulators_cpp {

using JointGroupName = std::string;
using JointName = std::string;

class Hardware {
 public:
  explicit Hardware(const std::string device_name);
  ~Hardware();
  bool load_config_file(const std::string& config_yaml);
  bool connect(const int baudrate = 3000000);
  void disconnect();
  bool torque_on(const std::string& group_name);
  bool torque_off(const std::string& group_name);
  bool sync_read(const std::string& group_name);
  bool sync_write(const std::string& group_name);
  bool start_thread(const std::vector<std::string>& group_names,
                    const std::chrono::milliseconds& update_cycle_ms);
  bool stop_thread();
  bool get_position(const uint8_t id, double& position);
  bool get_position(const std::string& joint_name, double& position);
  bool get_positions(const std::string& group_name, std::vector<double>& positions);
  bool get_velocity(const uint8_t id, double& velocity);
  bool get_velocity(const std::string& joint_name, double& velocity);
  bool get_velocities(const std::string& group_name, std::vector<double>& velocities);
  bool get_current(const uint8_t id, double& current);
  bool get_current(const std::string& joint_name, double& current);
  bool get_currents(const std::string& group_name, std::vector<double>& currents);
  bool get_voltage(const uint8_t id, double& voltage);
  bool get_voltage(const std::string& joint_name, double& voltage);
  bool get_voltages(const std::string& group_name, std::vector<double>& voltages);
  bool get_temperature(const uint8_t id, int8_t& temperature);
  bool get_temperature(const std::string& joint_name, int8_t& temperature);
  bool get_temperatures(const std::string& group_name, std::vector<int8_t>& temperatures);
  bool set_position(const uint8_t id, const double position);
  bool set_position(const std::string& joint_name, const double position);
  bool set_positions(const std::string& group_name, std::vector<double>& positions);
  bool set_velocity(const uint8_t id, const double velocity);
  bool set_velocity(const std::string& joint_name, const double velocity);
  bool set_velocities(const std::string& group_name, std::vector<double>& velocities);
  bool write_max_acceleration_to_group(const std::string& group_name,
                                       const double acceleration_rpss);
  bool write_max_velocity_to_group(const std::string& group_name, const double velocity_rps);
  bool write_position_pid_gain(const uint8_t id, const uint16_t p, const uint16_t i,
                               const uint16_t d);
  bool write_position_pid_gain(const std::string& joint_name, const uint16_t p, const uint16_t i,
                               const uint16_t d);
  bool write_position_pid_gain_to_group(const std::string& group_name, const uint16_t p,
                                        const uint16_t i, const uint16_t d);

 protected:
  bool write_byte_data(const uint8_t id, const uint16_t address, const uint8_t write_data);
  bool write_byte_data_to_group(const std::string& group_name, const uint16_t address,
                                const uint8_t write_data);
  bool write_word_data(const uint8_t id, const uint16_t address, const uint16_t write_data);
  bool write_word_data_to_group(const std::string& group_name, const uint16_t address,
                                const uint16_t write_data);
  bool write_double_word_data(const uint8_t id, const uint16_t address, const uint32_t write_data);
  bool write_double_word_data_to_group(const std::string& group_name, const uint16_t address,
                                       const uint32_t write_data);
  bool read_byte_data(const uint8_t id, const uint16_t address, uint8_t& read_data);
  bool read_double_word_data(const uint8_t id, const uint16_t address, uint32_t& read_data);

 private:
  bool parse_config_file(const std::string& config_yaml);
  bool joint_groups_contain(const std::string& group_name);
  bool all_joints_contain(const std::string& joint_name);
  bool all_joints_contain_id(const uint8_t id);
  bool write_operating_mode(const std::string& group_name);
  bool limit_goal_velocity_by_present_position(const std::string& group_name);
  bool create_sync_read_group(const std::string& group_name);
  bool create_sync_write_group(const std::string& group_name);
  bool set_indirect_address(const std::string& group_name, const uint16_t addr_indirect_start,
                            const uint16_t addr_target, const uint16_t len_target);
  void read_write_thread(const std::vector<std::string>& group_names,
                         const std::chrono::milliseconds& update_cycle_ms);
  bool parse_dxl_error(const std::string& func_name, const uint8_t id, const uint16_t address,
                       const int dxl_comm_result, const uint8_t dxl_packet_error);
  bool parse_dxl_error(const std::string& func_name, const int dxl_comm_result);
  double dxl_pos_to_radian(const int32_t position);
  double dxl_velocity_to_rps(const int32_t velocity) const;
  double dxl_current_to_ampere(const int16_t current) const;
  double dxl_voltage_to_volt(const int16_t voltage) const;
  uint32_t radian_to_dxl_pos(const double position);
  uint32_t to_dxl_velocity(const double velocity_rps);
  uint32_t to_dxl_acceleration(const double acceleration_rpss);
  uint32_t to_dxl_profile_velocity(const double velocity_rps);

  std::shared_ptr<dynamixel::PortHandler> port_handler_;
  std::shared_ptr<dynamixel::PacketHandler> packet_handler_;
  std::map<JointGroupName, std::shared_ptr<joint::JointGroup>> joint_groups_;
  std::map<JointName, std::shared_ptr<joint::Joint>> all_joints_;
  std::map<uint8_t, std::shared_ptr<joint::Joint>> all_joints_ref_from_id_;
  std::map<JointGroupName, std::shared_ptr<dynamixel::GroupSyncRead>> sync_read_groups_;
  std::map<JointGroupName, std::shared_ptr<dynamixel::GroupSyncWrite>> sync_write_groups_;
  std::map<JointGroupName, uint16_t> addr_sync_read_position_;
  std::map<JointGroupName, uint16_t> addr_sync_read_velocity_;
  std::map<JointGroupName, uint16_t> addr_sync_read_current_;
  std::map<JointGroupName, uint16_t> addr_sync_read_voltage_;
  std::map<JointGroupName, uint16_t> addr_sync_read_temperature_;
  bool thread_enable_;
  std::shared_ptr<std::thread> read_write_thread_;
};

}  // namespace rt_manipulators_cpp

#endif  // RT_MANIPULATORS_LIB_INCLUDE_HARDWARE_HPP_
