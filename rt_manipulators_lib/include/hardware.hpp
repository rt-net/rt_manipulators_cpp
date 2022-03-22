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

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "joint.hpp"
#include "hardware_joints.hpp"
#include "hardware_communicator.hpp"

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
  bool get_max_position_limit(const uint8_t & id, double & max_position_limit);
  bool get_min_position_limit(const uint8_t & id, double & min_position_limit);
  bool set_position(const uint8_t id, const double position);
  bool set_position(const std::string& joint_name, const double position);
  bool set_positions(const std::string& group_name, std::vector<double>& positions);
  bool set_velocity(const uint8_t id, const double velocity);
  bool set_velocity(const std::string& joint_name, const double velocity);
  bool set_velocities(const std::string& group_name, std::vector<double>& velocities);
  bool set_current(const uint8_t id, const double current);
  bool set_current(const std::string& joint_name, const double current);
  bool set_currents(const std::string& group_name, std::vector<double>& currents);
  bool write_max_acceleration_to_group(const std::string& group_name,
                                       const double acceleration_rpss);
  bool write_max_velocity_to_group(const std::string& group_name, const double velocity_rps);
  bool write_position_pid_gain(const uint8_t id, const uint16_t p, const uint16_t i,
                               const uint16_t d);
  bool write_position_pid_gain(const std::string& joint_name, const uint16_t p, const uint16_t i,
                               const uint16_t d);
  bool write_position_pid_gain_to_group(const std::string& group_name, const uint16_t p,
                                        const uint16_t i, const uint16_t d);
  bool write_velocity_pi_gain(const uint8_t id, const uint16_t p, const uint16_t i);
  bool write_velocity_pi_gain(const std::string& joint_name, const uint16_t p, const uint16_t i);
  bool write_velocity_pi_gain_to_group(const std::string& group_name, const uint16_t p,
                                       const uint16_t i);

 protected:
  std::shared_ptr<hardware_communicator::Communicator> comm_;

 private:
  bool write_operating_mode(const std::string& group_name);
  bool limit_goal_velocity_by_present_position(const std::string& group_name);
  bool limit_goal_current_by_present_position(const std::string& group_name);
  bool create_sync_read_group(const std::string& group_name);
  bool create_sync_write_group(const std::string& group_name);
  void read_write_thread(const std::vector<std::string>& group_names,
                         const std::chrono::milliseconds& update_cycle_ms);

  hardware_joints::Joints joints_;
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
