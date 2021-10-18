#ifndef HARDWARE_HPP_
#define HARDWARE_HPP_

#include <chrono>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <map>
#include <memory>
#include <thread>
#include <vector>

#include "joint.hpp"

namespace rt_manipulators_cpp
{

using JointGroupName = std::string;
using JointName = std::string;

class Hardware
{
public:
  Hardware(const std::string device_name);
  bool load_config_file(const std::string & config_yaml);
  bool connect(const int baudrate = 3000000);
  void disconnect();
  bool torque_on(const std::string & group_name);
  bool torque_off(const std::string & group_name);
  bool sync_read(const std::string & group_name);
  bool get_positions(const std::string & group_name, std::vector<double> & positions);
  bool get_position(const uint8_t id, double & position);
  bool start_thread(const std::vector<std::string> & group_names, const std::chrono::milliseconds & update_cycle_ms);
  bool stop_thread();

private:
  bool parse_config_file(const std::string & config_yaml);
  bool joint_groups_contain(const std::string & group_name);
  bool all_joints_contain(const std::string & joint_name);
  bool all_joints_contain_id(const uint8_t id);
  bool create_sync_read_group(const std::string & group_name, const std::vector<std::string> & targets);
  bool create_sync_write_group(const std::string & group_name, const std::vector<std::string> & targets);
  bool set_indirect_address(const std::string & group_name, const uint16_t addr_indirect_start, const uint16_t addr_target, const uint16_t len_target);
  void read_write_thread(const std::vector<std::string> & group_names, const std::chrono::milliseconds & update_cycle_ms);
  bool write_byte_data(const uint8_t id, const uint16_t address, const uint8_t write_data);
  bool write_byte_data_to_group(const std::string & group_name, const uint16_t address, const uint8_t write_data);
  bool write_word_data(const uint8_t id, const uint16_t address, const uint16_t write_data);
  bool write_word_data_to_group(const std::string & group_name, const uint16_t address, const uint16_t write_data);
  bool parse_dxl_error(const std::string & func_name, const uint8_t id,
    const uint16_t address, const int dxl_comm_result, const uint8_t dxl_packet_error);
  bool parse_dxl_error(const std::string & func_name, const int dxl_comm_result);
  double dxl_pos_to_radian(const int32_t position);

  std::shared_ptr<dynamixel::PortHandler> port_handler_;
  std::shared_ptr<dynamixel::PacketHandler> packet_handler_;
  std::map<JointGroupName, std::shared_ptr<joint::JointGroup>> joint_groups_;
  std::map<JointName, std::shared_ptr<joint::Joint>> all_joints_;
  std::map<uint8_t, std::shared_ptr<joint::Joint>> all_joints_ref_from_id_;
  std::map<JointGroupName, std::shared_ptr<dynamixel::GroupSyncRead>> sync_read_groups_;
  std::map<JointGroupName, std::shared_ptr<dynamixel::GroupSyncWrite>> sync_write_groups_;
  std::map<JointGroupName, uint16_t> address_indirect_present_position_;
  std::map<JointGroupName, uint16_t> address_indirect_goal_position_;
  bool thread_enable_;
  std::shared_ptr<std::thread> read_write_thread_;

};

}  // namespace rt_manipulators_cpp

#endif // HARDWARE_HPP_