#ifndef HARDWARE_HPP_
#define HARDWARE_HPP_

#include <dynamixel_sdk/dynamixel_sdk.h>
#include <map>
#include <memory>
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
  // bool torque_off();
  // bool torque_on_all();

private:
  bool parse_config_file(const std::string & config_yaml);
  bool joint_groups_contain(const std::string & group_name);
  bool all_joints_contain(const std::string & joint_name);
  bool write_1byte_to_group(const std::string & group_name, const uint16_t address, const uint8_t write_data);
  bool write_1byte(const uint8_t id, const uint16_t address, const uint8_t write_data);
  bool parse_dxl_error(const std::string & func_name, const uint8_t id,
    const uint16_t address, const int dxl_comm_result, const uint8_t dxl_packet_error);

  std::shared_ptr<dynamixel::PortHandler> port_handler_;
  std::shared_ptr<dynamixel::PacketHandler> packet_handler_;
  std::map<JointGroupName, std::vector<JointName>> joint_groups_;
  std::map<JointName, joint::Joint> all_joints_;
};

}  // namespace rt_manipulators_cpp

#endif // HARDWARE_HPP_