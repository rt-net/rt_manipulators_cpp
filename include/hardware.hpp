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
  // bool torque_off();
  // bool torque_on_all();

private:
  bool parse_config_file(const std::string & config_yaml);

  std::shared_ptr<dynamixel::PortHandler> port_handler_;
  std::shared_ptr<dynamixel::PacketHandler> packet_handler_;
  std::map<JointGroupName, std::vector<JointName>> joint_groups_;
  std::map<JointName, joint::Joint> all_joints_;
  int baudrate_;
};

}  // namespace rt_manipulators_cpp

#endif // HARDWARE_HPP_