#ifndef HARDWARE_HPP_
#define HARDWARE_HPP_

#include <dynamixel_sdk/dynamixel_sdk.h>
#include <memory>
#include <vector>

#include "joint.hpp"

namespace rt_manipulators_cpp
{

class Hardware
{
public:
  Hardware(const std::string device_name);
  bool load_config_file(const std::string & config_yaml);
  bool connect(const int baudrate = 3000000);
  void disconnect();
  bool torque_on();
  // bool torque_off();
  // bool torque_on_all();

private:
  bool parse_config_file(const std::string & config_yaml);

  std::shared_ptr<dynamixel::PortHandler> port_handler_;
  std::shared_ptr<dynamixel::PacketHandler> packet_handler_;
  std::vector<joint::JointGroup> joint_groups_;
  int baudrate_;
};

}  // namespace rt_manipulators_cpp

#endif // HARDWARE_HPP_