
#include <iostream>
#include <yaml-cpp/yaml.h>
#include "hardware.hpp"

namespace rt_manipulators_cpp
{

const double PROTOCOL_VERSION = 2.0;

Hardware::Hardware(const std::string device_name)
{
  port_handler_ = std::shared_ptr<dynamixel::PortHandler>(
      dynamixel::PortHandler::getPortHandler(device_name.c_str()));
  packet_handler_ = std::shared_ptr<dynamixel::PacketHandler>(
      dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION));
}

bool Hardware::load_config_file(const std::string & config_yaml)
{
  if(parse_config_file(config_yaml) == false){
    return false;
  }

  std::cout<<"Config file '"<<config_yaml<<"' loaded."<<std::endl;
  for(auto group : joint_groups_){
    std::cout<<group.first <<std::endl;
    for(auto joint_name : group.second){
      std::cout<<"\t"<<joint_name;
      std::cout<<", id:"<<std::to_string(all_joints_.at(joint_name).id());
      std::cout<<", mode:"<<std::to_string(all_joints_.at(joint_name).operating_mode());
      std::cout<<std::endl;
    }
  }

  return true;
}

bool Hardware::connect(const int baudrate)
{
  if(!port_handler_->setBaudRate(baudrate)){
      std::cerr << "Unable to set baudrate: " << std::to_string(baudrate) << std::endl;
      return false;
  }
  if(!port_handler_->openPort()) {
      std::cerr << "Unable to open port: " << port_handler_->getPortName() << std::endl;
      return false;
  }

  return true;
}

void Hardware::disconnect()
{
  port_handler_->closePort();
}

bool Hardware::torque_on(const std::string & group_name)
{
  // for(auto group : joint_groups_){
  //   for(auto joint : group.joints()){
  //     auto dxl_id = joint.id();
  //   }
  // }
}

bool Hardware::parse_config_file(const std::string & config_yaml)
{
  YAML::Node config = YAML::LoadFile(config_yaml);
  for(auto config_joint_group : config["joint_groups"]){
    auto group_name = config_joint_group.first.as<std::string>();

    for(auto config_joint : config["joint_groups"][group_name]){
      auto joint_name = config_joint.as<std::string>();
      joint_groups_[group_name].push_back(joint_name);
      all_joints_.emplace(joint_name,
        joint::Joint(config[joint_name]["id"].as<int>(),
                     config[joint_name]["operating_mode"].as<int>())
      );
    }
  }

  return true;
}


}  // namespace rt_manipulators_cpp