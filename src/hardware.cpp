
#include <fstream>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include "hardware.hpp"

namespace rt_manipulators_cpp
{

const double PROTOCOL_VERSION = 2.0;

// Dynamixel XM Series address table
// Ref: https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/
const uint16_t ADDR_TORQUE_ENABLE = 64;
const uint16_t ADDR_POSITION_D_GAIN = 80;
const uint16_t ADDR_POSITION_I_GAIN = 82;
const uint16_t ADDR_POSITION_P_GAIN = 84;
const uint16_t ADDR_GOAL_CURRENT = 102;
const uint16_t ADDR_GOAL_VELOCITY = 104;
const uint16_t ADDR_GOAL_POSITION = 116;
const uint16_t ADDR_PRESENT_CURRENT = 126;
const uint16_t ADDR_PRESENT_VELOCITY = 128;
const uint16_t ADDR_PRESENT_POSITION = 132;
const uint16_t ADDR_PRESENT_VOLTAGE = 144;
const uint16_t ADDR_PRESENT_TEMPERATURE = 146;
const uint16_t ADDR_INDIRECT_ADDRESS_1 = 168;
const uint16_t ADDR_INDIRECT_DATA_1 = 224;
const uint16_t ADDR_INDIRECT_ADDRESS_29 = 578;
const uint16_t ADDR_INDIRECT_DATA_29 = 634;

const uint16_t LEN_GOAL_CURRENT = 2;
const uint16_t LEN_GOAL_VELOCITY = 4;
const uint16_t LEN_GOAL_POSITION = 4;
const uint16_t LEN_PRESENT_CURRENT = 2;
const uint16_t LEN_PRESENT_VELOCITY = 4;
const uint16_t LEN_PRESENT_POSITION = 4;
const uint16_t LEN_PRESENT_VOLTAGE = 2;
const uint16_t LEN_PRESENT_TEMPERATURE = 1;
const uint16_t LEN_INDIRECT_ADDRESS = 2;

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
  return write_1byte_to_group(group_name, ADDR_TORQUE_ENABLE, 1);
}

bool Hardware::torque_off(const std::string & group_name)
{
  return write_1byte_to_group(group_name, ADDR_TORQUE_ENABLE, 0);
}

bool Hardware::parse_config_file(const std::string & config_yaml)
{
  // yamlファイルを読み取り、joint_groups_とall_joints_メンバ変数に格納する
  std::ifstream fs(config_yaml);
  if(!fs.is_open()){
    std::cerr<<"コンフィグファイル:"<<config_yaml<<"が存在しません."<<std::endl;
    return false;
  }

  YAML::Node config = YAML::LoadFile(config_yaml);
  for(auto config_joint_group : config["joint_groups"]){
    auto group_name = config_joint_group.first.as<std::string>();
    if(joint_groups_contain(group_name)){
      std::cerr<<group_name<<"グループが2つ以上存在します."<<std::endl;
      return false;
    }

    for(auto config_joint : config["joint_groups"][group_name]){
      auto joint_name = config_joint.as<std::string>();
      if(all_joints_contain(joint_name)){
        std::cerr<<joint_name<<"ジョイントが2つ以上存在します."<<std::endl;
        return false;
      }
      if(config[joint_name]){
        if(config[joint_name]["id"] && config[joint_name]["operating_mode"]){
          joint_groups_[group_name].push_back(joint_name);
          all_joints_.emplace(joint_name,
            joint::Joint(config[joint_name]["id"].as<int>(),
                        config[joint_name]["operating_mode"].as<int>())
          );
        }else{
          std::cerr<<joint_name<<"にidまたはoperating_modeが設定されていません."<<std::endl;
          return false;
        }
      }else{
        std::cerr<<joint_name<<"ジョイントの設定が存在しません."<<std::endl;
        return false;
      }
    }
  }

  return true;
}

bool Hardware::joint_groups_contain(const std::string & group_name)
{
  return joint_groups_.find(group_name) != joint_groups_.end();
}

bool Hardware::all_joints_contain(const std::string & joint_name)
{
  return all_joints_.find(joint_name) != all_joints_.end();
}

bool Hardware::write_1byte_to_group(const std::string & group_name, const uint16_t address, const uint8_t write_data)
{
  if(!joint_groups_contain(group_name)){
    std::cerr<<group_name<<"はjoint_groupsに存在しません."<<std::endl;
    return false;
  }

  bool retval = true;
  for(auto joint_name : joint_groups_[group_name]){
    auto id = all_joints_.at(joint_name).id();
    if(!write_1byte(id, address, write_data)){
      retval = false;
    }
  }
  return retval;
}

bool Hardware::write_1byte(const uint8_t id, const uint16_t address, const uint8_t write_data)
{
  uint8_t dxl_error = 0;
  int dxl_result = packet_handler_->write1ByteTxRx(port_handler_.get(), id, address, write_data, &dxl_error);

  if(!parse_dxl_error(std::string(__func__), id, address, dxl_result, dxl_error)){
      return false;
  }
  return true;
}

bool Hardware::parse_dxl_error(const std::string & func_name, const uint8_t id,
  const uint16_t address, const int dxl_comm_result, const uint8_t dxl_packet_error)
{
  bool retval = true;

  if (dxl_comm_result != COMM_SUCCESS)
  {
      std::cerr << "Function:" << func_name;
      std::cerr << ", ID:" << std::to_string(id);
      std::cerr << ", Address:" << std::to_string(address);
      std::cerr << ", CommError" << std::string(packet_handler_->getTxRxResult(dxl_comm_result)) << std::endl;
      retval = false;
  }

  if (dxl_packet_error != 0)
  {
      std::cerr << "Function:" << func_name;
      std::cerr << ", ID:" << std::to_string(id);
      std::cerr << ", Address:" << std::to_string(address);
      std::cerr << ", PacketError:" << std::string(packet_handler_->getRxPacketError(dxl_packet_error)) << std::endl;
      retval = false;
  }

  return retval;
}


}  // namespace rt_manipulators_cpp