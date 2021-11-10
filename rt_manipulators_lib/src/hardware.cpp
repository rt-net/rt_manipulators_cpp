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

#include "hardware.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>

namespace rt_manipulators_cpp {

// Ref: https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/
// Ref: https://emanual.robotis.com/docs/en/dxl/x/xm540-w270/
// Ref: https://emanual.robotis.com/docs/en/dxl/x/xm540-w150/
const double PROTOCOL_VERSION = 2.0;
const int DXL_HOME_POSITION = 2048;
const double TO_RADIANS = (180.0 / 2048.0) * M_PI / 180.0;
const double TO_DXL_POS = 1.0 / TO_RADIANS;
const double TO_VELOCITY_REV_PER_MIN = 0.229;
const double TO_VELOCITY_RAD_PER_MIN = TO_VELOCITY_REV_PER_MIN * 2.0 * M_PI;
const double TO_VELOCITY_RAD_PER_SEC = TO_VELOCITY_RAD_PER_MIN / 60.0;
const double DXL_VELOCITY_FROM_RAD_PER_SEC = 1.0 / TO_VELOCITY_RAD_PER_SEC;
const int DXL_MAX_VELOCITY = 32767;
const double TO_ACCELERATION_REV_PER_MM = 214.577;
const double TO_ACCELERATION_TO_RAD_PER_MM = TO_ACCELERATION_REV_PER_MM * 2.0 * M_PI;
const double TO_ACCELERATION_TO_RAD_PER_SS = TO_ACCELERATION_TO_RAD_PER_MM / 3600.0;
const double DXL_ACCELERATION_FROM_RAD_PER_SS = 1.0 / TO_ACCELERATION_TO_RAD_PER_SS;
const int DXL_MAX_ACCELERATION = 32767;
const double TO_CURRENT_AMPERE = 0.00269;
const double TO_VOLTAGE_VOLT = 0.1;

// Dynamixel XM Series address table
const uint16_t ADDR_OPERATING_MODE = 11;
const uint16_t ADDR_MAX_POSITION_LIMIT = 48;
const uint16_t ADDR_MIN_POSITION_LIMIT = 52;
const uint16_t ADDR_TORQUE_ENABLE = 64;
const uint16_t ADDR_VELOCITY_I_GAIN = 76;
const uint16_t ADDR_VELOCITY_P_GAIN = 78;
const uint16_t ADDR_POSITION_D_GAIN = 80;
const uint16_t ADDR_POSITION_I_GAIN = 82;
const uint16_t ADDR_POSITION_P_GAIN = 84;
const uint16_t ADDR_GOAL_CURRENT = 102;
const uint16_t ADDR_GOAL_VELOCITY = 104;
const uint16_t ADDR_GOAL_POSITION = 116;
const uint16_t ADDR_PROFILE_ACCELERATION = 108;
const uint16_t ADDR_PROFILE_VELOCITY = 112;
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

Hardware::Hardware(const std::string device_name) :
  is_connected_(false), thread_enable_(false) {
  port_handler_ = std::shared_ptr<dynamixel::PortHandler>(
      dynamixel::PortHandler::getPortHandler(device_name.c_str()));
  packet_handler_ = std::shared_ptr<dynamixel::PacketHandler>(
      dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION));
}

Hardware::~Hardware() {
  stop_thread();
  disconnect();
}

bool Hardware::load_config_file(const std::string& config_yaml) {
  // コンフィグファイルの読み込み
  if (parse_config_file(config_yaml) == false) {
    return false;
  }

  // sync_readとsync_writeの関係をチェック
  for (const auto& [group_name, group] : joint_groups_) {
    if (group->sync_write_velocity_enabled() && !group->sync_read_position_enabled()) {
      std::cerr << group_name << "グループはvelocityをsync_writeしますが, ";
      std::cerr << "positionをsync_readしません." << std::endl;
      std::cerr << "positionもsync_readするようにコンフィグファイルを修正して下さい." << std::endl;
      return false;
    }
  }

  std::cout << "Config file '" << config_yaml << "' loaded." << std::endl;
  for (const auto& [group_name, group] : joint_groups_) {
    std::cout << group_name << std::endl;
    for (const auto & joint_name : group->joint_names()) {
      std::cout << "\t" << joint_name;
      std::cout << ", id:" << std::to_string(all_joints_.at(joint_name)->id());
      std::cout << ", mode:" << std::to_string(all_joints_.at(joint_name)->operating_mode());
      std::cout << std::endl;
    }
  }

  return true;
}

bool Hardware::connect(const int baudrate) {
  if (!port_handler_->setBaudRate(baudrate)) {
    std::cerr << "Unable to set baudrate: " << std::to_string(baudrate) << std::endl;
    return false;
  }
  if (!port_handler_->openPort()) {
    std::cerr << "Unable to open port: " << port_handler_->getPortName() << std::endl;
    return false;
  }

  is_connected_ = true;
  return true;
}

void Hardware::disconnect() {
  if (is_connected_ == false) {
    return;
  }

  // 速度指示モードの場合は、goal_velocityを0にする
  for (const auto& [group_name, group] : joint_groups_) {
    if (group->sync_write_velocity_enabled()) {
      std::cout << group_name << "グループにはvelocityのsync_writeが設定されています." << std::endl;
      std::cout << "安全のため, disconnect()関数内で目標速度 0 rad/sを書き込みます." << std::endl;
      for (const auto & joint_name : group->joint_names()) {
          all_joints_.at(joint_name)->set_goal_velocity(0);
      }
      sync_write(group_name);
    }
  }

  port_handler_->closePort();
  is_connected_ = false;
}

bool Hardware::torque_on(const std::string& group_name) {
  if (!write_byte_data_to_group(group_name, ADDR_TORQUE_ENABLE, 1)) {
    std::cerr << group_name << "グループのトルクONに失敗しました." << std::endl;
    return false;
  }

  // 安全のため、サーボの現在角度を目標角度に設定する
  if (!sync_read(group_name)) {
    std::cerr << group_name << "グループのトルクON後のsync_readに失敗しました." << std::endl;
    return false;
  }
  for (const auto & joint_name : joint_groups_[group_name]->joint_names()) {
    all_joints_.at(joint_name)
        ->set_goal_position(all_joints_.at(joint_name)->get_present_position());
  }
  return true;
}

bool Hardware::torque_off(const std::string& group_name) {
  return write_byte_data_to_group(group_name, ADDR_TORQUE_ENABLE, 0);
}

bool Hardware::sync_read(const std::string& group_name) {
  // 指定されたグループのsyncReadを実行して、サーボモータからデータを読み取る
  // 読み取ったデータはメンバ変数に格納する
  if (!joint_groups_contain(group_name)) {
    std::cerr << group_name << "はjoint_groupsに存在しません." << std::endl;
    return false;
  }

  if (!sync_read_groups_[group_name]) {
    std::cerr << group_name << "にはsync_readが設定されていません." << std::endl;
    return false;
  }

  int dxl_result = sync_read_groups_[group_name]->txRxPacket();
  if (!parse_dxl_error(std::string(__func__), dxl_result)) {
    std::cerr << group_name << "のsync readに失敗しました." << std::endl;
    return false;
  }

  auto get_data = [this](auto group_name, auto joint_name, auto addr, auto len, auto & data){
    auto id = all_joints_.at(joint_name)->id();
    if (!sync_read_groups_[group_name]->isAvailable(id, addr, len)) {
      std::cerr << "id: " << std::to_string(id);
      std::cerr << ", addr: " << std::to_string(addr);
      std::cerr << ", len: " << std::to_string(len);
      std::cerr << " is not available." << std::endl;
      return false;
    } else {
      data = sync_read_groups_[group_name]->getData(id, addr, len);
      return true;
    }
  };

  bool retval = true;
  if (joint_groups_[group_name]->sync_read_position_enabled()) {
    for (const auto & joint_name : joint_groups_[group_name]->joint_names()) {
      int32_t data = 0;
      if (get_data(group_name, joint_name, addr_sync_read_position_[group_name],
                  LEN_PRESENT_POSITION, data)) {
        all_joints_.at(joint_name)->set_present_position(dxl_pos_to_radian(data));
      } else {
        std::cerr << joint_name << "のpresent_positionを取得できません." << std::endl;
        retval = false;
      }
    }
  }

  if (joint_groups_[group_name]->sync_read_velocity_enabled()) {
    for (const auto & joint_name : joint_groups_[group_name]->joint_names()) {
      int32_t data = 0;
      if (get_data(group_name, joint_name, addr_sync_read_velocity_[group_name],
                  LEN_PRESENT_VELOCITY, data)) {
        all_joints_.at(joint_name)->set_present_velocity(dxl_velocity_to_rps(data));
      } else {
        std::cerr << joint_name << "のpresent_velocityを取得できません." << std::endl;
        retval = false;
      }
    }
  }

  if (joint_groups_[group_name]->sync_read_current_enabled()) {
    for (const auto & joint_name : joint_groups_[group_name]->joint_names()) {
      int16_t data = 0;
      if (get_data(group_name, joint_name, addr_sync_read_current_[group_name],
                  LEN_PRESENT_CURRENT, data)) {
        all_joints_.at(joint_name)->set_present_current(dxl_current_to_ampere(data));
      } else {
        std::cerr << joint_name << "のpresent_currentを取得できません." << std::endl;
        retval = false;
      }
    }
  }

  if (joint_groups_[group_name]->sync_read_voltage_enabled()) {
    for (const auto & joint_name : joint_groups_[group_name]->joint_names()) {
      int16_t data = 0;
      if (get_data(group_name, joint_name, addr_sync_read_voltage_[group_name],
                  LEN_PRESENT_VOLTAGE, data)) {
        all_joints_.at(joint_name)->set_present_voltage(dxl_voltage_to_volt(data));
      } else {
        std::cerr << joint_name << "のpresent_voltageを取得できません." << std::endl;
        retval = false;
      }
    }
  }

  if (joint_groups_[group_name]->sync_read_temperature_enabled()) {
    for (const auto & joint_name : joint_groups_[group_name]->joint_names()) {
      int8_t data = 0;
      if (get_data(group_name, joint_name, addr_sync_read_temperature_[group_name],
                  LEN_PRESENT_TEMPERATURE, data)) {
        all_joints_.at(joint_name)->set_present_temperature(data);
      } else {
        std::cerr << joint_name << "のpresent_temperatureを取得できません." << std::endl;
        retval = false;
      }
    }
  }

  return retval;
}

bool Hardware::sync_write(const std::string& group_name) {
  // 指定されたグループのsyncWriteを実行して、サーボモータにデータを書き込む
  // 書き込むデータはメンバ変数から取り出す
  if (!joint_groups_contain(group_name)) {
    std::cerr << group_name << "はjoint_groupsに存在しません." << std::endl;
    return false;
  }

  if (!sync_write_groups_[group_name]) {
    std::cerr << group_name << "にはsync_writeが設定されていません." << std::endl;
    return false;
  }

  for (const auto & joint_name : joint_groups_[group_name]->joint_names()) {
    std::vector<uint8_t> write_data;
    if (joint_groups_[group_name]->sync_write_position_enabled()) {
      uint32_t goal_position = radian_to_dxl_pos(all_joints_.at(joint_name)->get_goal_position());
      write_data.push_back(DXL_LOBYTE(DXL_LOWORD(goal_position)));
      write_data.push_back(DXL_HIBYTE(DXL_LOWORD(goal_position)));
      write_data.push_back(DXL_LOBYTE(DXL_HIWORD(goal_position)));
      write_data.push_back(DXL_HIBYTE(DXL_HIWORD(goal_position)));
    }

    if (joint_groups_[group_name]->sync_write_velocity_enabled()) {
      uint32_t goal_velocity = to_dxl_velocity(all_joints_.at(joint_name)->get_goal_velocity());
      write_data.push_back(DXL_LOBYTE(DXL_LOWORD(goal_velocity)));
      write_data.push_back(DXL_HIBYTE(DXL_LOWORD(goal_velocity)));
      write_data.push_back(DXL_LOBYTE(DXL_HIWORD(goal_velocity)));
      write_data.push_back(DXL_HIBYTE(DXL_HIWORD(goal_velocity)));
    }

    auto id = all_joints_.at(joint_name)->id();
    if (!sync_write_groups_[group_name]->changeParam(id, write_data.data())) {
      std::cerr << group_name << ":" << std::to_string(id)
                << " のsyncWrite->changeParamに失敗しました." << std::endl;
      return false;
    }
  }

  int dxl_result = sync_write_groups_[group_name]->txPacket();
  if (!parse_dxl_error(std::string(__func__), dxl_result)) {
    std::cerr << group_name << "のsync writeに失敗しました." << std::endl;
    return false;
  }

  return true;
}

bool Hardware::start_thread(const std::vector<std::string>& group_names,
                            const std::chrono::milliseconds& update_cycle_ms) {
  // read, writeを繰り返すスレッドを開始する
  for (const auto & group_name : group_names) {
    if (!joint_groups_contain(group_name)) {
      std::cerr << group_name << "はjoint_groupsに存在しません." << std::endl;
      return false;
    }
  }

  if (thread_enable_) {
    std::cerr << "すでにスレッドが立ち上がっています." << std::endl;
    return false;
  }

  thread_enable_ = true;
  read_write_thread_ = std::make_shared<std::thread>(&Hardware::read_write_thread, this,
                                                     group_names, update_cycle_ms);

  return true;
}

bool Hardware::stop_thread() {
  // スレッド内部の無限ループを停止する

  // start_thread()を実行し、リソースを確保していなければfalseを返して終了
  if (!read_write_thread_) {
    return false;
  }

  thread_enable_ = false;

  // スレッドが停止するまで待機
  if (read_write_thread_->joinable()) {
    read_write_thread_->join();
  }

  // リソースを解放
  read_write_thread_.reset();

  // 速度指示モードの場合は、goal_velocityを0にする
  for (auto& [group_name, group] : joint_groups_) {
    if (group->sync_write_velocity_enabled()) {
      std::cout << group_name << "グループにはvelocityのsync_writeが設定されています." << std::endl;
      std::cout << "安全のため, stop_thread()関数内で目標速度 0 rad/sを書き込みます." << std::endl;
      for (const auto & joint_name : group->joint_names()) {
          all_joints_.at(joint_name)->set_goal_velocity(0);
      }
      sync_write(group_name);
    }
  }

  return true;
}

bool Hardware::get_position(const uint8_t id, double& position) {
  if (!all_joints_contain_id(id)) {
    std::cerr << "ID:" << std::to_string(id) << "のジョイントは存在しません." << std::endl;
    return false;
  }
  position = all_joints_ref_from_id_[id]->get_present_position();
  return true;
}

bool Hardware::get_position(const std::string& joint_name, double& position) {
  if (!all_joints_contain(joint_name)) {
    std::cerr << joint_name << "ジョイントは存在しません." << std::endl;
    return false;
  }
  position = all_joints_[joint_name]->get_present_position();
  return true;
}

bool Hardware::get_positions(const std::string& group_name, std::vector<double>& positions) {
  if (!joint_groups_contain(group_name)) {
    std::cerr << group_name << "はjoint_groupsに存在しません." << std::endl;
    return false;
  }

  for (const auto & joint_name : joint_groups_[group_name]->joint_names()) {
    positions.push_back(all_joints_.at(joint_name)->get_present_position());
  }
  return true;
}

bool Hardware::get_velocity(const uint8_t id, double& velocity) {
  if (!all_joints_contain_id(id)) {
    std::cerr << "ID:" << std::to_string(id) << "のジョイントは存在しません." << std::endl;
    return false;
  }
  velocity = all_joints_ref_from_id_[id]->get_present_velocity();
  return true;
}

bool Hardware::get_velocity(const std::string& joint_name, double& velocity) {
  if (!all_joints_contain(joint_name)) {
    std::cerr << joint_name << "ジョイントは存在しません." << std::endl;
    return false;
  }
  velocity = all_joints_[joint_name]->get_present_velocity();
  return true;
}

bool Hardware::get_velocities(const std::string& group_name, std::vector<double>& velocities) {
  if (!joint_groups_contain(group_name)) {
    std::cerr << group_name << "はjoint_groupsに存在しません." << std::endl;
    return false;
  }

  for (const auto & joint_name : joint_groups_[group_name]->joint_names()) {
    velocities.push_back(all_joints_.at(joint_name)->get_present_velocity());
  }
  return true;
}

bool Hardware::get_current(const uint8_t id, double& current) {
  if (!all_joints_contain_id(id)) {
    std::cerr << "ID:" << std::to_string(id) << "のジョイントは存在しません." << std::endl;
    return false;
  }
  current = all_joints_ref_from_id_[id]->get_present_current();
  return true;
}

bool Hardware::get_current(const std::string& joint_name, double& current) {
  if (!all_joints_contain(joint_name)) {
    std::cerr << joint_name << "ジョイントは存在しません." << std::endl;
    return false;
  }
  current = all_joints_[joint_name]->get_present_current();
  return true;
}

bool Hardware::get_currents(const std::string& group_name, std::vector<double>& currents) {
  if (!joint_groups_contain(group_name)) {
    std::cerr << group_name << "はjoint_groupsに存在しません." << std::endl;
    return false;
  }

  for (const auto & joint_name : joint_groups_[group_name]->joint_names()) {
    currents.push_back(all_joints_.at(joint_name)->get_present_current());
  }
  return true;
}

bool Hardware::get_voltage(const uint8_t id, double& voltage) {
  if (!all_joints_contain_id(id)) {
    std::cerr << "ID:" << std::to_string(id) << "のジョイントは存在しません." << std::endl;
    return false;
  }
  voltage = all_joints_ref_from_id_[id]->get_present_voltage();
  return true;
}

bool Hardware::get_voltage(const std::string& joint_name, double& voltage) {
  if (!all_joints_contain(joint_name)) {
    std::cerr << joint_name << "ジョイントは存在しません." << std::endl;
    return false;
  }
  voltage = all_joints_[joint_name]->get_present_voltage();
  return true;
}

bool Hardware::get_voltages(const std::string& group_name, std::vector<double>& voltages) {
  if (!joint_groups_contain(group_name)) {
    std::cerr << group_name << "はjoint_groupsに存在しません." << std::endl;
    return false;
  }

  for (const auto & joint_name : joint_groups_[group_name]->joint_names()) {
    voltages.push_back(all_joints_.at(joint_name)->get_present_voltage());
  }
  return true;
}

bool Hardware::get_temperature(const uint8_t id, int8_t& temperature) {
  if (!all_joints_contain_id(id)) {
    std::cerr << "ID:" << std::to_string(id) << "のジョイントは存在しません." << std::endl;
    return false;
  }
  temperature = all_joints_ref_from_id_[id]->get_present_temperature();
  return true;
}

bool Hardware::get_temperature(const std::string& joint_name, int8_t& temperature) {
  if (!all_joints_contain(joint_name)) {
    std::cerr << joint_name << "ジョイントは存在しません." << std::endl;
    return false;
  }
  temperature = all_joints_[joint_name]->get_present_temperature();
  return true;
}

bool Hardware::get_temperatures(const std::string& group_name, std::vector<int8_t>& temperatures) {
  if (!joint_groups_contain(group_name)) {
    std::cerr << group_name << "はjoint_groupsに存在しません." << std::endl;
    return false;
  }

  for (const auto & joint_name : joint_groups_[group_name]->joint_names()) {
    temperatures.push_back(all_joints_.at(joint_name)->get_present_temperature());
  }
  return true;
}

bool Hardware::set_position(const uint8_t id, const double position) {
  if (!all_joints_contain_id(id)) {
    std::cerr << "ID:" << std::to_string(id) << "のジョイントは存在しません." << std::endl;
    return false;
  }
  all_joints_ref_from_id_[id]->set_goal_position(position);
  return true;
}

bool Hardware::set_position(const std::string& joint_name, const double position) {
  if (!all_joints_contain(joint_name)) {
    std::cerr << joint_name << "ジョイントは存在しません." << std::endl;
    return false;
  }
  all_joints_[joint_name]->set_goal_position(position);
  return true;
}

bool Hardware::set_positions(const std::string& group_name, std::vector<double>& positions) {
  if (!joint_groups_contain(group_name)) {
    std::cerr << group_name << "はjoint_groupsに存在しません." << std::endl;
    return false;
  }

  if (joint_groups_[group_name]->joint_names().size() != positions.size()) {
    std::cerr << "目標値のサイズ:" << positions.size();
    std::cerr << "がジョイント数:" << joint_groups_[group_name]->joint_names().size();
    std::cerr << "と一致しません." << std::endl;
    return false;
  }

  for (size_t i = 0; i < positions.size(); i++) {
    auto joint_name = joint_groups_[group_name]->joint_names()[i];
    all_joints_[joint_name]->set_goal_position(positions[i]);
  }
  return true;
}

bool Hardware::set_velocity(const uint8_t id, const double velocity) {
  if (!all_joints_contain_id(id)) {
    std::cerr << "ID:" << std::to_string(id) << "のジョイントは存在しません." << std::endl;
    return false;
  }

  if (!thread_enable_) {
    std::cerr << "目標速度を書き込む場合は、";
    std::cerr << "安全のためstart_thread()を実行してください." << std::endl;
    return false;
  }

  all_joints_ref_from_id_[id]->set_goal_velocity(velocity);
  return true;
}

bool Hardware::set_velocity(const std::string& joint_name, const double velocity) {
  if (!all_joints_contain(joint_name)) {
    std::cerr << joint_name << "ジョイントは存在しません." << std::endl;
    return false;
  }

  if (!thread_enable_) {
    std::cerr << "目標速度を書き込む場合は、";
    std::cerr << "安全のためstart_thread()を実行してください." << std::endl;
    return false;
  }

  all_joints_[joint_name]->set_goal_velocity(velocity);
  return true;
}

bool Hardware::set_velocities(const std::string& group_name, std::vector<double>& velocities) {
  if (!joint_groups_contain(group_name)) {
    std::cerr << group_name << "はjoint_groupsに存在しません." << std::endl;
    return false;
  }

  if (joint_groups_[group_name]->joint_names().size() != velocities.size()) {
    std::cerr << "目標値のサイズ:" << velocities.size();
    std::cerr << "がジョイント数:" << joint_groups_[group_name]->joint_names().size();
    std::cerr << "と一致しません." << std::endl;
    return false;
  }

  if (!thread_enable_) {
    std::cerr << "目標速度を書き込む場合は、";
    std::cerr << "安全のためstart_thread()を実行してください." << std::endl;
    return false;
  }

  for (size_t i = 0; i < velocities.size(); i++) {
    auto joint_name = joint_groups_[group_name]->joint_names()[i];
    all_joints_[joint_name]->set_goal_velocity(velocities[i]);
  }
  return true;
}

bool Hardware::write_max_acceleration_to_group(const std::string& group_name,
                                               const double acceleration_rpss) {
  // 指定されたグループ内のサーボモータの最大動作加速度（radian / s^2）を設定する
  if (!joint_groups_contain(group_name)) {
    std::cerr << group_name << "はjoint_groupsに存在しません." << std::endl;
    return false;
  }

  auto dxl_acceleration = to_dxl_acceleration(acceleration_rpss);
  if (!write_double_word_data_to_group(group_name, ADDR_PROFILE_ACCELERATION, dxl_acceleration)) {
    std::cerr << group_name << "グループのProfile Accelerationの書き込みに失敗しました."
              << std::endl;
    return false;
  }

  return true;
}

bool Hardware::write_max_velocity_to_group(const std::string& group_name,
                                           const double velocity_rps) {
  // 指定されたグループ内のサーボモータの最大動作加速度（radian / s^2）を設定する
  if (!joint_groups_contain(group_name)) {
    std::cerr << group_name << "はjoint_groupsに存在しません." << std::endl;
    return false;
  }

  auto dxl_velocity = to_dxl_profile_velocity(velocity_rps);
  if (!write_double_word_data_to_group(group_name, ADDR_PROFILE_VELOCITY, dxl_velocity)) {
    std::cerr << group_name << "グループのProfile Velocityの書き込みに失敗しました." << std::endl;
    return false;
  }

  return true;
}

bool Hardware::write_position_pid_gain(const uint8_t id, const uint16_t p, const uint16_t i,
                               const uint16_t d) {
  // 指定されたサーボモータの位置制御PIDゲインを設定する
  if (!all_joints_contain_id(id)) {
    std::cerr << "ID:" << std::to_string(id) << "のジョイントは存在しません." << std::endl;
    return false;
  }

  if (!write_word_data(id, ADDR_POSITION_P_GAIN, p)) {
    std::cerr << "ID:" << std::to_string(id);
    std::cerr << "のPosition P Gainの書き込みに失敗しました." << std::endl;
    return false;
  }

  if (!write_word_data(id, ADDR_POSITION_I_GAIN, i)) {
    std::cerr << "ID:" << std::to_string(id);
    std::cerr << "のPosition I Gainの書き込みに失敗しました." << std::endl;
    return false;
  }

  if (!write_word_data(id, ADDR_POSITION_D_GAIN, d)) {
    std::cerr << "ID:" << std::to_string(id);
    std::cerr << "のPosition D Gainの書き込みに失敗しました." << std::endl;
    return false;
  }

  return true;
}

bool Hardware::write_position_pid_gain(const std::string& joint_name, const uint16_t p,
                                       const uint16_t i, const uint16_t d) {
  // 指定されたサーボモータの位置制御PIDゲインを設定する
  if (!all_joints_contain(joint_name)) {
    std::cerr << joint_name << "ジョイントは存在しません." << std::endl;
    return false;
  }

  return write_position_pid_gain(all_joints_.at(joint_name)->id(), p, i, d);
}

bool Hardware::write_position_pid_gain_to_group(const std::string& group_name, const uint16_t p,
                                                const uint16_t i, const uint16_t d) {
  // 指定されたグループ内のサーボモータの位置制御PIDゲインを設定する
  if (!joint_groups_contain(group_name)) {
    std::cerr << group_name << "はjoint_groupsに存在しません." << std::endl;
    return false;
  }

  if (!write_word_data_to_group(group_name, ADDR_POSITION_P_GAIN, p)) {
    std::cerr << group_name << "グループのPosition P Gainの書き込みに失敗しました." << std::endl;
    return false;
  }

  if (!write_word_data_to_group(group_name, ADDR_POSITION_I_GAIN, i)) {
    std::cerr << group_name << "グループのPosition I Gainの書き込みに失敗しました." << std::endl;
    return false;
  }

  if (!write_word_data_to_group(group_name, ADDR_POSITION_D_GAIN, d)) {
    std::cerr << group_name << "グループのPosition D Gainの書き込みに失敗しました." << std::endl;
    return false;
  }

  return true;
}

bool Hardware::write_velocity_pi_gain(const uint8_t id, const uint16_t p, const uint16_t i) {
  // 指定されたサーボモータの速度制御PIゲインを設定する
  if (!all_joints_contain_id(id)) {
    std::cerr << "ID:" << std::to_string(id) << "のジョイントは存在しません." << std::endl;
    return false;
  }

  if (!write_word_data(id, ADDR_VELOCITY_P_GAIN, p)) {
    std::cerr << "ID:" << std::to_string(id);
    std::cerr << "のVelocity P Gainの書き込みに失敗しました." << std::endl;
    return false;
  }

  if (!write_word_data(id, ADDR_VELOCITY_I_GAIN, i)) {
    std::cerr << "ID:" << std::to_string(id);
    std::cerr << "のVelocity I Gainの書き込みに失敗しました." << std::endl;
    return false;
  }

  return true;
}

bool Hardware::write_velocity_pi_gain(const std::string& joint_name, const uint16_t p,
                                      const uint16_t i) {
  // 指定されたサーボモータの速度制御PIゲインを設定する
  if (!all_joints_contain(joint_name)) {
    std::cerr << joint_name << "ジョイントは存在しません." << std::endl;
    return false;
  }

  return write_velocity_pi_gain(all_joints_.at(joint_name)->id(), p, i);
}

bool Hardware::write_velocity_pi_gain_to_group(const std::string& group_name, const uint16_t p,
                                       const uint16_t i) {
  // 指定されたグループ内のサーボモータの速度制御PIゲインを設定する
  if (!joint_groups_contain(group_name)) {
    std::cerr << group_name << "はjoint_groupsに存在しません." << std::endl;
    return false;
  }

  if (!write_word_data_to_group(group_name, ADDR_VELOCITY_P_GAIN, p)) {
    std::cerr << group_name << "グループのVelocity P Gainの書き込みに失敗しました." << std::endl;
    return false;
  }

  if (!write_word_data_to_group(group_name, ADDR_VELOCITY_I_GAIN, i)) {
    std::cerr << group_name << "グループのVelocity I Gainの書き込みに失敗しました." << std::endl;
    return false;
  }

  return true;
}

bool Hardware::write_byte_data(const uint8_t id, const uint16_t address, const uint8_t write_data) {
  uint8_t dxl_error = 0;
  int dxl_result =
      packet_handler_->write1ByteTxRx(port_handler_.get(), id, address, write_data, &dxl_error);

  if (!parse_dxl_error(std::string(__func__), id, address, dxl_result, dxl_error)) {
    return false;
  }
  return true;
}

bool Hardware::write_byte_data_to_group(const std::string& group_name, const uint16_t address,
                                        const uint8_t write_data) {
  if (!joint_groups_contain(group_name)) {
    std::cerr << group_name << "はjoint_groupsに存在しません." << std::endl;
    return false;
  }

  bool retval = true;
  for (const auto & joint_name : joint_groups_[group_name]->joint_names()) {
    auto id = all_joints_.at(joint_name)->id();
    if (!write_byte_data(id, address, write_data)) {
      retval = false;
    }
  }
  return retval;
}

bool Hardware::write_word_data(const uint8_t id, const uint16_t address,
                               const uint16_t write_data) {
  uint8_t dxl_error = 0;
  int dxl_result =
      packet_handler_->write2ByteTxRx(port_handler_.get(), id, address, write_data, &dxl_error);

  if (!parse_dxl_error(std::string(__func__), id, address, dxl_result, dxl_error)) {
    return false;
  }
  return true;
}

bool Hardware::write_word_data_to_group(const std::string& group_name, const uint16_t address,
                                        const uint16_t write_data) {
  if (!joint_groups_contain(group_name)) {
    std::cerr << group_name << "はjoint_groupsに存在しません." << std::endl;
    return false;
  }

  bool retval = true;
  for (const auto & joint_name : joint_groups_[group_name]->joint_names()) {
    auto id = all_joints_.at(joint_name)->id();
    if (!write_word_data(id, address, write_data)) {
      retval = false;
    }
  }
  return retval;
}

bool Hardware::write_double_word_data(const uint8_t id, const uint16_t address,
                                      const uint32_t write_data) {
  uint8_t dxl_error = 0;
  int dxl_result =
      packet_handler_->write4ByteTxRx(port_handler_.get(), id, address, write_data, &dxl_error);

  if (!parse_dxl_error(std::string(__func__), id, address, dxl_result, dxl_error)) {
    return false;
  }
  return true;
}

bool Hardware::write_double_word_data_to_group(const std::string& group_name,
                                               const uint16_t address, const uint32_t write_data) {
  if (!joint_groups_contain(group_name)) {
    std::cerr << group_name << "はjoint_groupsに存在しません." << std::endl;
    return false;
  }

  bool retval = true;
  for (const auto & joint_name : joint_groups_[group_name]->joint_names()) {
    auto id = all_joints_.at(joint_name)->id();
    if (!write_double_word_data(id, address, write_data)) {
      retval = false;
    }
  }
  return retval;
}

bool Hardware::read_byte_data(const uint8_t id, const uint16_t address, uint8_t& read_data) {
  uint8_t dxl_error = 0;
  uint8_t data = 0;
  int dxl_result =
      packet_handler_->read1ByteTxRx(port_handler_.get(), id, address, &data, &dxl_error);

  if (!parse_dxl_error(std::string(__func__), id, address, dxl_result, dxl_error)) {
    return false;
  }
  read_data = data;
  return true;
}

bool Hardware::read_double_word_data(const uint8_t id, const uint16_t address,
                                     uint32_t& read_data) {
  uint8_t dxl_error = 0;
  uint32_t data = 0;
  int dxl_result =
      packet_handler_->read4ByteTxRx(port_handler_.get(), id, address, &data, &dxl_error);

  if (!parse_dxl_error(std::string(__func__), id, address, dxl_result, dxl_error)) {
    return false;
  }
  read_data = data;
  return true;
}

bool Hardware::parse_config_file(const std::string& config_yaml) {
  // yamlファイルを読み取り、joint_groups_とall_joints_メンバ変数に格納する
  std::ifstream fs(config_yaml);
  if (!fs.is_open()) {
    std::cerr << "コンフィグファイル:" << config_yaml << "が存在しません." << std::endl;
    return false;
  }

  YAML::Node config = YAML::LoadFile(config_yaml);
  for (const auto & config_joint_group : config["joint_groups"]) {
    auto group_name = config_joint_group.first.as<std::string>();
    if (joint_groups_contain(group_name)) {
      std::cerr << group_name << "グループが2つ以上存在します." << std::endl;
      return false;
    }

    if (!config["joint_groups"][group_name]["joints"]) {
      std::cerr << group_name << "グループに'joints'が設定されていせん。" << std::endl;
      return false;
    }

    std::vector<JointName> joint_names;
    for (const auto & config_joint : config["joint_groups"][group_name]["joints"]) {
      auto joint_name = config_joint.as<std::string>();
      if (all_joints_contain(joint_name)) {
        std::cerr << joint_name << "ジョイントが2つ以上存在します." << std::endl;
        return false;
      }

      if (!config[joint_name]) {
        std::cerr << joint_name << "ジョイントの設定が存在しません." << std::endl;
        return false;
      }

      if (!config[joint_name]["id"] || !config[joint_name]["operating_mode"]) {
        std::cerr << joint_name << "にidまたはoperating_modeが設定されていません." << std::endl;
        return false;
      }

      joint_names.push_back(joint_name);
      auto joint_id = config[joint_name]["id"].as<int>();
      auto ope_mode = config[joint_name]["operating_mode"].as<int>();

      uint32_t dxl_max_pos_limit = 0;
      uint32_t dxl_min_pos_limit = 0;
      if (!read_double_word_data(joint_id, ADDR_MAX_POSITION_LIMIT, dxl_max_pos_limit)) {
        std::cerr << joint_name << "のMax Position Limitの読み取りに失敗しました." << std::endl;
        return false;
      }
      if (!read_double_word_data(joint_id, ADDR_MIN_POSITION_LIMIT, dxl_min_pos_limit)) {
        std::cerr << joint_name << "のMin Position Limitの読み取りに失敗しました." << std::endl;
        return false;
      }

      // 角度リミット値をラジアンに変換
      double max_position_limit = dxl_pos_to_radian(dxl_max_pos_limit);
      double min_position_limit = dxl_pos_to_radian(dxl_min_pos_limit);

      // 角度リミット値にマージンを加算する
      if (config[joint_name]["pos_limit_margin"]) {
        max_position_limit -= config[joint_name]["pos_limit_margin"].as<double>();
        min_position_limit += config[joint_name]["pos_limit_margin"].as<double>();
      }

      auto joint_ptr = std::make_shared<joint::Joint>(
        joint_id, ope_mode, max_position_limit, min_position_limit);
      all_joints_.emplace(joint_name, joint_ptr);
      all_joints_ref_from_id_.emplace(joint_id, joint_ptr);  // IDからもJointにアクセスできる
    }
    std::vector<std::string> sync_read_targets;
    std::vector<std::string> sync_write_targets;
    if (config["joint_groups"][group_name]["sync_read"]) {
      sync_read_targets =
          config["joint_groups"][group_name]["sync_read"].as<std::vector<std::string>>();
    }
    if (config["joint_groups"][group_name]["sync_write"]) {
      sync_write_targets =
          config["joint_groups"][group_name]["sync_write"].as<std::vector<std::string>>();
    }

    auto joint_group_ptr =
        std::make_shared<joint::JointGroup>(joint_names, sync_read_targets, sync_write_targets);
    joint_groups_.emplace(group_name, joint_group_ptr);

    if (!write_operating_mode(group_name)) {
      std::cerr << group_name << "のOperating Modeを設定できません." << std::endl;
      return false;
    }

    if (!create_sync_read_group(group_name)) {
      std::cerr << group_name << "のsync readグループを作成できません." << std::endl;
      return false;
    }
    if (!create_sync_write_group(group_name)) {
      std::cerr << group_name << "のsync writeグループを作成できません." << std::endl;
      return false;
    }
  }

  return true;
}

bool Hardware::joint_groups_contain(const std::string& group_name) {
  return joint_groups_.find(group_name) != joint_groups_.end();
}

bool Hardware::all_joints_contain(const std::string& joint_name) {
  return all_joints_.find(joint_name) != all_joints_.end();
}

bool Hardware::all_joints_contain_id(const uint8_t id) {
  return all_joints_ref_from_id_.find(id) != all_joints_ref_from_id_.end();
}

bool Hardware::write_operating_mode(const std::string& group_name) {
  // サーボモータから動作モードを読み取り、
  // コンフィグファイルと一致しない場合、動作モードを書き込む
  // 動作モードはROM領域にあるため、データ書き込み回数を抑えたい
  const std::vector<uint8_t> SUPPORT_MODE_LIST = {0, 1, 3, 5};

  for (const auto & joint_name : joint_groups_[group_name]->joint_names()) {
    auto target_ope_mode = all_joints_.at(joint_name)->operating_mode();
    auto result = std::find(SUPPORT_MODE_LIST.begin(), SUPPORT_MODE_LIST.end(), target_ope_mode);
    if (result == SUPPORT_MODE_LIST.end()) {
      std::cout << joint_name << "にライブラリがサポートしないOperating Mode:";
      std::cout << std::to_string(target_ope_mode) << "が設定されています." << std::endl;
      return false;
    }

    auto id = all_joints_.at(joint_name)->id();
    uint8_t present_ope_mode;
    if (!read_byte_data(id, ADDR_OPERATING_MODE, present_ope_mode)) {
      std::cout << joint_name << "ジョイントのOperating Modeを読み取れません." << std::endl;
      return false;
    }

    if (target_ope_mode != present_ope_mode) {
      if (!write_byte_data(id, ADDR_OPERATING_MODE, target_ope_mode)) {
        std::cout << joint_name << "ジョイントにOperating Modeを書き込めません." << std::endl;
        std::cout << "トルクがONになっている場合はOFFしてください." << std::endl;
        return false;
      }
      std::cout << joint_name << "ジョイントにOperating Mode:";
      std::cout << std::to_string(target_ope_mode) << "を書き込みました." << std::endl;
    }
  }

  return true;
}

bool Hardware::limit_goal_velocity_by_present_position(const std::string& group_name) {
  // ジョイントの現在角度がmax/min position limit を超えた場合、goal_velocityを0にする
  bool retval = true;
  for (const auto & joint_name : joint_groups_[group_name]->joint_names()) {
    auto max_position_limit = all_joints_.at(joint_name)->max_position_limit();
    auto min_position_limit = all_joints_.at(joint_name)->min_position_limit();
    auto present_position = all_joints_.at(joint_name)->get_present_position();
    auto goal_velocity = all_joints_.at(joint_name)->get_goal_velocity();
    bool has_exceeded_max_pos_limit = present_position > max_position_limit && goal_velocity > 0;
    bool has_exceeded_min_pos_limit = present_position < min_position_limit && goal_velocity < 0;

    if (has_exceeded_max_pos_limit || has_exceeded_min_pos_limit) {
      std::cout << joint_name << "ジョイントの現在角度が限界角度に到達しました、";
      std::cout << "goal_velocityを0で上書きします." << std::endl;
      all_joints_.at(joint_name)->set_goal_velocity(0);
      retval = false;
    }
  }

  return retval;
}

bool Hardware::create_sync_read_group(const std::string& group_name) {
  // sync_read_groups_に、指定されたデータを読むSyncReadGroupを追加する
  // できるだけ多くのデータをSyncReadで読み取るため、インダイレクトアドレスを活用する
  uint16_t start_address = ADDR_INDIRECT_ADDRESS_1;
  uint16_t total_length = 0;

  auto append = [this, group_name, &start_address, &total_length](
    auto target_addr, auto target_len, auto name, auto & indirect_address){
    if (!set_indirect_address(group_name, start_address, target_addr, target_len)) {
      std::cerr << name << "をindirect addressにセットできません." << std::endl;
      return false;
    }
    indirect_address = ADDR_INDIRECT_DATA_1 + total_length;
    total_length += target_len;
    start_address += LEN_INDIRECT_ADDRESS * target_len;
    return true;
  };

  if (joint_groups_[group_name]->sync_read_position_enabled()) {
    if (!append(ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION, "present_position",
        addr_sync_read_position_[group_name])) {
      return false;
    }
  }

  if (joint_groups_[group_name]->sync_read_velocity_enabled()) {
    if (!append(ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY, "present_velocity",
        addr_sync_read_velocity_[group_name])) {
      return false;
    }
  }

  if (joint_groups_[group_name]->sync_read_current_enabled()) {
    if (!append(ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT, "present_current",
        addr_sync_read_current_[group_name])) {
      return false;
    }
  }

  if (joint_groups_[group_name]->sync_read_voltage_enabled()) {
    if (!append(ADDR_PRESENT_VOLTAGE, LEN_PRESENT_VOLTAGE, "present_voltage",
        addr_sync_read_voltage_[group_name])) {
      return false;
    }
  }

  if (joint_groups_[group_name]->sync_read_temperature_enabled()) {
    if (!append(ADDR_PRESENT_TEMPERATURE, LEN_PRESENT_TEMPERATURE, "present_temperature",
        addr_sync_read_temperature_[group_name])) {
      return false;
    }
  }

  sync_read_groups_[group_name] = std::make_shared<dynamixel::GroupSyncRead>(
      port_handler_.get(), packet_handler_.get(), ADDR_INDIRECT_DATA_1, total_length);

  for (const auto & joint_name : joint_groups_[group_name]->joint_names()) {
    auto id = all_joints_.at(joint_name)->id();
    if (!sync_read_groups_[group_name]->addParam(id)) {
      std::cerr << group_name << ":" << joint_name << "のgroupSyncRead.addParam に失敗しました."
                << std::endl;
      return false;
    }
  }

  return true;
}

bool Hardware::create_sync_write_group(const std::string& group_name) {
  // sync_write_groups_に、指定されたデータを書き込むSyncWriteGroupを追加する
  // できるだけ多くのデータをSyncWriteで書き込むため、インダイレクトアドレスを活用する
  uint16_t start_address = ADDR_INDIRECT_ADDRESS_29;
  uint16_t total_length = 0;

  auto append = [this, group_name, &start_address, &total_length](
    auto target_addr, auto target_len, auto name){
    if (!set_indirect_address(group_name, start_address, target_addr, target_len)) {
      std::cerr << name << "をindirect addressにセットできません." << std::endl;
      return false;
    }
    total_length += target_len;
    start_address += LEN_INDIRECT_ADDRESS * target_len;
    return true;
  };

  if (joint_groups_[group_name]->sync_write_position_enabled()) {
    if (!append(ADDR_GOAL_POSITION, LEN_GOAL_POSITION, "goal_position")) {
      return false;
    }
  }

  if (joint_groups_[group_name]->sync_write_velocity_enabled()) {
    if (!append(ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY, "goal_velocity")) {
      return false;
    }
  }

  sync_write_groups_[group_name] = std::make_shared<dynamixel::GroupSyncWrite>(
      port_handler_.get(), packet_handler_.get(), ADDR_INDIRECT_DATA_29, total_length);

  std::vector<uint8_t> init_data(total_length, 0);

  for (const auto & joint_name : joint_groups_[group_name]->joint_names()) {
    auto id = all_joints_.at(joint_name)->id();
    if (!sync_write_groups_[group_name]->addParam(id, init_data.data())) {
      std::cerr << group_name << ":" << joint_name << "のgroupSyncWrite.addParam に失敗しました."
                << std::endl;
      return false;
    }
  }

  return true;
}

bool Hardware::set_indirect_address(const std::string& group_name,
                                    const uint16_t addr_indirect_start, const uint16_t addr_target,
                                    const uint16_t len_target) {
  // 指定されたグループのサーボモータのインダイレクトアドレスに、指定されたデータのアドレスを設定する

  bool retval = true;
  for (int i = 0; i < len_target; i++) {
    uint16_t target_indirect_address = addr_indirect_start + LEN_INDIRECT_ADDRESS * i;
    uint16_t target_data_address = addr_target + i;

    if (!write_word_data_to_group(group_name, target_indirect_address, target_data_address)) {
      std::cerr << "インダイレクトアドレス:" << std::to_string(target_indirect_address);
      std::cerr << "にターゲットデータアドレス:" << std::to_string(target_data_address);
      std::cerr << "を書き込めませんでした." << std::endl;
      return false;
    }
  }
  return true;
}

void Hardware::read_write_thread(const std::vector<std::string>& group_names,
                                 const std::chrono::milliseconds& update_cycle_ms) {
  // sync_read、sync_writeを繰り返すスレッド

  static auto current_time = std::chrono::steady_clock::now();
  auto next_start_time = current_time;
  while (thread_enable_) {
    next_start_time = current_time + update_cycle_ms;
    current_time = next_start_time;

    for (const auto & group_name : group_names) {
      sync_read(group_name);
      if (joint_groups_[group_name]->sync_write_velocity_enabled()) {
        limit_goal_velocity_by_present_position(group_name);
      }
      sync_write(group_name);
    }

    std::this_thread::sleep_until(next_start_time);
  }
}

bool Hardware::parse_dxl_error(const std::string& func_name, const uint8_t id,
                               const uint16_t address, const int dxl_comm_result,
                               const uint8_t dxl_packet_error) {
  bool retval = true;

  if (dxl_comm_result != COMM_SUCCESS) {
    std::cerr << "Function:" << func_name;
    std::cerr << ", ID:" << std::to_string(id);
    std::cerr << ", Address:" << std::to_string(address);
    std::cerr << ", CommError:" << std::string(packet_handler_->getTxRxResult(dxl_comm_result))
              << std::endl;
    retval = false;
  }

  if (dxl_packet_error != 0) {
    std::cerr << "Function:" << func_name;
    std::cerr << ", ID:" << std::to_string(id);
    std::cerr << ", Address:" << std::to_string(address);
    std::cerr << ", PacketError:"
              << std::string(packet_handler_->getRxPacketError(dxl_packet_error)) << std::endl;
    retval = false;
  }

  return retval;
}

bool Hardware::parse_dxl_error(const std::string& func_name, const int dxl_comm_result) {
  if (dxl_comm_result != COMM_SUCCESS) {
    std::cerr << "Function:" << func_name;
    std::cerr << ", CommError:" << std::string(packet_handler_->getTxRxResult(dxl_comm_result));
    std::cerr << std::endl;
    return false;
  }

  return true;
}

double Hardware::dxl_pos_to_radian(const int32_t position) {
  return (position - DXL_HOME_POSITION) * TO_RADIANS;
}

double Hardware::dxl_velocity_to_rps(const int32_t velocity) const {
  return velocity * TO_VELOCITY_RAD_PER_SEC;
}

double Hardware::dxl_current_to_ampere(const int16_t current) const {
  return current * TO_CURRENT_AMPERE;
}

double Hardware::dxl_voltage_to_volt(const int16_t voltage) const {
  return voltage * TO_VOLTAGE_VOLT;
}

uint32_t Hardware::radian_to_dxl_pos(const double position) {
  return position * TO_DXL_POS + DXL_HOME_POSITION;
}

uint32_t Hardware::to_dxl_velocity(const double velocity_rps) {
  return velocity_rps * DXL_VELOCITY_FROM_RAD_PER_SEC;
}

uint32_t Hardware::to_dxl_acceleration(const double acceleration_rpss) {
  // 加速度 rad/s^2をDYNAMIXELのデータに変換する

  int dxl_acceleration = DXL_ACCELERATION_FROM_RAD_PER_SS * acceleration_rpss;
  if (dxl_acceleration > DXL_MAX_ACCELERATION) {
    dxl_acceleration = DXL_MAX_ACCELERATION;
  } else if (dxl_acceleration <= 0) {
    // XMシリーズのDYNAMIXELでは、'0'は最大加速度を意味する
    // よって、加速度の最小値は'1'である
    dxl_acceleration = 1;
  }

  return static_cast<uint32_t>(dxl_acceleration);
}

uint32_t Hardware::to_dxl_profile_velocity(const double velocity_rps) {
  // 速度 rad/sをDYNAMIXELのProfile Velocityデータに変換する

  int dxl_velocity = DXL_VELOCITY_FROM_RAD_PER_SEC * velocity_rps;
  if (dxl_velocity > DXL_MAX_VELOCITY) {
    dxl_velocity = DXL_MAX_VELOCITY;
  } else if (dxl_velocity <= 0) {
    // XMシリーズのDYNAMIXELでは、'0'は最大速度を意味する
    // よって、速度の最小値は'1'である
    dxl_velocity = 1;
  }

  return static_cast<uint32_t>(dxl_velocity);
}

}  // namespace rt_manipulators_cpp
