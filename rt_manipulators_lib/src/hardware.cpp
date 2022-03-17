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

#include <algorithm>
#include <cmath>
#include <iostream>

#include "hardware.hpp"
#include "config_file_parser.hpp"

namespace rt_manipulators_cpp {

// Ref: https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/
// Ref: https://emanual.robotis.com/docs/en/dxl/x/xm540-w270/
// Ref: https://emanual.robotis.com/docs/en/dxl/x/xm540-w150/
const int DXL_HOME_POSITION = 2048;
const double TO_RADIANS = (180.0 / 2048.0) * M_PI / 180.0;
const double TO_DXL_POS = 1.0 / TO_RADIANS;
const double TO_VELOCITY_REV_PER_MIN = 0.229;
const double TO_VELOCITY_RAD_PER_MIN = TO_VELOCITY_REV_PER_MIN * 2.0 * M_PI;
const double TO_VELOCITY_RAD_PER_SEC = TO_VELOCITY_RAD_PER_MIN / 60.0;
const double DXL_VELOCITY_FROM_RAD_PER_SEC = 1.0 / TO_VELOCITY_RAD_PER_SEC;
const int DXL_MAX_VELOCITY = 32767;
const double TO_CURRENT_AMPERE = 0.00269;
const double TO_DXL_CURRENT = 1.0 / TO_CURRENT_AMPERE;
const double TO_VOLTAGE_VOLT = 0.1;

// Dynamixel XM Series address table
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

Hardware::Hardware(const std::string device_name) :
  thread_enable_(false) {
  comm_ = std::make_shared<hardware_communicator::Communicator>(device_name);
}

Hardware::~Hardware() {
  stop_thread();
  disconnect();
}

bool Hardware::load_config_file(const std::string& config_yaml) {
  // コンフィグファイルの読み込み
  if (config_file_parser::parse(config_yaml, joints_) == false) {
    return false;
  }

  for (const auto& [group_name, group] : joints_.groups()) {
    // ジョイントリミットの読み込みと設定
    for (const auto& joint_name : group->joint_names()) {
      double max_position_limit = 0.0;
      double min_position_limit = 0.0;
      double current_limit = 0.0;
      if (!joints_.joint(joint_name)->dxl->read_max_position_limit(comm_, max_position_limit)) {
        std::cerr << joint_name << "のMax Position Limitの読み取りに失敗しました." << std::endl;
        return false;
      }
      if (!joints_.joint(joint_name)->dxl->read_min_position_limit(comm_, min_position_limit)) {
        std::cerr << joint_name << "のMin Position Limitの読み取りに失敗しました." << std::endl;
        return false;
      }
      if (!joints_.joint(joint_name)->dxl->read_current_limit(comm_, current_limit)) {
        std::cerr << joint_name << "のCurrent Limitの読み取りに失敗しました." << std::endl;
        return false;
      }

      joints_.joint(joint_name)->set_position_limit(min_position_limit, max_position_limit);
      joints_.joint(joint_name)->set_current_limit(current_limit);
    }

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

  std::cout << "Config file '" << config_yaml << "' loaded." << std::endl;
  for (const auto& [group_name, group] : joints_.groups()) {
    std::cout << group_name << std::endl;
    for (const auto & joint_name : group->joint_names()) {
      std::cout << "\t" << joint_name;
      std::cout << ", id:" << std::to_string(joints_.joint(joint_name)->id());
      std::cout << ", mode:" << std::to_string(joints_.joint(joint_name)->operating_mode());
      std::cout << ", modified max_position_limit:" << std::to_string(
        joints_.joint(joint_name)->max_position_limit());
      std::cout << ", modified min_position_limit:" << std::to_string(
        joints_.joint(joint_name)->min_position_limit());
      std::cout << ", current_limit_when_position_exceeds_limit:" << std::to_string(
        joints_.joint(joint_name)->current_limit_when_position_exceeds_limit());
      std::cout << std::endl;
    }
  }

  return true;
}

bool Hardware::connect(const int baudrate) {
  if (!comm_->connect()) {
    return false;
  }

  return true;
}

void Hardware::disconnect() {
  if (!comm_->is_connected()) {
    return;
  }

  for (const auto& [group_name, group] : joints_.groups()) {
    // 速度指示モードの場合は、goal_velocityを0にする
    if (group->sync_write_velocity_enabled()) {
      std::cout << group_name << "グループにはvelocityのsync_writeが設定されています." << std::endl;
      std::cout << "安全のため, disconnect()関数内で目標速度 0 rad/sを書き込みます." << std::endl;
      for (const auto & joint_name : group->joint_names()) {
          joints_.joint(joint_name)->set_goal_velocity(0);
      }
      sync_write(group_name);
    }
    // 電流指示モードの場合は、goal_currentを0にする
    if (group->sync_write_current_enabled()) {
      std::cout << group_name << "グループにはcurrentのsync_writeが設定されています." << std::endl;
      std::cout << "安全のため, disconnect()関数内で目標速度 0 Aを書き込みます." << std::endl;
      for (const auto & joint_name : group->joint_names()) {
          joints_.joint(joint_name)->set_goal_current(0);
      }
      sync_write(group_name);
    }
  }

  comm_->disconnect();
}

bool Hardware::torque_on(const std::string& group_name) {
  for (const auto & joint_name : joints_.group(group_name)->joint_names()) {
    if (!joints_.joint(joint_name)->dxl->write_torque_enable(comm_, true)) {
      std::cerr << joint_name << "ジョイントのトルクONに失敗しました." << std::endl;
      return false;
    }
  }

  // 安全のため、サーボの現在角度を目標角度に設定する
  if (!sync_read(group_name)) {
    std::cerr << group_name << "グループのトルクON後のsync_readに失敗しました." << std::endl;
    return false;
  }
  for (const auto & joint_name : joints_.group(group_name)->joint_names()) {
    joints_.joint(joint_name)
        ->set_goal_position(joints_.joint(joint_name)->get_present_position());
  }
  return true;
}

bool Hardware::torque_off(const std::string& group_name) {
  bool retval = true;
  for (const auto & joint_name : joints_.group(group_name)->joint_names()) {
    if (!joints_.joint(joint_name)->dxl->write_torque_enable(comm_, false)) {
      std::cerr << joint_name << "ジョイントのトルクOFFに失敗しました." << std::endl;
      retval = false;
    }
  }
  return retval;
}

bool Hardware::sync_read(const std::string& group_name) {
  // 指定されたグループのsyncReadを実行して、サーボモータからデータを読み取る
  // 読み取ったデータはメンバ変数に格納する
  if (!joints_.has_group(group_name)) {
    std::cerr << group_name << "はjoint_groupsに存在しません." << std::endl;
    return false;
  }

  if (!comm_->send_sync_read_packet(group_name)) {
    return false;
  }

  auto get_data = [this](auto group_name, auto joint_name, auto addr, auto len, auto & data){
    auto id = joints_.joint(joint_name)->id();
    return comm_->get_sync_read_data(group_name, id, addr, len, data);
  };

  bool retval = true;
  if (joints_.group(group_name)->sync_read_position_enabled()) {
    for (const auto & joint_name : joints_.group(group_name)->joint_names()) {
      uint32_t data = 0;
      if (get_data(group_name, joint_name, addr_sync_read_position_[group_name],
                  LEN_PRESENT_POSITION, data)) {
        joints_.joint(joint_name)->set_present_position(
          joints_.joint(joint_name)->dxl->to_position_radian(static_cast<int>(data)));
      } else {
        std::cerr << joint_name << "のpresent_positionを取得できません." << std::endl;
        retval = false;
      }
    }
  }

  if (joints_.group(group_name)->sync_read_velocity_enabled()) {
    for (const auto & joint_name : joints_.group(group_name)->joint_names()) {
      uint32_t data = 0;
      if (get_data(group_name, joint_name, addr_sync_read_velocity_[group_name],
                  LEN_PRESENT_VELOCITY, data)) {
        joints_.joint(joint_name)->set_present_velocity(
          dxl_velocity_to_rps(static_cast<int32_t>(data)));
      } else {
        std::cerr << joint_name << "のpresent_velocityを取得できません." << std::endl;
        retval = false;
      }
    }
  }

  if (joints_.group(group_name)->sync_read_current_enabled()) {
    for (const auto & joint_name : joints_.group(group_name)->joint_names()) {
      uint32_t data = 0;
      if (get_data(group_name, joint_name, addr_sync_read_current_[group_name],
                  LEN_PRESENT_CURRENT, data)) {
        joints_.joint(joint_name)->set_present_current(
          joints_.joint(joint_name)->dxl->to_current_ampere(static_cast<int>(data)));
      } else {
        std::cerr << joint_name << "のpresent_currentを取得できません." << std::endl;
        retval = false;
      }
    }
  }

  if (joints_.group(group_name)->sync_read_voltage_enabled()) {
    for (const auto & joint_name : joints_.group(group_name)->joint_names()) {
      uint32_t data = 0;
      if (get_data(group_name, joint_name, addr_sync_read_voltage_[group_name],
                  LEN_PRESENT_VOLTAGE, data)) {
        joints_.joint(joint_name)->set_present_voltage(
          dxl_voltage_to_volt(static_cast<int16_t>(data)));
      } else {
        std::cerr << joint_name << "のpresent_voltageを取得できません." << std::endl;
        retval = false;
      }
    }
  }

  if (joints_.group(group_name)->sync_read_temperature_enabled()) {
    for (const auto & joint_name : joints_.group(group_name)->joint_names()) {
      uint32_t data = 0;
      if (get_data(group_name, joint_name, addr_sync_read_temperature_[group_name],
                  LEN_PRESENT_TEMPERATURE, data)) {
        joints_.joint(joint_name)->set_present_temperature(static_cast<int8_t>(data));
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
  if (!joints_.has_group(group_name)) {
    std::cerr << group_name << "はjoint_groupsに存在しません." << std::endl;
    return false;
  }

  for (const auto & joint_name : joints_.group(group_name)->joint_names()) {
    std::vector<uint8_t> write_data;
    if (joints_.group(group_name)->sync_write_position_enabled()) {
      uint32_t goal_position = radian_to_dxl_pos(joints_.joint(joint_name)->get_goal_position());
      write_data.push_back(DXL_LOBYTE(DXL_LOWORD(goal_position)));
      write_data.push_back(DXL_HIBYTE(DXL_LOWORD(goal_position)));
      write_data.push_back(DXL_LOBYTE(DXL_HIWORD(goal_position)));
      write_data.push_back(DXL_HIBYTE(DXL_HIWORD(goal_position)));
    }

    if (joints_.group(group_name)->sync_write_velocity_enabled()) {
      uint32_t goal_velocity = to_dxl_velocity(joints_.joint(joint_name)->get_goal_velocity());
      write_data.push_back(DXL_LOBYTE(DXL_LOWORD(goal_velocity)));
      write_data.push_back(DXL_HIBYTE(DXL_LOWORD(goal_velocity)));
      write_data.push_back(DXL_LOBYTE(DXL_HIWORD(goal_velocity)));
      write_data.push_back(DXL_HIBYTE(DXL_HIWORD(goal_velocity)));
    }

    if (joints_.group(group_name)->sync_write_current_enabled()) {
      uint16_t goal_current = to_dxl_current(joints_.joint(joint_name)->get_goal_current());
      write_data.push_back(DXL_LOBYTE(goal_current));
      write_data.push_back(DXL_HIBYTE(goal_current));
    }

    auto id = joints_.joint(joint_name)->id();
    if (!comm_->set_sync_write_data(group_name, id, write_data)) {
      return false;
    }
  }

  if (!comm_->send_sync_write_packet(group_name)) {
    return false;
  }

  return true;
}

bool Hardware::start_thread(const std::vector<std::string>& group_names,
                            const std::chrono::milliseconds& update_cycle_ms) {
  // read, writeを繰り返すスレッドを開始する
  for (const auto & group_name : group_names) {
    if (!joints_.has_group(group_name)) {
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

  for (const auto& [group_name, group] : joints_.groups()) {
    // 速度指示モードの場合は、goal_velocityを0にする
    if (group->sync_write_velocity_enabled()) {
      std::cout << group_name << "グループにはvelocityのsync_writeが設定されています." << std::endl;
      std::cout << "安全のため, stop_thread()関数内で目標速度 0 rad/sを書き込みます." << std::endl;
      for (const auto & joint_name : group->joint_names()) {
          joints_.joint(joint_name)->set_goal_velocity(0);
      }
      sync_write(group_name);
    }
    // 電流指示モードの場合は、goal_currentを0にする
    if (group->sync_write_current_enabled()) {
      std::cout << group_name << "グループにはcurrentのsync_writeが設定されています." << std::endl;
      std::cout << "安全のため, stop_thread()関数内で目標電流 0 Aを書き込みます." << std::endl;
      for (const auto & joint_name : group->joint_names()) {
          joints_.joint(joint_name)->set_goal_current(0);
      }
      sync_write(group_name);
    }
  }

  return true;
}

bool Hardware::get_position(const uint8_t id, double& position) {
  return joints_.get_position(id, position);
}

bool Hardware::get_position(const std::string& joint_name, double& position) {
  return joints_.get_position(joint_name, position);
}

bool Hardware::get_positions(const std::string& group_name, std::vector<double>& positions) {
  return joints_.get_positions(group_name, positions);
}

bool Hardware::get_velocity(const uint8_t id, double& velocity) {
  return joints_.get_velocity(id, velocity);
}

bool Hardware::get_velocity(const std::string& joint_name, double& velocity) {
  return joints_.get_velocity(joint_name, velocity);
}

bool Hardware::get_velocities(const std::string& group_name, std::vector<double>& velocities) {
  return joints_.get_velocities(group_name, velocities);
}

bool Hardware::get_current(const uint8_t id, double& current) {
  return joints_.get_current(id, current);
}

bool Hardware::get_current(const std::string& joint_name, double& current) {
  return joints_.get_current(joint_name, current);
}

bool Hardware::get_currents(const std::string& group_name, std::vector<double>& currents) {
  return joints_.get_currents(group_name, currents);
}

bool Hardware::get_voltage(const uint8_t id, double& voltage) {
  return joints_.get_voltage(id, voltage);
}

bool Hardware::get_voltage(const std::string& joint_name, double& voltage) {
  return joints_.get_voltage(joint_name, voltage);
}

bool Hardware::get_voltages(const std::string& group_name, std::vector<double>& voltages) {
  return joints_.get_voltages(group_name, voltages);
}

bool Hardware::get_temperature(const uint8_t id, int8_t& temperature) {
  return joints_.get_temperature(id, temperature);
}

bool Hardware::get_temperature(const std::string& joint_name, int8_t& temperature) {
  return joints_.get_temperature(joint_name, temperature);
}

bool Hardware::get_temperatures(const std::string& group_name, std::vector<int8_t>& temperatures) {
  return joints_.get_temperatures(group_name, temperatures);
}

bool Hardware::get_max_position_limit(const uint8_t & id, double & max_position_limit) {
  return joints_.get_max_position_limit(id, max_position_limit);
}

bool Hardware::get_min_position_limit(const uint8_t & id, double & min_position_limit) {
  return joints_.get_min_position_limit(id, min_position_limit);
}

bool Hardware::set_position(const uint8_t id, const double position) {
  return joints_.set_position(id, position);
}

bool Hardware::set_position(const std::string& joint_name, const double position) {
  return joints_.set_position(joint_name, position);
}

bool Hardware::set_positions(const std::string& group_name, std::vector<double>& positions) {
  return joints_.set_positions(group_name, positions);
}

bool Hardware::set_velocity(const uint8_t id, const double velocity) {
  if (!thread_enable_) {
    std::cerr << "目標速度を書き込む場合は、";
    std::cerr << "安全のためstart_thread()を実行してください." << std::endl;
    return false;
  }

  return joints_.set_velocity(id, velocity);
}

bool Hardware::set_velocity(const std::string& joint_name, const double velocity) {
  if (!thread_enable_) {
    std::cerr << "目標速度を書き込む場合は、";
    std::cerr << "安全のためstart_thread()を実行してください." << std::endl;
    return false;
  }

  return joints_.set_velocity(joint_name, velocity);
}

bool Hardware::set_velocities(const std::string& group_name, std::vector<double>& velocities) {
  if (!thread_enable_) {
    std::cerr << "目標速度を書き込む場合は、";
    std::cerr << "安全のためstart_thread()を実行してください." << std::endl;
    return false;
  }

  return joints_.set_velocities(group_name, velocities);
}

bool Hardware::set_current(const uint8_t id, const double current) {
  if (!thread_enable_) {
    std::cerr << "目標電流を書き込む場合は、";
    std::cerr << "安全のためstart_thread()を実行してください." << std::endl;
    return false;
  }

  return joints_.set_current(id, current);
}

bool Hardware::set_current(const std::string& joint_name, const double current) {
  if (!thread_enable_) {
    std::cerr << "目標電流を書き込む場合は、";
    std::cerr << "安全のためstart_thread()を実行してください." << std::endl;
    return false;
  }

  return joints_.set_current(joint_name, current);
}

bool Hardware::set_currents(const std::string& group_name, std::vector<double>& currents) {
  if (!thread_enable_) {
    std::cerr << "目標電流を書き込む場合は、";
    std::cerr << "安全のためstart_thread()を実行してください." << std::endl;
    return false;
  }

  return joints_.set_currents(group_name, currents);
}

bool Hardware::write_max_acceleration_to_group(const std::string& group_name,
                                               const double acceleration_rpss) {
  // 指定されたグループ内のサーボモータの最大動作加速度（radian / s^2）を設定する
  if (!joints_.has_group(group_name)) {
    std::cerr << group_name << "はjoint_groupsに存在しません." << std::endl;
    return false;
  }

  bool retval = true;
  for (const auto & joint_name : joints_.group(group_name)->joint_names()) {
    if (!joints_.joint(joint_name)->dxl->write_profile_acceleration(comm_, acceleration_rpss)) {
      std::cerr << joint_name << std::endl;
      std::cerr << "ジョイントのProfile Accelerationの書き込みに失敗しました." << std::endl;
      retval = false;
    }
  }
  return retval;
}

bool Hardware::write_max_velocity_to_group(const std::string& group_name,
                                           const double velocity_rps) {
  // 指定されたグループ内のサーボモータの最大動作加速度（radian / s^2）を設定する
  if (!joints_.has_group(group_name)) {
    std::cerr << group_name << "はjoint_groupsに存在しません." << std::endl;
    return false;
  }

  bool retval = true;
  for (const auto & joint_name : joints_.group(group_name)->joint_names()) {
    if (!joints_.joint(joint_name)->dxl->write_profile_velocity(comm_, velocity_rps)) {
      std::cerr << joint_name << std::endl;
      std::cerr << "ジョイントのProfile Velocityの書き込みに失敗しました." << std::endl;
      retval = false;
    }
  }
  return retval;
}

bool Hardware::write_position_pid_gain(const uint8_t id, const uint16_t p, const uint16_t i,
                               const uint16_t d) {
  // 指定されたサーボモータの位置制御PIDゲインを設定する
  if (!joints_.has_joint(id)) {
    std::cerr << "ID:" << std::to_string(id) << "のジョイントは存在しません." << std::endl;
    return false;
  }

  if (!joints_.joint(id)->dxl->write_position_p_gain(comm_, p)) {
    std::cerr << "ID:" << std::to_string(id);
    std::cerr << "のPosition P Gainの書き込みに失敗しました." << std::endl;
    return false;
  }

  if (!joints_.joint(id)->dxl->write_position_i_gain(comm_, i)) {
    std::cerr << "ID:" << std::to_string(id);
    std::cerr << "のPosition I Gainの書き込みに失敗しました." << std::endl;
    return false;
  }

  if (!joints_.joint(id)->dxl->write_position_d_gain(comm_, d)) {
    std::cerr << "ID:" << std::to_string(id);
    std::cerr << "のPosition D Gainの書き込みに失敗しました." << std::endl;
    return false;
  }

  return true;
}

bool Hardware::write_position_pid_gain(const std::string& joint_name, const uint16_t p,
                                       const uint16_t i, const uint16_t d) {
  // 指定されたサーボモータの位置制御PIDゲインを設定する
  if (!joints_.has_joint(joint_name)) {
    std::cerr << joint_name << "ジョイントは存在しません." << std::endl;
    return false;
  }

  return write_position_pid_gain(joints_.joint(joint_name)->id(), p, i, d);
}

bool Hardware::write_position_pid_gain_to_group(const std::string& group_name, const uint16_t p,
                                                const uint16_t i, const uint16_t d) {
  // 指定されたグループ内のサーボモータの位置制御PIDゲインを設定する
  if (!joints_.has_group(group_name)) {
    std::cerr << group_name << "はjoint_groupsに存在しません." << std::endl;
    return false;
  }

  bool retval = true;
  for (const auto & joint_name : joints_.group(group_name)->joint_names()) {
    if (!write_position_pid_gain(joint_name, p, i, d)) {
      retval = false;
    }
  }
  return retval;
}

bool Hardware::write_velocity_pi_gain(const uint8_t id, const uint16_t p, const uint16_t i) {
  // 指定されたサーボモータの速度制御PIゲインを設定する
  if (!joints_.has_joint(id)) {
    std::cerr << "ID:" << std::to_string(id) << "のジョイントは存在しません." << std::endl;
    return false;
  }

  if (!joints_.joint(id)->dxl->write_velocity_p_gain(comm_, p)) {
    std::cerr << "ID:" << std::to_string(id);
    std::cerr << "のVelocity P Gainの書き込みに失敗しました." << std::endl;
    return false;
  }

  if (!joints_.joint(id)->dxl->write_velocity_i_gain(comm_, i)) {
    std::cerr << "ID:" << std::to_string(id);
    std::cerr << "のVelocity I Gainの書き込みに失敗しました." << std::endl;
    return false;
  }

  return true;
}

bool Hardware::write_velocity_pi_gain(const std::string& joint_name, const uint16_t p,
                                      const uint16_t i) {
  // 指定されたサーボモータの速度制御PIゲインを設定する
  if (!joints_.has_joint(joint_name)) {
    std::cerr << joint_name << "ジョイントは存在しません." << std::endl;
    return false;
  }

  return write_velocity_pi_gain(joints_.joint(joint_name)->id(), p, i);
}

bool Hardware::write_velocity_pi_gain_to_group(const std::string& group_name, const uint16_t p,
                                       const uint16_t i) {
  // 指定されたグループ内のサーボモータの速度制御PIゲインを設定する
  if (!joints_.has_group(group_name)) {
    std::cerr << group_name << "はjoint_groupsに存在しません." << std::endl;
    return false;
  }

  bool retval = true;
  for (const auto & joint_name : joints_.group(group_name)->joint_names()) {
    if (!write_velocity_pi_gain(joint_name, p, i)) {
      retval = false;
    }
  }
  return retval;
}

bool Hardware::write_word_data_to_group(const std::string& group_name, const uint16_t address,
                                        const uint16_t write_data) {
  if (!joints_.has_group(group_name)) {
    std::cerr << group_name << "はjoint_groupsに存在しません." << std::endl;
    return false;
  }

  bool retval = true;
  for (const auto & joint_name : joints_.group(group_name)->joint_names()) {
    auto id = joints_.joint(joint_name)->id();
    if (!comm_->write_word_data(id, address, write_data)) {
      retval = false;
    }
  }
  return retval;
}

bool Hardware::write_operating_mode(const std::string& group_name) {
  // サーボモータから動作モードを読み取り、
  // コンフィグファイルと一致しない場合、動作モードを書き込む
  // 動作モードはROM領域にあるため、データ書き込み回数を抑えたい
  const std::vector<uint8_t> SUPPORT_MODE_LIST = {0, 1, 3, 5};

  for (const auto & joint_name : joints_.group(group_name)->joint_names()) {
    auto target_ope_mode = joints_.joint(joint_name)->operating_mode();
    auto result = std::find(SUPPORT_MODE_LIST.begin(), SUPPORT_MODE_LIST.end(), target_ope_mode);
    if (result == SUPPORT_MODE_LIST.end()) {
      std::cout << joint_name << "にライブラリがサポートしないOperating Mode:";
      std::cout << std::to_string(target_ope_mode) << "が設定されています." << std::endl;
      return false;
    }

    uint8_t present_ope_mode;
    if (!joints_.joint(joint_name)->dxl->read_operating_mode(comm_, present_ope_mode)) {
      std::cout << joint_name << "ジョイントのOperating Modeを読み取れません." << std::endl;
      return false;
    }

    if (target_ope_mode != present_ope_mode) {
      if (!joints_.joint(joint_name)->dxl->write_operating_mode(comm_, target_ope_mode)) {
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
  for (const auto & joint_name : joints_.group(group_name)->joint_names()) {
    auto max_position_limit = joints_.joint(joint_name)->max_position_limit();
    auto min_position_limit = joints_.joint(joint_name)->min_position_limit();
    auto present_position = joints_.joint(joint_name)->get_present_position();
    auto goal_velocity = joints_.joint(joint_name)->get_goal_velocity();
    bool has_exceeded_max_pos_limit = present_position > max_position_limit && goal_velocity > 0;
    bool has_exceeded_min_pos_limit = present_position < min_position_limit && goal_velocity < 0;

    if (has_exceeded_max_pos_limit || has_exceeded_min_pos_limit) {
      std::cout << joint_name << "ジョイントの現在角度が限界角度に到達しました、";
      std::cout << "goal_velocityを0で上書きします." << std::endl;
      joints_.joint(joint_name)->set_goal_velocity(0);
      retval = false;
    }
  }

  return retval;
}

bool Hardware::limit_goal_current_by_present_position(const std::string& group_name) {
  // ジョイントの現在角度がmax/min position limit を超えた場合、
  // goal_currentをcurrent limitに制限する
  bool retval = true;
  for (const auto & joint_name : joints_.group(group_name)->joint_names()) {
    auto max_position_limit = joints_.joint(joint_name)->max_position_limit();
    auto min_position_limit = joints_.joint(joint_name)->min_position_limit();
    auto current_limit = joints_.joint(joint_name)->current_limit_when_position_exceeds_limit();
    auto present_position = joints_.joint(joint_name)->get_present_position();
    auto goal_current = joints_.joint(joint_name)->get_goal_current();
    bool has_exceeded_max_pos_limit = present_position > max_position_limit &&
      goal_current > current_limit;
    bool has_exceeded_min_pos_limit = present_position < min_position_limit &&
      goal_current < -current_limit;

    if (has_exceeded_max_pos_limit || has_exceeded_min_pos_limit) {
      std::cout << joint_name << "ジョイントの現在角度が限界角度に到達しました、";
      std::cout << "goal_currentを" << current_limit << " Aに制限します." << std::endl;
      auto limited_current = std::clamp(goal_current, -current_limit, current_limit);
      joints_.joint(joint_name)->set_goal_current(limited_current);
      retval = false;
    }
  }

  return retval;
}

bool Hardware::create_sync_read_group(const std::string& group_name) {
  // HardwareCommunicatorに、指定されたデータを読むSyncReadGroupを追加する
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

  if (joints_.group(group_name)->sync_read_position_enabled()) {
    if (!append(ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION, "present_position",
        addr_sync_read_position_[group_name])) {
      return false;
    }
  }

  if (joints_.group(group_name)->sync_read_velocity_enabled()) {
    if (!append(ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY, "present_velocity",
        addr_sync_read_velocity_[group_name])) {
      return false;
    }
  }

  if (joints_.group(group_name)->sync_read_current_enabled()) {
    if (!append(ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT, "present_current",
        addr_sync_read_current_[group_name])) {
      return false;
    }
  }

  if (joints_.group(group_name)->sync_read_voltage_enabled()) {
    if (!append(ADDR_PRESENT_VOLTAGE, LEN_PRESENT_VOLTAGE, "present_voltage",
        addr_sync_read_voltage_[group_name])) {
      return false;
    }
  }

  if (joints_.group(group_name)->sync_read_temperature_enabled()) {
    if (!append(ADDR_PRESENT_TEMPERATURE, LEN_PRESENT_TEMPERATURE, "present_temperature",
        addr_sync_read_temperature_[group_name])) {
      return false;
    }
  }

  comm_->make_sync_read_group(group_name, ADDR_INDIRECT_DATA_1, total_length);

  for (const auto & joint_name : joints_.group(group_name)->joint_names()) {
    auto id = joints_.joint(joint_name)->id();
    if (!comm_->append_id_to_sync_read_group(group_name, id)) {
      return false;
    }
  }

  return true;
}

bool Hardware::create_sync_write_group(const std::string& group_name) {
  // HardwareCommunicatorに、指定されたデータを書き込むSyncWriteGroupを追加する
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

  if (joints_.group(group_name)->sync_write_position_enabled()) {
    if (!append(ADDR_GOAL_POSITION, LEN_GOAL_POSITION, "goal_position")) {
      return false;
    }
  }

  if (joints_.group(group_name)->sync_write_velocity_enabled()) {
    if (!append(ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY, "goal_velocity")) {
      return false;
    }
  }

  if (joints_.group(group_name)->sync_write_current_enabled()) {
    if (!append(ADDR_GOAL_CURRENT, LEN_GOAL_CURRENT, "goal_current")) {
      return false;
    }
  }

  comm_->make_sync_write_group(group_name, ADDR_INDIRECT_DATA_29, total_length);

  std::vector<uint8_t> init_data(total_length, 0);
  for (const auto & joint_name : joints_.group(group_name)->joint_names()) {
    auto id = joints_.joint(joint_name)->id();
    if (!comm_->append_id_to_sync_write_group(group_name, id, init_data)) {
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
      if (joints_.group(group_name)->sync_write_velocity_enabled()) {
        limit_goal_velocity_by_present_position(group_name);
      }
      if (joints_.group(group_name)->sync_write_current_enabled()) {
        limit_goal_current_by_present_position(group_name);
      }
      sync_write(group_name);
    }

    std::this_thread::sleep_until(next_start_time);
  }
}

double Hardware::dxl_velocity_to_rps(const int32_t velocity) const {
  return velocity * TO_VELOCITY_RAD_PER_SEC;
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

uint16_t Hardware::to_dxl_current(const double current_ampere) {
  return current_ampere * TO_DXL_CURRENT;
}

}  // namespace rt_manipulators_cpp
