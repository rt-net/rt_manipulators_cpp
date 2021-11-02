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

#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "rt_manipulators_cpp/hardware.hpp"

void print_names(void) {
  std::cout << "position[rad], ";
  std::cout << "velocity[rad/s], ";
  std::cout << "current[A], ";
  std::cout << "voltage[V], ";
  std::cout << "temperature[deg]";
  std::cout << std::endl;
}

void print_values(const double position, const double velocity, const double current,
                  const double voltage, const int8_t temperature) {
  std::cout << std::to_string(position) << ", ";
  std::cout << std::to_string(velocity) << ", ";
  std::cout << std::to_string(current) << ", ";
  std::cout << std::to_string(voltage) << ", ";
  std::cout << std::to_string(temperature);
  std::cout << std::endl;
}

int main() {
  std::cout << "CRANE-X7のサーボモータ";
  std::cout << "位置、速度、電流、入力電圧、温度を読み取るサンプルです." << std::endl;

  std::string port_name = "/dev/ttyUSB0";
  int baudrate = 3000000;  // 3Mbps
  std::string config_file = "../config/crane-x7_read.yaml";

  rt_manipulators_cpp::Hardware hardware(port_name);
  if (!hardware.connect(baudrate)) {
    std::cerr << "ロボットとの接続に失敗しました." << std::endl;
    return -1;
  }

  if (!hardware.load_config_file(config_file)) {
    std::cerr << "コンフィグファイルの読み込みに失敗しました." << std::endl;
    return -1;
  }

  std::cout << "サーボモータの電流値を観察しやすくするため、" << std::endl;
  std::cout << "5秒後にhandグループのトルクがONします." << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(5));

  if (!hardware.torque_on("hand")) {
    std::cerr << "handグループのトルクをONできませんでした." << std::endl;
    return -1;
  }

  std::vector<std::string> groups = {"arm", "hand"};
  for (int i = 0; i < 1000; i++) {
    for (auto group_name : groups) {
      if (!hardware.sync_read(group_name)) {
        std::cerr << group_name << "グループのsync readに失敗しました." << std::endl;
        break;
      }

      std::vector<double> positions;
      std::vector<double> velocities;
      std::vector<double> currents;
      std::vector<double> voltages;
      std::vector<int8_t> temperatures;
      if (hardware.get_positions(group_name, positions) &&
          hardware.get_velocities(group_name, velocities) &&
          hardware.get_currents(group_name, currents) &&
          hardware.get_voltages(group_name, voltages) &&
          hardware.get_temperatures(group_name, temperatures)) {
        std::cout << group_name << ": index, ";
        print_names();
        for (int i = 0; i < positions.size(); i++) {
          std::cout << std::to_string(i) << ", ";
          print_values(positions[i], velocities[i], currents[i], voltages[i], temperatures[i]);
        }
      }
    }

    const int dxl_id = 9;
    const std::string joint_name = "joint_hand";
    double position;
    double velocity;
    double current;
    double voltage;
    int8_t temperature;
    if (hardware.get_position(dxl_id, position) &&
        hardware.get_velocity(dxl_id, velocity) &&
        hardware.get_current(dxl_id, current) &&
        hardware.get_voltage(dxl_id, voltage) &&
        hardware.get_temperature(dxl_id, temperature)) {
      std::cout << "ID:" << std::to_string(dxl_id) << ": ";
      print_names();
      print_values(position, velocity, current, voltage, temperature);
    }

    if (hardware.get_position(joint_name, position) &&
        hardware.get_velocity(joint_name, velocity) &&
        hardware.get_current(joint_name, current) &&
        hardware.get_voltage(joint_name, voltage) &&
        hardware.get_temperature(joint_name, temperature)) {
      std::cout << joint_name << ": ";
      print_names();
      print_values(position, velocity, current, voltage, temperature);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  if (!hardware.torque_off("hand")) {
    std::cerr << "handグループのトルクをOFFできませんでした." << std::endl;
  }

  std::cout << "CRANE-X7との接続を解除します." << std::endl;
  hardware.disconnect();
  return 0;
}
