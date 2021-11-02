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
        std::cout << group_name;
        std::cout << " : index, position[rad], velocity[rad/s], current[A]";
        std::cout << ", voltage[V], temperature[deg]";
        std::cout << std::endl;
        for (int i = 0; i < positions.size(); i++) {
          std::cout << std::to_string(i) << ", ";
          std::cout << std::to_string(positions[i]) << ", ";
          std::cout << std::to_string(velocities[i]) << ", ";
          std::cout << std::to_string(currents[i]) << ", ";
          std::cout << std::to_string(voltages[i]) << ", ";
          std::cout << std::to_string(temperatures[i]) << ", ";
          std::cout << std::endl;
        }
      }
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
