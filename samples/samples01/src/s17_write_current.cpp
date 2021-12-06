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
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>
#include "rt_manipulators_cpp/hardware.hpp"

int main() {
  std::cout << "Sciurus17のサーボモータに目標電流を書き込むサンプルです." << std::endl;

  std::string port_name = "/dev/ttyUSB0";
  int baudrate = 3000000;  // 3Mbps
  std::string config_file = "../config/sciurus17_current.yaml";

  rt_manipulators_cpp::Hardware hardware(port_name);
  if (!hardware.connect(baudrate)) {
    std::cerr << "ロボットとの接続に失敗しました." << std::endl;
    return -1;
  }

  if (!hardware.load_config_file(config_file)) {
    std::cerr << "コンフィグファイルの読み込みに失敗しました." << std::endl;
    return -1;
  }

  std::vector<std::string> group_names = {"right_wrist", "left_wrist"};

  for (const auto & group_name : group_names) {
    if (!hardware.torque_on(group_name)) {
      std::cerr << group_name << "グループのトルクをONできませんでした." << std::endl;
      return -1;
    }
  }

  std::cout << "read/writeスレッドを起動します." << std::endl;
  if (!hardware.start_thread(group_names, std::chrono::milliseconds(10))) {
    std::cerr << "スレッドの起動に失敗しました." << std::endl;
    return -1;
  }

  std::cout << "5秒後に手先が動き出すため、手先の周りに物や人を近づけないで下さい." << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(5));

  // 目標速度を段階的に早くする
  const double MAX_CURRENT = 0.05;  // Ampere
  const double STEPS = 10;
  for (int i=1; i <=STEPS; i++) {
    double goal_current = MAX_CURRENT * i / static_cast<double>(STEPS);
    std::cout << "set current:" << goal_current << " A" << std::endl;
    hardware.set_current(7, goal_current);
    hardware.set_current(8, goal_current);
    hardware.set_current(15, goal_current);
    hardware.set_current(16, goal_current);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "set current:" << -goal_current << " A" << std::endl;
    hardware.set_current("right_arm_joint6", -goal_current);
    hardware.set_current("right_arm_joint7", -goal_current);
    hardware.set_current("left_arm_joint6", -goal_current);
    hardware.set_current("left_arm_joint7", -goal_current);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  std::cout << "set current: 0.0 A" << std::endl;
  std::vector<double> goal_currents = {0.0, 0.0};

  for (const auto & group_name : group_names) {
    hardware.set_currents(group_name, goal_currents);
  }
  std::this_thread::sleep_for(std::chrono::seconds(1));

  std::cout << "スレッドを停止します." << std::endl;
  hardware.stop_thread();


  for (const auto & group_name : group_names) {
    if (!hardware.torque_off(group_name)) {
      std::cerr << group_name << "グループのトルクをOFFできませんでした." << std::endl;
    }
  }

  std::cout << "Sciurus17との接続を解除します." << std::endl;
  hardware.disconnect();
  return 0;
}
