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
  std::cout << "CRANE-X7のサーボモータ目標速度を書き込むサンプルです." << std::endl;

  std::string port_name = "/dev/ttyUSB0";
  int baudrate = 3000000;  // 3Mbps
  std::string config_file = "../config/crane-x7_velocity.yaml";

  rt_manipulators_cpp::Hardware hardware(port_name);
  if (!hardware.connect(baudrate)) {
    std::cerr << "ロボットとの接続に失敗しました." << std::endl;
    return -1;
  }

  if (!hardware.load_config_file(config_file)) {
    std::cerr << "コンフィグファイルの読み込みに失敗しました." << std::endl;
    return -1;
  }

  if (!hardware.torque_on("forearm")) {
    std::cerr << "forearmグループのトルクをONできませんでした." << std::endl;
    return -1;
  }

  std::cout << "read/writeスレッドを起動します." << std::endl;
  std::vector<std::string> group_names = {"forearm"};
  if (!hardware.start_thread(group_names, std::chrono::milliseconds(10))) {
    std::cerr << "スレッドの起動に失敗しました." << std::endl;
    return -1;
  }

  // 目標速度を段階的に早くする
  const double MAX_VELOCITY = M_PI;
  const double STEPS = 10;

  for(int i=0; i<STEPS; i++){
    double goal_velocity = M_PI * i / static_cast<double>(STEPS);
    std::cout << "goal_velocity:" << goal_velocity << " rad/s" << std::endl;
    hardware.set_velocity(6, goal_velocity);
    hardware.set_velocity(7, goal_velocity);
    hardware.set_velocity(8, goal_velocity);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    hardware.set_velocity("joint5", -goal_velocity);
    hardware.set_velocity("joint6", -goal_velocity);
    hardware.set_velocity("joint7", -goal_velocity);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  std::vector<double> goal_velocities = {0.0, 0.0, 0.0};
  hardware.set_velocities("forearm", goal_velocities);

  std::cout << "スレッドを停止します." << std::endl;
  hardware.stop_thread();

  if (!hardware.torque_off("forearm")) {
    std::cerr << "forearmグループのトルクをOFFできませんでした." << std::endl;
  }

  std::cout << "CRANE-X7との接続を解除します." << std::endl;
  hardware.disconnect();
  return 0;
}
