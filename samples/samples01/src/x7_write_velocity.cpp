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
  std::cout << "CRANE-X7のサーボモータに目標速度を書き込むサンプルです." << std::endl;

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

  std::cout << "wristグループのサーボ最大加速度を5pi rad/s^2に設定します."
            << std::endl;
  if (!hardware.write_max_acceleration_to_group("wrist", 5.0 * M_PI)) {
    std::cerr << "wristグループの最大加速度を設定できませんでした." << std::endl;
    return -1;
  }

  std::cout << "wristグループのサーボ速度制御PIゲインに(100, 1920)を書き込みます."
            << std::endl;
  if (!hardware.write_velocity_pi_gain_to_group("wrist", 100, 1920)) {
    std::cerr << "wristグループにPIゲインを書き込めませんでした." << std::endl;
    return -1;
  }
  // PIゲインは指定したサーボモータにも設定できます.
  if (!hardware.write_velocity_pi_gain(7, 100, 1920)) {
    std::cerr << "ID:7ジョイントにPIゲインを書き込めませんでした." << std::endl;
    return -1;
  }
  if (!hardware.write_velocity_pi_gain("joint7", 100, 1920)) {
    std::cerr << "joint7ジョイントにPIゲインを書き込めませんでした." << std::endl;
    return -1;
  }

  if (!hardware.torque_on("wrist")) {
    std::cerr << "wristグループのトルクをONできませんでした." << std::endl;
    return -1;
  }

  std::cout << "read/writeスレッドを起動します." << std::endl;
  std::vector<std::string> group_names = {"wrist"};
  if (!hardware.start_thread(group_names, std::chrono::milliseconds(10))) {
    std::cerr << "スレッドの起動に失敗しました." << std::endl;
    return -1;
  }

  std::cout << "5秒後に手先が動き出すため、手先の周りに物や人を近づけないで下さい." << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(5));

  // 目標速度を段階的に早くする
  const double MAX_VELOCITY = M_PI;
  const double STEPS = 10;
  for (int i=1; i <=STEPS; i++) {
    double goal_velocity = MAX_VELOCITY * i / static_cast<double>(STEPS);
    std::cout << "set velocity:" << goal_velocity << " rad/s" << std::endl;
    hardware.set_velocity(6, goal_velocity);
    hardware.set_velocity(7, goal_velocity);
    hardware.set_velocity(8, goal_velocity);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "set velocity:" << -goal_velocity << " rad/s" << std::endl;
    hardware.set_velocity("joint5", -goal_velocity);
    hardware.set_velocity("joint6", -goal_velocity);
    hardware.set_velocity("joint7", -goal_velocity);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  std::cout << "set velocity: 0.0 rad/s" << std::endl;
  std::vector<double> goal_velocities = {0.0, 0.0, 0.0};
  hardware.set_velocities("wrist", goal_velocities);
  std::this_thread::sleep_for(std::chrono::seconds(1));

  std::cout << "スレッドを停止します." << std::endl;
  hardware.stop_thread();

  if (!hardware.torque_off("wrist")) {
    std::cerr << "wristグループのトルクをOFFできませんでした." << std::endl;
  }

  std::cout << "CRANE-X7との接続を解除します." << std::endl;
  hardware.disconnect();
  return 0;
}
