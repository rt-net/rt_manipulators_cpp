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
  std::cout << "Sciurus17のサーボモータ角度を読み取るサンプルです." << std::endl;

  std::string port_name = "/dev/ttyUSB0";
  int baudrate = 3000000;  // 3Mbps
  std::string config_file = "../config/sciurus17.yaml";

  rt_manipulators_cpp::Hardware hardware(port_name);
  if (!hardware.connect(baudrate)) {
    std::cerr << "ロボットとの接続に失敗しました." << std::endl;
    return -1;
  }

  if (!hardware.load_config_file(config_file)) {
    std::cerr << "コンフィグファイルの読み込みに失敗しました." << std::endl;
    return -1;
  }

  for (int i = 0; i < 1000; i++) {
    if (!hardware.sync_read("right_arm")) {
      std::cerr << "right_armグループのsync readに失敗しました." << std::endl;
      break;
    }

    if (!hardware.sync_read("right_hand")) {
      std::cerr << "right_handグループのsync readに失敗しました." << std::endl;
      break;
    }

    if (!hardware.sync_read("left_arm")) {
      std::cerr << "left_armグループのsync readに失敗しました." << std::endl;
      break;
    }

    if (!hardware.sync_read("left_hand")) {
      std::cerr << "left_handグループのsync readに失敗しました." << std::endl;
      break;
    }

    if (!hardware.sync_read("torso")) {
      std::cerr << "torsoグループのsync readに失敗しました." << std::endl;
      break;
    }

    double position;
    int dxl_id = 2;
    if (hardware.get_position(dxl_id, position)) {
      std::cout << "ID:" << std::to_string(dxl_id) << "のサーボ角度は" << std::to_string(position)
                << "radです." << std::endl;
    }

    std::string joint_name = "left_arm_joint_hand";
    if (hardware.get_position(joint_name, position)) {
      std::cout << joint_name << "のサーボ角度は" << std::to_string(position) << "radです."
                << std::endl;
    }

    std::vector<double> positions;
    if (hardware.get_positions("torso", positions)) {
      for (int i = 0; i < positions.size(); i++) {
        std::cout << "torsoグループの" << std::to_string(i) << "番目のサーボ角度は"
                  << std::to_string(positions[i]) << "radです." << std::endl;
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  std::cout << "Sciurus17との接続を解除します." << std::endl;
  hardware.disconnect();
  return 0;
}
