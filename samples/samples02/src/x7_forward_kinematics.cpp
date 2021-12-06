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
#include "rt_manipulators_cpp/kinematics.hpp"
#include "rt_manipulators_cpp/kinematics_utils.hpp"
#include "rt_manipulators_cpp/link.hpp"

void set_arm_joint_positions(std::vector<manipulators_link::Link> & links, std::vector<double> positions) {
  // リンクにarmジョイントの現在角度をセットする
  if (positions.size() != 7) {
    std::cerr << "引数positionsには7個のジョイント角度をセットしてください" << std::endl;
    return;
  }

  int start_id = 2;  // Link1
  for (int i=0; i < positions.size(); i++) {
    links[start_id + i].q = positions[i];
  }
}

int main() {
  std::cout << "CRANE-X7のサーボモータ角度を読み取り、順運動学を解くサンプルです." << std::endl;

  std::string port_name = "/dev/ttyUSB0";
  int baudrate = 3000000;  // 3Mbps
  std::string hardware_config_file = "../config/crane-x7.yaml";
  std::string link_config_file = "../config/crane-x7_links.csv";

  auto links = kinematics_utils::parse_link_config_file(link_config_file);
  kinematics::forward_kinematics(links, 1);
  kinematics_utils::print_links(links, 1);

  rt_manipulators_cpp::Hardware hardware(port_name);
  if (!hardware.connect(baudrate)) {
    std::cerr << "ロボットとの接続に失敗しました." << std::endl;
    return -1;
  }

  if (!hardware.load_config_file(hardware_config_file)) {
    std::cerr << "コンフィグファイルの読み込みに失敗しました." << std::endl;
    return -1;
  }

  std::cout << "read/writeスレッドを起動します." << std::endl;
  std::vector<std::string> group_names = {"arm", "hand"};
  if (!hardware.start_thread(group_names, std::chrono::milliseconds(10))) {
    std::cerr << "スレッドの起動に失敗しました." << std::endl;
    return -1;
  }

  std::this_thread::sleep_for(std::chrono::seconds(3));

  for (int i = 0; i < 4000; i++) {
    std::vector<double> positions;
    if (hardware.get_positions("arm", positions)) {
      set_arm_joint_positions(links, positions);
      kinematics::forward_kinematics(links, 1);
      int target_link = 8;
      std::cout << "リンク名: " << links[target_link].name << std::endl;
      auto pos_xyz = links[target_link].p;
      std::cout << "位置 X: " <<  pos_xyz[0] << "\t[m]" << std::endl;
      std::cout << "位置 Y: " <<  pos_xyz[1] << "\t[m]" << std::endl;
      std::cout << "位置 Z: " <<  pos_xyz[2] << "\t[m]" << std::endl;
      auto euler_zyx = kinematics_utils::rotation_to_euler_ZYX(links[target_link].R);
      std::cout << "姿勢 Z:" << euler_zyx[0] * 180.0 / M_PI << "\t[deg]" << std::endl;
      std::cout << "姿勢 Y:" << euler_zyx[1] * 180.0 / M_PI << "\t[deg]" << std::endl;
      std::cout << "姿勢 X:" << euler_zyx[2] * 180.0 / M_PI << "\t[deg]" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  std::cout << "スレッドを停止します." << std::endl;
  hardware.stop_thread();

  std::cout << "CRANE-X7との接続を解除します." << std::endl;
  hardware.disconnect();
  return 0;
}
