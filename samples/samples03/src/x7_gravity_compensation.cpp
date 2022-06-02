// Copyright 2022 RT Corporation
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

// For getch(), kbhit()
// Ref: https://github.com/ROBOTIS-GIT/DynamixelSDK/blob/master/c%2B%2B/example/protocol2.0/read_write/read_write.cpp
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#define ESC_ASCII_VALUE 0x1b

#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "rt_manipulators_cpp/hardware.hpp"
#include "rt_manipulators_cpp/kinematics.hpp"
#include "rt_manipulators_cpp/kinematics_utils.hpp"
#include "rt_manipulators_cpp/link.hpp"
#include "rt_manipulators_dynamics.hpp"

int getch() {
  // Ref: https://github.com/ROBOTIS-GIT/DynamixelSDK/blob/c7e1eb71c911b87f7bdeda3c2c9e92276c2b4627/c%2B%2B/example/protocol2.0/read_write/read_write.cpp#L100-L114
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
}

int kbhit(void) {
  // Ref: https://github.com/ROBOTIS-GIT/DynamixelSDK/blob/c7e1eb71c911b87f7bdeda3c2c9e92276c2b4627/c%2B%2B/example/protocol2.0/read_write/read_write.cpp#L116-L143
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF) {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
}

void set_arm_joint_positions(std::vector<manipulators_link::Link> & links,
                             std::vector<double> positions) {
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
  std::cout << "現在姿勢をもとに重力補償トルクを計算し、"
            << "CRANE-X7のサーボモータに入力するサンプルです" << std::endl;

  std::string port_name = "/dev/ttyUSB0";
  int baudrate = 3000000;  // 3Mbps
  std::string hardware_config_file = "../config/crane-x7_current.yaml";
  std::string link_config_file = "../config/crane-x7_links.csv";

  auto links = kinematics_utils::parse_link_config_file(link_config_file);
  kinematics::forward_kinematics(links, 1);

  rt_manipulators_cpp::Hardware hardware(port_name);
  if (!hardware.connect(baudrate)) {
    std::cerr << "ロボットとの接続に失敗しました." << std::endl;
    return -1;
  }

  if (!hardware.load_config_file(hardware_config_file)) {
    std::cerr << "コンフィグファイルの読み込みに失敗しました." << std::endl;
    return -1;
  }

  // 関節可動範囲の設定
  for (auto link_id : kinematics_utils::find_route(links, 8)) {
    hardware.get_max_position_limit(links[link_id].dxl_id, links[link_id].max_q);
    hardware.get_min_position_limit(links[link_id].dxl_id, links[link_id].min_q);
  }

  if (!hardware.torque_on("arm")) {
    std::cerr << "armグループのトルクをONできませんでした." << std::endl;
    return -1;
  }

  std::cout << "read/writeスレッドを起動します." << std::endl;
  std::vector<std::string> group_names = {"arm"};
  if (!hardware.start_thread(group_names, std::chrono::milliseconds(10))) {
    std::cerr << "スレッドの起動に失敗しました." << std::endl;
    return -1;
  }

  std::cout << "5秒後に重力補償トルクをサーボモータへ入力します" << std::endl;
  std::cout << "終了する場合はEscキーを押してください" << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(5));

  kinematics_utils::q_list_t q_list;
  kinematics_utils::link_id_t target_id = 8;
  // トルク・電流比 A/Nm
  // Dynamixelのe-manualに記載されたパラメータをもとに微調整しています
  samples03_dynamics::torque_to_current_t torque_to_current = {
    {2, 1.0 / 2.20},
    {3, 1.0 / 3.60},
    {4, 1.0 / 2.20},
    {5, 1.0 / 2.20},
    {6, 1.0 / 2.20},
    {7, 1.0 / 2.20},
    {8, 1.0 / 2.20}
  };

  while (1) {
    if (kbhit()) {
      if (getch() == ESC_ASCII_VALUE) {
        std::cout << "Escが入力されました" << std::endl;
        break;
      }
    }

    // 姿勢を更新
    std::vector<double> positions;
    if (hardware.get_positions("arm", positions)) {
      set_arm_joint_positions(links, positions);
      kinematics::forward_kinematics(links, 1);
    }

    // ここで重力補償分の電流値を計算
    samples03_dynamics::gravity_compensation(
      links, target_id, torque_to_current, q_list);
    for (const auto & [target_id, q_value] : q_list) {
      hardware.set_current(links[target_id].dxl_id, q_value);
    }
  }


  std::cout << "終了します" << std::endl;

  std::cout << "スレッドを停止します." << std::endl;
  hardware.stop_thread();

  if (!hardware.torque_off("arm")) {
    std::cerr << "armグループのトルクをOFFできませんでした." << std::endl;
  }

  std::cout << "CRANE-X7との接続を解除します." << std::endl;
  hardware.disconnect();
  return 0;
}
