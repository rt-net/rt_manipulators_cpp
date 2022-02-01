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

#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "rt_manipulators_cpp/hardware.hpp"
#include "rt_manipulators_cpp/kinematics.hpp"
#include "rt_manipulators_cpp/kinematics_utils.hpp"
#include "rt_manipulators_cpp/link.hpp"
#include "inverse_kinematics.hpp"

void move_to(rt_manipulators_cpp::Hardware & hardware,
             const kinematics_utils::links_t & links,
             Eigen::Vector3d & target_p) {
  // 目標位置をもとにIKを解き、関節角度をセットする
  std::cout << "目標位置:" << std::endl << target_p << std::endl;
  std::cout << "----------------------" << std::endl;
  kinematics_utils::q_list_t q_list;
  if (samples03::x7_3dof_inverse_kinematics(links, target_p, q_list) == false) {
    std::cout << "IKに失敗しました" << std::endl;
  } else {
    std::cout << "IKに成功しました" << std::endl;
  }
  for (const auto & [target_id, q_value] : q_list) {
    hardware.set_position(links[target_id].dxl_id, q_value);
  }
}

int main() {
  std::cout << "手先目標位置をもとに解析的に逆運動学を解き、"
            << "CRANE-X7を動かすサンプルです." << std::endl;

  std::string port_name = "/dev/ttyUSB0";
  int baudrate = 3000000;  // 3Mbps
  std::string hardware_config_file = "../config/crane-x7.yaml";
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

  std::cout << "armグループのサーボ最大加速度をpi rad/s^2、最大速度をpi rad/sに設定します."
            << std::endl;
  if (!hardware.write_max_acceleration_to_group("arm", 1.0 * M_PI)) {
    std::cerr << "armグループの最大加速度を設定できませんでした." << std::endl;
    return -1;
  }

  if (!hardware.write_max_velocity_to_group("arm", 1.0 * M_PI)) {
    std::cerr << "armグループの最大速度を設定できませんでした." << std::endl;
    return -1;
  }

  std::cout << "armグループのサーボ位置制御PIDゲインに(800, 0, 0)を書き込みます."
            << std::endl;
  if (!hardware.write_position_pid_gain_to_group("arm", 800, 0, 0)) {
    std::cerr << "armグループにPIDゲインを書き込めませんでした." << std::endl;
    return -1;
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

  std::cout << "5秒後にX7が垂直姿勢へ移行するため、X7の周りに物や人を近づけないで下さい."
            << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(5));

  // 垂直姿勢へ移動
  std::vector<double> target_positions(7, 0.0);
  hardware.set_positions("arm", target_positions);
  std::this_thread::sleep_for(std::chrono::seconds(5));

  Eigen::Vector3d target_p;
  kinematics_utils::q_list_t q_list;

  std::cout << "正面へ移動" << std::endl;
  target_p << 0.2, 0.0, 0.3;
  move_to(hardware, links, target_p);
  std::this_thread::sleep_for(std::chrono::seconds(5));

  std::cout << "左上へ移動" << std::endl;
  target_p << 0.2, 0.2, 0.5;
  move_to(hardware, links, target_p);
  std::this_thread::sleep_for(std::chrono::seconds(3));

  std::cout << "左下へ移動" << std::endl;
  target_p << 0.2, 0.2, 0.2;
  move_to(hardware, links, target_p);
  std::this_thread::sleep_for(std::chrono::seconds(3));

  std::cout << "右下へ移動" << std::endl;
  target_p << 0.2, -0.2, 0.2;
  move_to(hardware, links, target_p);
  std::this_thread::sleep_for(std::chrono::seconds(3));

  std::cout << "右上へ移動" << std::endl;
  target_p << 0.2, -0.2, 0.5;
  move_to(hardware, links, target_p);
  std::this_thread::sleep_for(std::chrono::seconds(3));

  std::cout << "正面へ移動し、手先を下げる" << std::endl;
  target_p << 0.2, 0.0, 0.15;
  move_to(hardware, links, target_p);
  std::this_thread::sleep_for(std::chrono::seconds(3));

  std::cout << "スレッドを停止します." << std::endl;
  hardware.stop_thread();

  std::cout << "armグループのサーボ位置制御PIDゲインに(5, 0, 0)を書き込み、脱力させます."
            << std::endl;
  if (!hardware.write_position_pid_gain_to_group("arm", 5, 0, 0)) {
    std::cerr << "armグループにPIDゲインを書き込めませんでした." << std::endl;
  }
  std::cout << "10秒間スリープします." << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(10));

  if (!hardware.torque_off("arm")) {
    std::cerr << "armグループのトルクをOFFできませんでした." << std::endl;
  }

  if (!hardware.write_position_pid_gain_to_group("arm", 800, 0, 0)) {
    std::cerr << "armグループにPIDゲインを書き込めませんでした." << std::endl;
  }

  std::cout << "CRANE-X7との接続を解除します." << std::endl;
  hardware.disconnect();
  return 0;
}
