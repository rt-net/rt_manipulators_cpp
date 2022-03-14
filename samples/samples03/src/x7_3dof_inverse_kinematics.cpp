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
#include "rt_manipulators_ik.hpp"

void move_to(rt_manipulators_cpp::Hardware & hardware,
             const kinematics_utils::links_t & links,
             Eigen::Vector3d & target_p,
             bool move_to_picking_pose = false) {
  // 目標位置をもとにIKを解き、関節角度をセットする
  std::cout << "----------------------" << std::endl;
  std::cout << "目標位置:" << std::endl << target_p << std::endl;
  kinematics_utils::q_list_t q_list;

  if (move_to_picking_pose == true &&
      samples03::x7_3dof_picking_inverse_kinematics(links, target_p, q_list) == false) {
    std::cout << "ピッキング姿勢のIKに失敗しました" << std::endl;
    return;
  } else if (move_to_picking_pose == false &&
             samples03::x7_3dof_inverse_kinematics(links, target_p, q_list) == false) {
    std::cout << "IKに失敗しました" << std::endl;
    return;
  }

  std::cout << "IKに成功しました" << std::endl;
  for (const auto & [target_id, q_value] : q_list) {
    hardware.set_position(links[target_id].dxl_id, q_value);
  }
}

void move_demo(rt_manipulators_cpp::Hardware & hardware,
             const kinematics_utils::links_t & links,
             bool move_to_picking_pose = false) {
  Eigen::Vector3d target_p;
  double target_z = 0.2;
  if (move_to_picking_pose) {
    // ピッキング姿勢のIKを解く場合は、目標位置が手先となる
    target_z = 0.05;
  }

  std::cout << "左奥へ移動" << std::endl;
  target_p << 0.3, 0.1, target_z;
  move_to(hardware, links, target_p, move_to_picking_pose);
  std::this_thread::sleep_for(std::chrono::seconds(2));

  std::cout << "左手前へ移動" << std::endl;
  target_p << 0.1, 0.1, target_z;
  move_to(hardware, links, target_p, move_to_picking_pose);
  std::this_thread::sleep_for(std::chrono::seconds(2));

  std::cout << "右手前へ移動" << std::endl;
  target_p << 0.1, -0.1, target_z;
  move_to(hardware, links, target_p, move_to_picking_pose);
  std::this_thread::sleep_for(std::chrono::seconds(2));

  std::cout << "右奥へ移動" << std::endl;
  target_p << 0.3, -0.1, target_z;
  move_to(hardware, links, target_p, move_to_picking_pose);
  std::this_thread::sleep_for(std::chrono::seconds(2));
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

  std::cout << "正面へ移動" << std::endl;
  target_p << 0.2, 0.0, 0.2;
  move_to(hardware, links, target_p);
  std::this_thread::sleep_for(std::chrono::seconds(3));

  move_demo(hardware, links, false);
  std::cout << "ピッキング姿勢のIKを解きます" << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(2));
  move_demo(hardware, links, true);

  std::cout << "正面へ移動し、手先を下げる" << std::endl;
  target_p << 0.2, 0.0, 0.01;
  move_to(hardware, links, target_p, true);
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
