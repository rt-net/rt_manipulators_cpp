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
#include <map>
#include <thread>
#include <vector>
#include "rt_manipulators_cpp/hardware.hpp"
#include "rt_manipulators_cpp/kinematics.hpp"
#include "rt_manipulators_cpp/kinematics_utils.hpp"
#include "rt_manipulators_cpp/link.hpp"

void move_to(rt_manipulators_cpp::Hardware & hardware,
             const kinematics_utils::links_t & links,
             const kinematics_utils::link_id_t & target_id,
             Eigen::Vector3d & target_p, const Eigen::Matrix3d & target_R) {
  // 目標位置・姿勢をもとにIKを解き、関節角度をセットする
  std::cout << "目標位置:" << std::endl << target_p << std::endl;
  std::cout << "目標姿勢:" << std::endl << target_R << std::endl;
  std::cout << "----------------------" << std::endl;
  kinematics_utils::q_list_t q_list;
  if (kinematics::inverse_kinematics_LM(links, target_id, target_p, target_R, q_list) == false) {
    std::cout << "IKに失敗しました" << std::endl;
  } else {
    std::cout << "IKに成功しました" << std::endl;
  }
  for (const auto & [target_id, q_value] : q_list) {
    hardware.set_position(links[target_id].dxl_id, q_value);
  }
}

int main() {
  std::cout << "手先目標位置・姿勢をもとに逆運動学を解き、"
            << "Sciurus17を動かすサンプルです." << std::endl;

  std::string port_name = "/dev/ttyUSB0";
  int baudrate = 3000000;  // 3Mbps
  std::string hardware_config_file = "../config/sciurus17.yaml";
  std::string link_config_file = "../config/sciurus17_links.csv";

  auto links = kinematics_utils::parse_link_config_file(link_config_file);
  kinematics::forward_kinematics(links, 1);
  // kinematics_utils::print_links(links, 1);

  rt_manipulators_cpp::Hardware hardware(port_name);
  if (!hardware.connect(baudrate)) {
    std::cerr << "ロボットとの接続に失敗しました." << std::endl;
    return -1;
  }

  if (!hardware.load_config_file(hardware_config_file)) {
    std::cerr << "コンフィグファイルの読み込みに失敗しました." << std::endl;
    return -1;
  }

  // 手先リンクのIDを設定
  const int RIGHT = 0;
  const int LEFT = 1;
  const std::vector<int> SIDES = {RIGHT, LEFT};
  std::map<int, int> TARGET_LINK_ID{
    {RIGHT, 11},
    {LEFT, 20},
  };
  // 関節可動範囲の設定
  for (const auto & side : SIDES) {
    for (auto link_id : kinematics_utils::find_route(links, TARGET_LINK_ID[side])) {
      hardware.get_max_position_limit(links[link_id].dxl_id, links[link_id].max_q);
      hardware.get_min_position_limit(links[link_id].dxl_id, links[link_id].min_q);
    }
  }

  std::vector<std::string> group_names = {"right_arm", "left_arm", "torso"};
  for (auto group : group_names) {
    std::cout << group << "グループのサーボ最大加速度をpi rad/s^2、最大速度をpi rad/sに設定します."
              << std::endl;
    if (!hardware.write_max_acceleration_to_group(group, 1.0 * M_PI)) {
      std::cerr << group << "グループの最大加速度を設定できませんでした." << std::endl;
      return -1;
    }

    if (!hardware.write_max_velocity_to_group(group, 1.0 * M_PI)) {
      std::cerr << group << "グループの最大速度を設定できませんでした." << std::endl;
      return -1;
    }

    std::cout << group << "グループのサーボ位置制御PIDゲインに(800, 0, 0)を書き込みます."
              << std::endl;
    if (!hardware.write_position_pid_gain_to_group(group, 800, 0, 0)) {
      std::cerr << group << "グループにPIDゲインを書き込めませんでした." << std::endl;
      return -1;
    }

    if (!hardware.torque_on(group)) {
      std::cerr << group << "グループのトルクをONできませんでした." << std::endl;
      return -1;
    }
  }

  std::cout << "read/writeスレッドを起動します." << std::endl;
  if (!hardware.start_thread(group_names, std::chrono::milliseconds(10))) {
    std::cerr << "スレッドの起動に失敗しました." << std::endl;
    return -1;
  }

  std::cout << "5秒後にSciurus17が動作するため、X7の周りに物や人を近づけないで下さい."
            << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(5));

  // 手先は正面に向ける
  std::map<int, Eigen::Matrix3d> target_R{
    {RIGHT, kinematics_utils::rotation_from_euler_ZYX(M_PI_2, -M_PI_2, 0)},
    {LEFT, kinematics_utils::rotation_from_euler_ZYX(-M_PI_2, -M_PI_2, 0)}
  };
  // 上腕と前腕の磁石を付ける姿勢
  std::map<int, std::vector<double>> home_positions = {
    {RIGHT, {-0.3574, -1.57, 0, 2.72, 0, -1.12, 0} },
    {LEFT,  {0.3574, 1.57, 0, -2.72, 0, 1.12, 0} },
  };
  std::map<int, std::string> arm_group_name = {
    {RIGHT, "right_arm"},
    {LEFT, "left_arm"},
  };

  for (const auto & side : SIDES) {
    Eigen::Vector3d target_p;
    kinematics_utils::q_list_t q_list;

    std::cout << "正面へ移動" << std::endl;
    target_p << 0.4, 0.0, 0.6;
    move_to(hardware, links, TARGET_LINK_ID[side], target_p, target_R[side]);
    std::this_thread::sleep_for(std::chrono::seconds(3));

    std::cout << "左上へ移動" << std::endl;
    target_p << 0.4, 0.2, 0.8;
    move_to(hardware, links, TARGET_LINK_ID[side], target_p, target_R[side]);
    std::this_thread::sleep_for(std::chrono::seconds(3));

    std::cout << "左下へ移動" << std::endl;
    target_p << 0.4, 0.2, 0.4;
    move_to(hardware, links, TARGET_LINK_ID[side], target_p, target_R[side]);
    std::this_thread::sleep_for(std::chrono::seconds(3));

    std::cout << "右下へ移動" << std::endl;
    target_p << 0.4, -0.2, 0.4;
    move_to(hardware, links, TARGET_LINK_ID[side], target_p, target_R[side]);
    std::this_thread::sleep_for(std::chrono::seconds(3));

    std::cout << "右上へ移動" << std::endl;
    target_p << 0.4, -0.2, 0.8;
    move_to(hardware, links, TARGET_LINK_ID[side], target_p, target_R[side]);
    std::this_thread::sleep_for(std::chrono::seconds(3));

    std::cout << "正面へ移動" << std::endl;
    target_p << 0.4, 0.0, 0.6;
    move_to(hardware, links, TARGET_LINK_ID[side], target_p, target_R[side]);
    std::this_thread::sleep_for(std::chrono::seconds(3));

    std::cout << "終了姿勢へ移動" << std::endl;
    hardware.set_positions(arm_group_name[side], home_positions[side]);
    std::this_thread::sleep_for(std::chrono::seconds(3));
  }
  std::cout << "腰軸を終了姿勢へ移動" << std::endl;
  hardware.set_position(18, 0.0);
  std::this_thread::sleep_for(std::chrono::seconds(5));

  std::cout << "スレッドを停止します." << std::endl;
  hardware.stop_thread();

  for (auto group : group_names) {
    std::cout << group << "グループのサーボ位置制御PIDゲインに(5, 0, 0)を書き込み、脱力させます."
              << std::endl;
    if (!hardware.write_position_pid_gain_to_group(group, 5, 0, 0)) {
      std::cerr << group << "グループにPIDゲインを書き込めませんでした." << std::endl;
    }
  }

  std::cout << "10秒間スリープします." << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(10));

  for (auto group : group_names) {
    if (!hardware.torque_off(group)) {
      std::cerr << group << "グループのトルクをOFFできませんでした." << std::endl;
    }

    if (!hardware.write_position_pid_gain_to_group(group, 800, 0, 0)) {
      std::cerr << group << "グループにPIDゲインを書き込めませんでした." << std::endl;
    }
  }

  std::cout << "Sciurus17との接続を解除します." << std::endl;
  hardware.disconnect();
  return 0;
}
