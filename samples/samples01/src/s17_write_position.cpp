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
  std::cout << "Sciurus17のサーボモータ目標角度を書き込むサンプルです." << std::endl;

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

  std::cout
      << "right_handグループのサーボ最大加速度を0.5pi rad/s^2、最大速度を0.5pi rad/sに設定します."
      << std::endl;
  if (!hardware.write_max_acceleration_to_group("right_hand", 0.5 * M_PI)) {
    std::cerr << "right_handグループの最大加速度を設定できませんでした." << std::endl;
    return -1;
  }

  if (!hardware.write_max_velocity_to_group("right_hand", 0.5 * M_PI)) {
    std::cerr << "right_handグループの最大速度を設定できませんでした." << std::endl;
    return -1;
  }

  std::cout << "torsoグループのサーボ最大加速度を0.5pi rad/s^2、最大速度を0.5pi rad/sに設定します."
            << std::endl;
  if (!hardware.write_max_acceleration_to_group("torso", 0.5 * M_PI)) {
    std::cerr << "torsoグループの最大加速度を設定できませんでした." << std::endl;
    return -1;
  }

  if (!hardware.write_max_velocity_to_group("torso", 0.5 * M_PI)) {
    std::cerr << "torsoグループの最大速度を設定できませんでした." << std::endl;
    return -1;
  }

  std::cout << "right_hand、torsoグループのサーボ位置制御PIDゲインに(800, 0, 0)を書き込みます."
            << std::endl;
  if (!hardware.write_position_pid_gain_to_group("right_hand", 800, 0, 0)) {
    std::cerr << "right_handグループにPIDゲインを書き込めませんでした." << std::endl;
    return -1;
  }
  if (!hardware.write_position_pid_gain_to_group("torso", 800, 0, 0)) {
    std::cerr << "torsoグループにPIDゲインを書き込めませんでした." << std::endl;
    return -1;
  }

  if (!hardware.torque_on("right_hand")) {
    std::cerr << "right_handグループのトルクをONできませんでした." << std::endl;
    return -1;
  }

  if (!hardware.torque_on("torso")) {
    std::cerr << "torsoグループのトルクをONできませんでした." << std::endl;
    return -1;
  }

  std::vector<double> target_positions(3, 0.0);
  std::cout << "torsoグループのサーボ目標角度に0.0 radを書き込みます." << std::endl;
  std::cout << "5秒後にSciurus17が正面を向く姿勢へ移行するため、Sciurus17の周りに物や人を近づけない"
               "で下さい."
            << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(5));
  hardware.set_positions("torso", target_positions);
  if (!hardware.sync_write("torso")) {
    std::cerr << "torsoグループのsync writeに失敗しました." << std::endl;
  }
  std::cout << "5秒間スリープして動作完了を待ちます." << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(5));

  int dxl_id = 9;
  double target_position = 30.0 * M_PI / 180.0;  // radian
  std::cout << "ID:" << std::to_string(dxl_id) << "のサーボ目標角度に"
            << std::to_string(target_position) << " radを書き込みます." << std::endl;
  hardware.set_position(dxl_id, target_position);
  if (!hardware.sync_write("right_hand")) {
    std::cerr << "right_handグループのsync writeに失敗しました." << std::endl;
  }
  std::cout << "5秒間スリープして動作完了を待ちます." << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(5));

  std::string joint_name = "right_arm_joint_hand";
  target_position = 0.0;  // radian
  std::cout << joint_name << "ジョイントのサーボ目標角度に" << std::to_string(target_position)
            << " radを書き込みます." << std::endl;
  hardware.set_position(joint_name, target_position);
  if (!hardware.sync_write("right_hand")) {
    std::cerr << "right_handグループのsync writeに失敗しました." << std::endl;
  }
  std::cout << "5秒間スリープして動作完了を待ちます." << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(5));

  std::cout
      << "right_hand、torsoグループのサーボ位置制御PIDゲインに(5, 0, 0)を書き込み、脱力させます."
      << std::endl;
  if (!hardware.write_position_pid_gain_to_group("right_hand", 5, 0, 0)) {
    std::cerr << "right_handグループにPIDゲインを書き込めませんでした." << std::endl;
  }
  if (!hardware.write_position_pid_gain_to_group("torso", 5, 0, 0)) {
    std::cerr << "torsoグループにPIDゲインを書き込めませんでした." << std::endl;
  }
  std::cout << "5秒間スリープします." << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(5));

  if (!hardware.torque_off("right_hand")) {
    std::cerr << "right_handグループのトルクをOFFできませんでした." << std::endl;
  }

  if (!hardware.torque_off("torso")) {
    std::cerr << "torsoグループのトルクをOFFできませんでした." << std::endl;
  }

  if (!hardware.write_position_pid_gain_to_group("right_hand", 800, 0, 0)) {
    std::cerr << "right_handグループにPIDゲインを書き込めませんでした." << std::endl;
  }

  if (!hardware.write_position_pid_gain_to_group("torso", 800, 0, 0)) {
    std::cerr << "torsoグループにPIDゲインを書き込めませんでした." << std::endl;
  }
  std::this_thread::sleep_for(std::chrono::seconds(1));

  std::cout << "Sciurus17との接続を解除します." << std::endl;
  hardware.disconnect();
  return 0;
}
