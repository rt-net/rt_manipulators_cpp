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
#include "rt_manipulators_cpp/hardware.hpp"

int main() {
  std::cout << "CRANE-X7のトルクをON/OFFするサンプルです." << std::endl;

  std::string port_name = "/dev/ttyUSB0";
  int baudrate = 3000000;  // 3Mbps
  std::string config_file = "../config/crane-x7.yaml";

  std::cout << "CRANE-X7(";
  std::cout << "ポート:" << port_name;
  std::cout << " ボーレート:" << std::to_string(baudrate);
  std::cout << ")に接続します." << std::endl;

  rt_manipulators_cpp::Hardware hardware(port_name);
  if (!hardware.connect(baudrate)) {
    std::cerr << "ロボットとの接続に失敗しました." << std::endl;
    return -1;
  }

  std::cout << "コンフィグファイル:" << config_file << "を読み込みます." << std::endl;
  if (!hardware.load_config_file(config_file)) {
    std::cerr << "コンフィグファイルの読み込みに失敗しました." << std::endl;
    return -1;
  }

  std::string group_name = "arm";
  std::cout << "ジョイントグループ:" << group_name << "のトルクをONにします." << std::endl;
  if (!hardware.torque_on(group_name)) {
    std::cerr << group_name << "グループのトルクをONできませんでした." << std::endl;
    return -1;
  }

  std::cout << "10秒間スリープします.ロボットに触れるとトルクがONになっていることがわかります."
            << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(10));

  std::cout << "ジョイントグループ:" << group_name << "のトルクをOFFにします." << std::endl;
  if (!hardware.torque_off(group_name)) {
    std::cerr << group_name << "グループのトルクをOFFできませんでした." << std::endl;
  }

  std::cout << "CRANE-X7との接続を解除します." << std::endl;
  hardware.disconnect();
  return 0;
}
