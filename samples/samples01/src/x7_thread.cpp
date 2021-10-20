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

int main()
{
  std::cout<<"スレッドを起動し、CRANE-X7のサーボモータ角度を読み書きするサンプルです."<<std::endl;

  std::string port_name = "/dev/ttyUSB0";
  int baudrate = 3000000;  // 3Mbps
  std::string config_file = "../config/crane-x7.yaml";

  rt_manipulators_cpp::Hardware hardware(port_name);
  if(!hardware.connect(baudrate)){
    std::cerr<<"ロボットとの接続に失敗しました."<<std::endl;
    return -1;
  }

  if(!hardware.load_config_file(config_file)){
    std::cerr<<"コンフィグファイルの読み込みに失敗しました."<<std::endl;
    return -1;
  }

  std::cout<<"handジョイントグループのトルクをONにします."<<std::endl;
  if(!hardware.torque_on("hand")){
    std::cerr<<"handグループのトルクをONできませんでした."<<std::endl;
    return -1;
  }

  std::cout<<"read/writeスレッドを起動します."<<std::endl;
  std::vector<std::string> group_names = {"arm", "hand"};
  if(!hardware.start_thread(group_names, std::chrono::milliseconds(10))){
    std::cerr<<"スレッドの起動に失敗しました."<<std::endl;
    return -1;
  }

  std::cout<<"5秒後にCRANE-X7のハンドが開くので、ハンドに触れないでください."<<std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(5));

  for(int i=0; i<2000 ; i++){
    for(std::string group_name : group_names){
      std::vector<double> positions;
      if(hardware.get_positions(group_name, positions)){
        for(int i=0; i<positions.size(); i++){
          std::cout<<group_name<<"グループの"<<std::to_string(i)<<"番目のサーボ角度は"<<std::to_string(positions[i])<<"radです."<<std::endl;
        }
      }
    }
    // 肘の関節を開くと、ハンドが開くように目標角度を設定する
    double position;
    hardware.get_position("joint4", position);
    hardware.set_position("joint_hand", 0.5 * (M_PI + position));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  std::cout<<"スレッドを停止します."<<std::endl;
  hardware.stop_thread();

  if(!hardware.torque_off("hand")){
    std::cerr<<"handグループのトルクをOFFできませんでした."<<std::endl;
  }
  
  std::cout<<"CRANE-X7との接続を解除します."<<std::endl;
  hardware.disconnect();
  return 0;
}