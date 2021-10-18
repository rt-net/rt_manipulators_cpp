
#include <cmath>
#include <chrono>
#include <iostream>
#include <numeric>
#include <thread>
#include <vector>

#include "hardware.hpp"

using std::this_thread::sleep_for;

double to_radians(const double degree_angle){
  return degree_angle * M_PI / 180.0;
}

int main()
{
  rt_manipulators_cpp::Hardware hardware("/dev/ttyUSB0");

  if(!hardware.connect(3000000)){
    std::cerr<<"ロボットとの接続に失敗しました."<<std::endl;
    return -1;
  }

  if(!hardware.load_config_file("../config/crane-x7.yaml")){
    std::cerr<<"コンフィグファイルの読み込みに失敗しました."<<std::endl;
    return -1;
  }

  if(!hardware.torque_on("arm")){
    std::cerr<<"グループのトルクをONできませんでした."<<std::endl;
    return -1;
  }

  // 角度の目標値をセット

  std::cout<<"角度を書き込みます"<<std::endl;
  std::vector<double> positions = {0, to_radians(30), to_radians(90)};
  std::this_thread::sleep_for(std::chrono::seconds(3));
  hardware.set_positions("arm", positions);
  hardware.sync_write("arm");
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // hardware.set_position(6, to_radians(30));
  // hardware.set_position(7, to_radians(30));
  // hardware.set_position(8, to_radians(30));
  // hardware.sync_write("arm");
  // std::this_thread::sleep_for(std::chrono::seconds(2));

  // hardware.set_position(6, to_radians(-30));
  // hardware.set_position(7, to_radians(-30));
  // hardware.set_position(8, to_radians(-30));
  // hardware.sync_write("arm");
  // std::this_thread::sleep_for(std::chrono::seconds(2));

  // auto current_time = std::chrono::steady_clock::now();
  // auto next_start_time = current_time;
  // std::vector<std::string> group_names = {"arm", "hand"};

  // hardware.start_thread(group_names, std::chrono::milliseconds(10));
  // for(int i=0; i<50; i++){
  //   current_time = std::chrono::steady_clock::now();
  //   next_start_time = current_time + std::chrono::milliseconds(100);

  //   for(auto group_name : group_names){
  //     std::vector<double> positions;
  //     if(hardware.get_positions(group_name, positions)){
  //       int index=0;
  //       for(auto position : positions){
  //         std::cout<<group_name<<" :"<<std::to_string(index)<<"->pos:"<<std::to_string(position)<<std::endl;
  //         index++;
  //       }
  //     }
  //   }

  //   std::this_thread::sleep_until(next_start_time);
  // }
  // hardware.stop_thread();

  if(!hardware.torque_off("arm")){
    std::cerr<<"グループのトルクをOFFできませんでした."<<std::endl;
  }
  
  hardware.disconnect();
  return 0;
}
