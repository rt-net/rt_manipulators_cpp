
#include <chrono>
#include <iostream>
#include <numeric>
#include <thread>
#include <vector>

#include "hardware.hpp"

using std::this_thread::sleep_for;

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

  // if(!hardware.torque_on("arm")){
  //   std::cerr<<"グループのトルクをONできませんでした."<<std::endl;
  //   return -1;
  // }

  auto current_time = std::chrono::steady_clock::now();
  auto next_start_time = current_time;
  std::vector<std::string> group_names = {"arm", "hand"};

  hardware.start_thread(group_names, std::chrono::milliseconds(10));
  for(int i=0; i<50; i++){
    current_time = std::chrono::steady_clock::now();
    next_start_time = current_time + std::chrono::milliseconds(100);

    for(auto group_name : group_names){
      std::vector<double> positions;
      if(hardware.get_positions(group_name, positions)){
        int index=0;
        for(auto position : positions){
          std::cout<<group_name<<" :"<<std::to_string(index)<<"->pos:"<<std::to_string(position)<<std::endl;
          index++;
        }
      }
    }

    std::this_thread::sleep_until(next_start_time);
  }
  hardware.stop_thread();

  // if(!hardware.torque_off("arm")){
  //   std::cerr<<"グループのトルクをOFFできませんでした."<<std::endl;
  // }
  
  hardware.disconnect();
  return 0;
}
