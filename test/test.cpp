
#include <chrono>
#include <iostream>
#include <numeric>
#include <thread>

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

  int NUM_LOOPS = 1000;
  int loop_count = 0;

  auto current_time = std::chrono::steady_clock::now();
  auto next_start_time = current_time;

  while(loop_count < NUM_LOOPS){
    current_time = std::chrono::steady_clock::now();
    next_start_time = current_time + std::chrono::milliseconds(10);

    if(!hardware.sync_read("arm")){
      std::cout<<"SyncReadに失敗しました."<<std::endl;
      break;
    }

    std::vector<double> positions;
    if(hardware.get_positions("arm", positions)){
      int index=0;
      for(auto position : positions){
        std::cout<<std::to_string(index)<<"->pos:"<<std::to_string(position)<<std::endl;
        index++;
      }

    }

    loop_count++;
    std::this_thread::sleep_until(next_start_time);
  }

  if(!hardware.torque_off("arm")){
    std::cerr<<"グループのトルクをOFFできませんでした."<<std::endl;
  }
  
  hardware.disconnect();
  return 0;
}
