
#include <chrono>
#include <iostream>
#include <thread>

#include "hardware.hpp"

using std::this_thread::sleep_for;

int main()
{
  rt_manipulators_cpp::Hardware hardware("/dev/ttyUSB0");

  if(!hardware.load_config_file("../config/crane-x7.yaml")){
    std::cerr<<"コンフィグファイルの読み込みに失敗しました."<<std::endl;
    return -1;
  }

  if(!hardware.connect(3000000)){
    std::cerr<<"ロボットとの接続に失敗しました."<<std::endl;
    return -1;
  }

  if(!hardware.torque_on("arm")){
    std::cerr<<"グループのトルクをONできませんでした."<<std::endl;
    return -1;
  }

  sleep_for(std::chrono::milliseconds(3000));

  if(!hardware.torque_off("arm")){
    std::cerr<<"グループのトルクをOFFできませんでした."<<std::endl;
  }
  
  hardware.disconnect();
  return 0;
}
