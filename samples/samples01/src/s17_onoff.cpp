
#include <chrono>
#include <iostream>
#include <thread>
#include "rt_manipulators_cpp/hardware.hpp"

int main()
{
  std::cout<<"Sciurus17のトルクをON/OFFするサンプルです."<<std::endl;

  std::string port_name = "/dev/ttyUSB0";
  int baudrate = 3000000;  // 3Mbps
  std::string config_file = "../config/sciurus17.yaml";

  std::cout<<"Sciurus17(";
  std::cout<<"ポート:" << port_name;
  std::cout<<" ボーレート:" << std::to_string(baudrate);
  std::cout<<")に接続します." << std::endl;

  rt_manipulators_cpp::Hardware hardware(port_name);
  if(!hardware.connect(baudrate)){
    std::cerr<<"ロボットとの接続に失敗しました."<<std::endl;
    return -1;
  }

  std::cout<<"コンフィグファイル:"<<config_file<<"を読み込みます."<<std::endl;
  if(!hardware.load_config_file(config_file)){
    std::cerr<<"コンフィグファイルの読み込みに失敗しました."<<std::endl;
    return -1;
  }

  std::string group_name = "right_arm";
  std::cout<<"サーボグループ:"<<group_name<<"のトルクをONにします."<<std::endl;
  if(!hardware.torque_on(group_name)){
    std::cerr<<group_name<<"グループのトルクをONできませんでした."<<std::endl;
    return -1;
  }

  std::cout<<"10秒間スリープします.ロボットに触れるとトルクがONになっていることがわかります."<<std::endl;
  std::cout<<"Sciurus17制御基板の通信タイムアウト機能が働くと、トルクON後に脱力します."<<std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(10));

  std::cout<<"サーボグループ:"<<group_name<<"のトルクをOFFにします."<<std::endl;
  if(!hardware.torque_off(group_name)){
    std::cerr<<group_name<<"グループのトルクをOFFできませんでした."<<std::endl;
  }
  
  std::cout<<"Sciurus17との接続を解除します."<<std::endl;
  hardware.disconnect();
  return 0;
}