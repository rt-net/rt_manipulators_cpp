
#include <chrono>
#include <iostream>
#include <thread>

#include "hardware.hpp"

using std::this_thread::sleep_for;

int main()
{
  rt_manipulators_cpp::Hardware hardware("/dev/ttyUSB0");
  hardware.load_config_file("../config/crane-x7.yaml");


  if(!hardware.connect(3000000)){
    std::cerr<<"Failed to connect."<<std::endl;
    return -1;
  }

  if(!hardware.torque_on("arm")){
    std::cerr<<"Failed to set torque on."<<std::endl;
    return -1;
  }

  sleep_for(std::chrono::milliseconds(3000));

  hardware.torque_off("arm");
  
  hardware.disconnect();
  std::cout << "End."<< std::endl;
  return 0;
}
