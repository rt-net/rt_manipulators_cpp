#include <iostream>

#include "hardware.hpp"

int main()
{
  std::cout << "Hello world" << std::endl;
  rt_manipulators_cpp::Hardware hardware("/dev/ttyUSB0");
  hardware.load_config_file("../config/crane-x7.yaml");

  if(!hardware.connect()){
    std::cerr<<"Failed to connect."<<std::endl;
    return -1;
  }
  
  hardware.disconnect();
  std::cout << "End."<< std::endl;
  return 0;
}
