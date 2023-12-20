// Copyright 2023 RT Corporation
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

#include "gtest/gtest.h"
#include "rt_manipulators_cpp/hardware.hpp"


TEST(HardwareTest, write_data) {

  rt_manipulators_cpp::Hardware hardware("/dev/ttyUSB0");
  uint16_t test_addr = 0x1234;
  uint8_t test_data = 0x56;
  EXPECT_EQ(hardware.write_data("unregistered_id", test_addr, test_data), false);
  EXPECT_EQ(hardware.write_data(0, test_addr, test_data), false);

  hardware.load_config_file("../config/ok_single_joint.yaml");
  EXPECT_EQ(hardware.write_data("joint1", test_addr, test_data), true);
}
