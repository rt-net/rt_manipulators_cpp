// Copyright 2022 RT Corporation
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

#include <cmath>
#include <memory>

#include "gtest/gtest.h"
#include "rt_manipulators_cpp/dynamixel_base.hpp"
#include "rt_manipulators_cpp/dynamixel_xh430.hpp"
#include "rt_manipulators_cpp/dynamixel_xh540.hpp"

TEST(DynamixelXHTest, create_xh_series_instance) {
  std::shared_ptr<dynamixel_base::DynamixelBase> dxl;
  dxl = std::make_shared<dynamixel_xh430::DynamixelXH430>(1);
  EXPECT_EQ(dxl->get_name(), "XH430");
  dxl = std::make_shared<dynamixel_xh540::DynamixelXH540>(1);
  EXPECT_EQ(dxl->get_name(), "XH540");
}

TEST(DynamixelXHTest, to_current_ampere_430) {
  std::shared_ptr<dynamixel_base::DynamixelBase> dxl;
  dxl = std::make_shared<dynamixel_xh430::DynamixelXH430>(1);
  EXPECT_DOUBLE_EQ(dxl->to_current_ampere(0), 0.0);
  EXPECT_DOUBLE_EQ(dxl->to_current_ampere(1000), 1.34);
  EXPECT_DOUBLE_EQ(dxl->to_current_ampere(-1000), -1.34);
}

TEST(DynamixelXHTest, from_current_ampere_430) {
  std::shared_ptr<dynamixel_base::DynamixelBase> dxl;
  dxl = std::make_shared<dynamixel_xh430::DynamixelXH430>(1);
  EXPECT_EQ(dxl->from_current_ampere(0.0), 0);
  EXPECT_EQ(dxl->from_current_ampere(1.341), 1000);
  EXPECT_EQ(dxl->from_current_ampere(-1.341), 0xFFFFFC18);  // -1000
}
