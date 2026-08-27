// Copyright 2024 RT Corporation
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
#include "rt_manipulators_cpp/dynamixel_ph54.hpp"


class PH54TestFixture : public ::testing::Test {
 protected:
  virtual void SetUp() {
    dxl = std::make_shared<dynamixel_ph54::DynamixelPH54>(1);
  }

  virtual void TearDown() {
    dxl.reset();
  }

  std::shared_ptr<dynamixel_base::DynamixelBase> dxl;
};

TEST_F(PH54TestFixture, create_ph54_instance) {
  EXPECT_EQ(dxl->get_name(), "PH54");
}

TEST_F(PH54TestFixture, to_profile_acceleration) {
  // rad/s^2 to rev/min^2
  // 0以下に対しては1を返すことを期待
  EXPECT_EQ(dxl->to_profile_acceleration(-1), 1);
  EXPECT_EQ(dxl->to_profile_acceleration(0), 1);
  EXPECT_EQ(dxl->to_profile_acceleration(0.017454), 10);
  EXPECT_EQ(dxl->to_profile_acceleration(1000000), 4255632);
}

TEST_F(PH54TestFixture, to_position_radian) {
  EXPECT_DOUBLE_EQ(dxl->to_position_radian(0), 0.0);
  // 250961 = 0x0003 D451
  // 250961 = 0xFFFC 2BAF
  EXPECT_NEAR(dxl->to_position_radian(0x0003D451), M_PI_2, 0.0001);
  EXPECT_NEAR(dxl->to_position_radian(0xFFFC2BAF), -M_PI_2, 0.0001);
}

TEST_F(PH54TestFixture, from_position_radian) {
  EXPECT_EQ(dxl->from_position_radian(0.0), 0);
  EXPECT_EQ(dxl->from_position_radian(M_PI_2), 250961);
  EXPECT_EQ(dxl->from_position_radian(-M_PI_2), -250961);
}
