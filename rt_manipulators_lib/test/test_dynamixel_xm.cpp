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

#include <memory>

#include "gtest/gtest.h"
#include "rt_manipulators_cpp/dynamixel_base.hpp"
#include "rt_manipulators_cpp/dynamixel_xm.hpp"
#include "rt_manipulators_cpp/dynamixel_xm430.hpp"
#include "rt_manipulators_cpp/hardware_communicator.hpp"

TEST(DynamixelXMTest, create_xm_series_instance) {
  std::shared_ptr<dynamixel_base::DynamixelBase> dxl;
  dxl = std::make_shared<dynamixel_xm430::DynamixelXM430>(1);
  EXPECT_EQ(dxl->get_name(), "XM430");
}

class XMTestFixture : public ::testing::Test {
 protected:
  virtual void SetUp() {
    dxl = std::make_shared<dynamixel_xm::DynamixelXM>(1);
    comm = std::make_shared<hardware_communicator::Communicator>("dummy_port");
  }

  virtual void TearDown() {
    dxl.reset();
  }

  std::shared_ptr<dynamixel_base::DynamixelBase> dxl;
  std::shared_ptr<hardware_communicator::Communicator> comm;
};

TEST_F(XMTestFixture, get_id) {
  ASSERT_EQ(dxl->get_id(), 1);
}

TEST_F(XMTestFixture, write_torque_enable) {
  // Dynamixelが接続されていないため、通信が関わるテストはFalseを返す
  ASSERT_FALSE(dxl->write_torque_enable(comm, false));
}

TEST_F(XMTestFixture, write_velocity_i_gain) {
  ASSERT_FALSE(dxl->write_velocity_i_gain(comm, 123));
}

TEST_F(XMTestFixture, write_velocity_p_gain) {
  ASSERT_FALSE(dxl->write_velocity_p_gain(comm, 123));
}

TEST_F(XMTestFixture, write_position_d_gain) {
  ASSERT_FALSE(dxl->write_position_d_gain(comm, 123));
}

TEST_F(XMTestFixture, write_position_i_gain) {
  ASSERT_FALSE(dxl->write_position_i_gain(comm, 123));
}

TEST_F(XMTestFixture, write_position_p_gain) {
  ASSERT_FALSE(dxl->write_position_p_gain(comm, 123));
}

TEST_F(XMTestFixture, write_profile_acceleration) {
  ASSERT_FALSE(dxl->write_profile_acceleration(comm, 0));
}

TEST_F(XMTestFixture, write_profile_velocity) {
  ASSERT_FALSE(dxl->write_profile_velocity(comm, 0));
}

TEST_F(XMTestFixture, to_profile_acceleration) {
  // rad/s^2 to rev/min^2
  // 0以下に対しては1を返すことを期待
  EXPECT_EQ(dxl->to_profile_acceleration(-1), 1);
  EXPECT_EQ(dxl->to_profile_acceleration(0), 1);
  EXPECT_EQ(dxl->to_profile_acceleration(3.745076), 10);
  EXPECT_EQ(dxl->to_profile_acceleration(1000000), 32767);
}

TEST_F(XMTestFixture, to_profile_velocity) {
  // rad/s to rev/min
  // 0以下に対しては1を返すことを期待
  EXPECT_EQ(dxl->to_profile_velocity(-1), 1);
  EXPECT_EQ(dxl->to_profile_velocity(0), 1);
  EXPECT_EQ(dxl->to_profile_velocity(0.239809), 10);
  EXPECT_EQ(dxl->to_profile_velocity(1000000), 32767);
}
