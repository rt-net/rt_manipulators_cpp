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
#include "rt_manipulators_cpp/dynamixel/dynamixel_base.hpp"
#include "rt_manipulators_cpp/dynamixel/dynamixel_xm.hpp"
#include "rt_manipulators_cpp/dynamixel/dynamixel_xm430.hpp"
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
