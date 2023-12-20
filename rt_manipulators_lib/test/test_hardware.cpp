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

#include <memory>

#include "fakeit.hpp"
#include "gtest/gtest.h"
#include "rt_manipulators_cpp/hardware.hpp"
#include "rt_manipulators_cpp/hardware_communicator.hpp"

using namespace fakeit;

class HardwareTestFixture: public ::testing::Test {
 protected:
  virtual void SetUp() {
    Mock<hardware_communicator::Communicator> mock;
    When(Method(mock, read_byte_data)).AlwaysReturn(true);
    When(Method(mock, read_word_data)).AlwaysReturn(true);
    When(Method(mock, read_double_word_data)).AlwaysReturn(true);

    hardware = std::make_shared<rt_manipulators_cpp::Hardware>(
      std::unique_ptr<hardware_communicator::Communicator>(&mock.get()));
  }

  virtual void TearDown() {
  }

  std::shared_ptr<rt_manipulators_cpp::Hardware> hardware;
};

TEST_F(HardwareTestFixture, load_config_file) {
  EXPECT_TRUE(hardware->load_config_file("../config/ok_single_joint.yaml"));
}
