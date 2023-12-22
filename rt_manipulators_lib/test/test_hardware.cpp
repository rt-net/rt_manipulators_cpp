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

using fakeit::Mock;
using fakeit::Verify;
using fakeit::When;

Mock<hardware_communicator::Communicator> create_default_mock(void) {
  Mock<hardware_communicator::Communicator> mock;
  When(Method(mock, is_connected)).AlwaysReturn(true);
  When(Method(mock, connect)).AlwaysReturn(true);
  When(Method(mock, disconnect)).AlwaysReturn();
  When(Method(mock, make_sync_write_group)).AlwaysReturn();
  When(Method(mock, make_sync_read_group)).AlwaysReturn();
  When(Method(mock, append_id_to_sync_write_group)).AlwaysReturn(true);
  When(Method(mock, append_id_to_sync_read_group)).AlwaysReturn(true);
  When(Method(mock, send_sync_read_packet)).AlwaysReturn(true);
  When(Method(mock, send_sync_write_packet)).AlwaysReturn(true);
  When(Method(mock, get_sync_read_data)).AlwaysReturn(true);
  When(Method(mock, set_sync_write_data)).AlwaysReturn(true);
  When(Method(mock, write_byte_data)).AlwaysReturn(true);
  When(Method(mock, write_word_data)).AlwaysReturn(true);
  When(Method(mock, write_double_word_data)).AlwaysReturn(true);
  When(Method(mock, read_byte_data)).AlwaysReturn(true);
  When(Method(mock, read_word_data)).AlwaysReturn(true);
  When(Method(mock, read_double_word_data)).AlwaysReturn(true);

  return mock;
}

TEST(HardwareTest, load_config_file) {
  // Expect the load_config_file method to be called twice and return true and false respectively.
  auto mock = create_default_mock();

  rt_manipulators_cpp::Hardware hardware(
    std::unique_ptr<hardware_communicator::Communicator>(&mock.get()));

  EXPECT_TRUE(hardware.load_config_file("../config/ok_has_dynamixel_name.yaml"));
  EXPECT_FALSE(hardware.load_config_file("../config/ng_has_same_joints.yaml"));
}

TEST(HardwareTest, connect) {
  // Expect the connect method to be called twice and return true and false respectively.
  auto mock = create_default_mock();
  When(Method(mock, connect)).Return(true, false);  // Return true then false.

  rt_manipulators_cpp::Hardware hardware(
    std::unique_ptr<hardware_communicator::Communicator>(&mock.get()));

  EXPECT_TRUE(hardware.connect());
  EXPECT_FALSE(hardware.connect());
}

TEST(HardwareTest, disconnect) {
  // Expect the disconnect method to be called once and never.
  auto mock = create_default_mock();
  When(Method(mock, is_connected)).Return(false).AlwaysReturn(true);  // Return false then true.
  When(Method(mock, disconnect)).AlwaysReturn();

  rt_manipulators_cpp::Hardware hardware(
    std::unique_ptr<hardware_communicator::Communicator>(&mock.get()));

  hardware.disconnect();
  Verify(Method(mock, disconnect)).Never();

  hardware.disconnect();
  Verify(Method(mock, disconnect)).Once();
}
