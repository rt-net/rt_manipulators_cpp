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

Mock<hardware_communicator::Communicator> create_comm_mock(void) {
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
  auto mock = create_comm_mock();

  rt_manipulators_cpp::Hardware hardware(
    std::unique_ptr<hardware_communicator::Communicator>(&mock.get()));

  EXPECT_TRUE(hardware.load_config_file("../config/ok_has_dynamixel_name.yaml"));
  EXPECT_FALSE(hardware.load_config_file("../config/ng_has_same_joints.yaml"));
}

TEST(HardwareTest, connect) {
  // Expect the connect method to be called twice and return true and false respectively.
  auto mock = create_comm_mock();
  When(Method(mock, connect)).Return(true, false);  // Return true then false.

  rt_manipulators_cpp::Hardware hardware(
    std::unique_ptr<hardware_communicator::Communicator>(&mock.get()));

  EXPECT_TRUE(hardware.connect());
  EXPECT_FALSE(hardware.connect());
}

TEST(HardwareTest, disconnect) {
  // Expect the disconnect method to be called once and never.
  auto mock = create_comm_mock();
  When(Method(mock, is_connected)).Return(false).AlwaysReturn(true);  // Return false then true.
  When(Method(mock, disconnect)).AlwaysReturn();

  rt_manipulators_cpp::Hardware hardware(
    std::unique_ptr<hardware_communicator::Communicator>(&mock.get()));

  hardware.disconnect();
  Verify(Method(mock, disconnect)).Never();

  hardware.disconnect();
  Verify(Method(mock, disconnect)).Once();
}

TEST(HardwareTest, write_data) {
  auto mock = create_comm_mock();
  rt_manipulators_cpp::Hardware hardware(
    std::unique_ptr<hardware_communicator::Communicator>(&mock.get()));

  EXPECT_TRUE(hardware.load_config_file("../config/ok_has_dynamixel_name.yaml"));

  // Return false when joint name or id is not found
  mock.ClearInvocationHistory();
  EXPECT_FALSE(hardware.write_data("joint0", 0x00, static_cast<uint8_t>(0x00)));
  EXPECT_FALSE(hardware.write_data(0, 0x00, static_cast<uint8_t>(0x00)));
  Verify(Method(mock, write_byte_data)).Never();
  Verify(Method(mock, write_word_data)).Never();
  Verify(Method(mock, write_double_word_data)).Never();

  // Identify joint via joint name
  EXPECT_TRUE(hardware.write_data("joint1", 0x00, static_cast<uint8_t>(0x00)));
  Verify(Method(mock, write_byte_data)).Once();

  // Identify joint via joint id
  EXPECT_TRUE(hardware.write_data(2, 0x00, static_cast<uint16_t>(0x00)));
  Verify(Method(mock, write_word_data)).Once();

  EXPECT_TRUE(hardware.write_data("joint3", 0x00, static_cast<uint32_t>(0x00)));
  Verify(Method(mock, write_double_word_data)).Once();

  // Return false when data type is not matched
  mock.ClearInvocationHistory();
  EXPECT_FALSE(hardware.write_data("joint1", 0x00, 0x00));
  Verify(Method(mock, write_byte_data)).Never();
  Verify(Method(mock, write_word_data)).Never();
  Verify(Method(mock, write_double_word_data)).Never();
}

TEST(HardwareTest, read_data) {
  auto mock = create_comm_mock();
  rt_manipulators_cpp::Hardware hardware(
    std::unique_ptr<hardware_communicator::Communicator>(&mock.get()));

  EXPECT_TRUE(hardware.load_config_file("../config/ok_has_dynamixel_name.yaml"));

  uint8_t byte_data = 0x00;
  uint16_t word_data = 0x00;
  uint32_t double_word_data = 0x00;
  // Return false when joint name or id is not found
  mock.ClearInvocationHistory();
  EXPECT_FALSE(hardware.read_data("joint0", 0x00, byte_data));
  EXPECT_FALSE(hardware.read_data(0, 0x00, byte_data));
  Verify(Method(mock, read_byte_data)).Never();
  Verify(Method(mock, read_word_data)).Never();
  Verify(Method(mock, read_double_word_data)).Never();

  // Identify joint via joint name
  EXPECT_FALSE(hardware.read_data("joint1", 0x00, byte_data));
  Verify(Method(mock, read_byte_data)).Once();

  // Identify joint via joint id
  EXPECT_FALSE(hardware.read_data(2, 0x00, word_data));
  Verify(Method(mock, read_word_data)).Once();

  EXPECT_FALSE(hardware.read_data("joint3", 0x00, double_word_data));
  Verify(Method(mock, read_double_word_data)).Once();

  // Return false when data type is not matched
  double invalid_type_data = 0.0;
  mock.ClearInvocationHistory();
  EXPECT_FALSE(hardware.read_data("joint1", 0x00, invalid_type_data));
  Verify(Method(mock, read_byte_data)).Never();
  Verify(Method(mock, read_word_data)).Never();
  Verify(Method(mock, read_double_word_data)).Never();
}
