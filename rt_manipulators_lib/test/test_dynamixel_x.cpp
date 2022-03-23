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
#include "rt_manipulators_cpp/dynamixel_x.hpp"
#include "rt_manipulators_cpp/dynamixel_xm430.hpp"
#include "rt_manipulators_cpp/hardware_communicator.hpp"

TEST(DynamixelXTest, create_xm_series_instance) {
  std::shared_ptr<dynamixel_base::DynamixelBase> dxl;
  dxl = std::make_shared<dynamixel_xm430::DynamixelXM430>(1);
  EXPECT_EQ(dxl->get_name(), "XM430");
}

class XTestFixture : public ::testing::Test {
 protected:
  virtual void SetUp() {
    dxl = std::make_shared<dynamixel_x::DynamixelX>(1);
    comm = std::make_shared<hardware_communicator::Communicator>("dummy_port");
  }

  virtual void TearDown() {
    dxl.reset();
  }

  std::shared_ptr<dynamixel_base::DynamixelBase> dxl;
  std::shared_ptr<hardware_communicator::Communicator> comm;
};

TEST_F(XTestFixture, get_id) {
  ASSERT_EQ(dxl->get_id(), 1);
}

TEST_F(XTestFixture, read_operating_mode) {
  uint8_t mode;
  // Dynamixelが接続されていないため、通信が関わるテストはFalseを返す
  ASSERT_FALSE(dxl->read_operating_mode(comm, mode));
}

TEST_F(XTestFixture, write_operating_mode) {
  ASSERT_FALSE(dxl->write_operating_mode(comm, false));
}

TEST_F(XTestFixture, read_current_limit) {
  double limit;
  ASSERT_FALSE(dxl->read_current_limit(comm, limit));
}

TEST_F(XTestFixture, read_max_position_limit) {
  double limit;
  ASSERT_FALSE(dxl->read_max_position_limit(comm, limit));
}

TEST_F(XTestFixture, read_min_position_limit) {
  double limit;
  ASSERT_FALSE(dxl->read_min_position_limit(comm, limit));
}

TEST_F(XTestFixture, write_torque_enable) {
  ASSERT_FALSE(dxl->write_torque_enable(comm, false));
}

TEST_F(XTestFixture, write_velocity_i_gain) {
  ASSERT_FALSE(dxl->write_velocity_i_gain(comm, 123));
}

TEST_F(XTestFixture, write_velocity_p_gain) {
  ASSERT_FALSE(dxl->write_velocity_p_gain(comm, 123));
}

TEST_F(XTestFixture, write_position_d_gain) {
  ASSERT_FALSE(dxl->write_position_d_gain(comm, 123));
}

TEST_F(XTestFixture, write_position_i_gain) {
  ASSERT_FALSE(dxl->write_position_i_gain(comm, 123));
}

TEST_F(XTestFixture, write_position_p_gain) {
  ASSERT_FALSE(dxl->write_position_p_gain(comm, 123));
}

TEST_F(XTestFixture, write_profile_acceleration) {
  ASSERT_FALSE(dxl->write_profile_acceleration(comm, 0));
}

TEST_F(XTestFixture, write_profile_velocity) {
  ASSERT_FALSE(dxl->write_profile_velocity(comm, 0));
}

TEST_F(XTestFixture, to_profile_acceleration) {
  // rad/s^2 to rev/min^2
  // 0以下に対しては1を返すことを期待
  EXPECT_EQ(dxl->to_profile_acceleration(-1), 1);
  EXPECT_EQ(dxl->to_profile_acceleration(0), 1);
  EXPECT_EQ(dxl->to_profile_acceleration(3.745076), 10);
  EXPECT_EQ(dxl->to_profile_acceleration(1000000), 32767);
}

TEST_F(XTestFixture, to_profile_velocity) {
  // rad/s to rev/min
  // 0以下に対しては1を返すことを期待
  EXPECT_EQ(dxl->to_profile_velocity(-1), 1);
  EXPECT_EQ(dxl->to_profile_velocity(0), 1);
  EXPECT_EQ(dxl->to_profile_velocity(0.239809), 10);
  EXPECT_EQ(dxl->to_profile_velocity(1000000), 32767);
}

TEST_F(XTestFixture, to_position_radian) {
  EXPECT_DOUBLE_EQ(dxl->to_position_radian(2048), 0.0);
  EXPECT_DOUBLE_EQ(dxl->to_position_radian(2048 + 1024), M_PI_2);
  EXPECT_DOUBLE_EQ(dxl->to_position_radian(2048 - 1024), -M_PI_2);
}

TEST_F(XTestFixture, to_velocity_rps) {
  EXPECT_DOUBLE_EQ(dxl->to_velocity_rps(0), 0.0);
  EXPECT_NEAR(dxl->to_velocity_rps(1), 0.0239808, 0.0001);
  EXPECT_NEAR(dxl->to_velocity_rps(-1), -0.0239808, 0.0001);
}

TEST_F(XTestFixture, to_current_ampere) {
  EXPECT_DOUBLE_EQ(dxl->to_current_ampere(0), 0.0);
  EXPECT_DOUBLE_EQ(dxl->to_current_ampere(1000), 2.69);
  EXPECT_DOUBLE_EQ(dxl->to_current_ampere(-1000), -2.69);
}

TEST_F(XTestFixture, to_voltage_volt) {
  EXPECT_DOUBLE_EQ(dxl->to_voltage_volt(0), 0.0);
  EXPECT_DOUBLE_EQ(dxl->to_voltage_volt(1), 0.1);
  EXPECT_DOUBLE_EQ(dxl->to_voltage_volt(2), 0.2);
}

TEST_F(XTestFixture, from_position_radian) {
  EXPECT_EQ(dxl->from_position_radian(0.0), 2048);
  EXPECT_EQ(dxl->from_position_radian(M_PI_2), 2048 + 1024);
  EXPECT_EQ(dxl->from_position_radian(-M_PI_2), 2048 - 1024);
}

TEST_F(XTestFixture, from_velocity_rps) {
  EXPECT_EQ(dxl->from_velocity_rps(0.0), 0);
  EXPECT_EQ(dxl->from_velocity_rps(0.239809), 10);
  EXPECT_EQ(dxl->from_velocity_rps(-0.239809), 0xFFFFFFF6);  // -10
}

TEST_F(XTestFixture, from_current_ampere) {
  EXPECT_EQ(dxl->from_current_ampere(0.0), 0);
  EXPECT_EQ(dxl->from_current_ampere(2.691), 1000);
  EXPECT_EQ(dxl->from_current_ampere(-2.691), 0xFFFFFC18);  // -1000
}

TEST_F(XTestFixture, set_indirect_addresses_read) {
  EXPECT_EQ(dxl->start_address_for_indirect_read(), 634);
  EXPECT_EQ(dxl->length_of_indirect_data_read(), 0);
  EXPECT_EQ(dxl->next_indirect_addr_read(), 578);

  EXPECT_FALSE(dxl->auto_set_indirect_address_of_present_position(comm));
  // indirect_dataの開始位置は変わらないことを期待
  EXPECT_EQ(dxl->start_address_for_indirect_read(), 634);
  EXPECT_EQ(dxl->length_of_indirect_data_read(), 4);
  EXPECT_EQ(dxl->next_indirect_addr_read(), 586);
  EXPECT_EQ(dxl->indirect_addr_of_present_position(), 634);

  EXPECT_FALSE(dxl->auto_set_indirect_address_of_present_velocity(comm));
  EXPECT_EQ(dxl->length_of_indirect_data_read(), 8);
  EXPECT_EQ(dxl->next_indirect_addr_read(), 594);
  EXPECT_EQ(dxl->indirect_addr_of_present_velocity(), 638);

  EXPECT_FALSE(dxl->auto_set_indirect_address_of_present_current(comm));
  EXPECT_EQ(dxl->length_of_indirect_data_read(), 10);
  EXPECT_EQ(dxl->next_indirect_addr_read(), 598);
  EXPECT_EQ(dxl->indirect_addr_of_present_current(), 642);

  EXPECT_FALSE(dxl->auto_set_indirect_address_of_present_input_voltage(comm));
  EXPECT_EQ(dxl->length_of_indirect_data_read(), 12);
  EXPECT_EQ(dxl->next_indirect_addr_read(), 602);
  EXPECT_EQ(dxl->indirect_addr_of_present_input_voltage(), 644);

  EXPECT_FALSE(dxl->auto_set_indirect_address_of_present_temperature(comm));
  EXPECT_EQ(dxl->length_of_indirect_data_read(), 13);
  EXPECT_EQ(dxl->next_indirect_addr_read(), 604);
  EXPECT_EQ(dxl->indirect_addr_of_present_temperature(), 646);
}

TEST_F(XTestFixture, set_indirect_addresses_write) {
  EXPECT_EQ(dxl->start_address_for_indirect_write(), 649);
  EXPECT_EQ(dxl->length_of_indirect_data_write(), 0);
  EXPECT_EQ(dxl->next_indirect_addr_write(), 608);

  EXPECT_FALSE(dxl->auto_set_indirect_address_of_goal_position(comm));
  // indirect_dataの開始位置は変わらないことを期待
  EXPECT_EQ(dxl->start_address_for_indirect_write(), 649);
  EXPECT_EQ(dxl->length_of_indirect_data_write(), 4);
  EXPECT_EQ(dxl->next_indirect_addr_write(), 616);
  EXPECT_EQ(dxl->indirect_addr_of_goal_position(), 649);

  EXPECT_FALSE(dxl->auto_set_indirect_address_of_goal_velocity(comm));
  EXPECT_EQ(dxl->length_of_indirect_data_write(), 8);
  EXPECT_EQ(dxl->next_indirect_addr_write(), 624);
  EXPECT_EQ(dxl->indirect_addr_of_goal_velocity(), 653);

  EXPECT_FALSE(dxl->auto_set_indirect_address_of_goal_current(comm));
  EXPECT_EQ(dxl->length_of_indirect_data_write(), 10);
  EXPECT_EQ(dxl->next_indirect_addr_write(), 628);
  EXPECT_EQ(dxl->indirect_addr_of_goal_current(), 657);
}

TEST_F(XTestFixture, extract_present_position_from_sync_read) {
  std::string group_name = "test";
  double position;
  ASSERT_FALSE(dxl->extract_present_position_from_sync_read(comm, group_name, position));
}

TEST_F(XTestFixture, extract_present_velocity_from_sync_read) {
  std::string group_name = "test";
  double velocity;
  ASSERT_FALSE(dxl->extract_present_velocity_from_sync_read(comm, group_name, velocity));
}

TEST_F(XTestFixture, extract_present_current_from_sync_read) {
  std::string group_name = "test";
  double current;
  ASSERT_FALSE(dxl->extract_present_current_from_sync_read(comm, group_name, current));
}

TEST_F(XTestFixture, extract_present_input_voltage_from_sync_read) {
  std::string group_name = "test";
  double voltage;
  ASSERT_FALSE(dxl->extract_present_input_voltage_from_sync_read(comm, group_name, voltage));
}

TEST_F(XTestFixture, extract_present_temperature_from_sync_read) {
  std::string group_name = "test";
  int temp;
  ASSERT_FALSE(dxl->extract_present_temperature_from_sync_read(comm, group_name, temp));
}

TEST_F(XTestFixture, push_back_position_for_sync_write) {
  std::vector<uint8_t> test_data;
  dxl->push_back_position_for_sync_write(M_PI_2, test_data);
  ASSERT_EQ(test_data.size(), 4);
  // from_position_radian(M_PI_2) = 2048 + 1024 (0x0000 0C00)がセットされる
  EXPECT_EQ(test_data.at(0), 0x00);
  EXPECT_EQ(test_data.at(1), 0x0C);
  EXPECT_EQ(test_data.at(2), 0x00);
  EXPECT_EQ(test_data.at(3), 0x00);

  test_data.clear();
  dxl->push_back_position_for_sync_write(-M_PI_2, test_data);
  ASSERT_EQ(test_data.size(), 4);
  // from_position_radian(-M_PI_2) = 2048 - 1024 (0x0000 0400)がセットされる
  EXPECT_EQ(test_data.at(0), 0x00);
  EXPECT_EQ(test_data.at(1), 0x04);
  EXPECT_EQ(test_data.at(2), 0x00);
  EXPECT_EQ(test_data.at(3), 0x00);
}

TEST_F(XTestFixture, push_back_velocity_for_sync_write) {
  std::vector<uint8_t> test_data;
  dxl->push_back_velocity_for_sync_write(0.239809, test_data);
  ASSERT_EQ(test_data.size(), 4);
  // from_velocity_rps(0.239809) = 10 (0x0000 000A)がセットされる
  EXPECT_EQ(test_data.at(0), 0x0A);
  EXPECT_EQ(test_data.at(1), 0x00);
  EXPECT_EQ(test_data.at(2), 0x00);
  EXPECT_EQ(test_data.at(3), 0x00);

  test_data.clear();
  dxl->push_back_velocity_for_sync_write(-0.239809, test_data);
  ASSERT_EQ(test_data.size(), 4);
  // from_velocity_rps(-0.239809) = -10 (0xFFFF FFF6)がセットされる
  EXPECT_EQ(test_data.at(0), 0xF6);
  EXPECT_EQ(test_data.at(1), 0xFF);
  EXPECT_EQ(test_data.at(2), 0xFF);
  EXPECT_EQ(test_data.at(3), 0xFF);
}

TEST_F(XTestFixture, push_back_current_for_sync_write) {
  std::vector<uint8_t> test_data;
  dxl->push_back_current_for_sync_write(2.691, test_data);
  ASSERT_EQ(test_data.size(), 2);
  // from_current_ampere(2.691) = 1000 (0x03E8)がセットされる
  EXPECT_EQ(test_data.at(0), 0xE8);
  EXPECT_EQ(test_data.at(1), 0x03);

  test_data.clear();
  dxl->push_back_current_for_sync_write(-2.691, test_data);
  ASSERT_EQ(test_data.size(), 2);
  // from_current_ampere(-2.691) = -1000 (0xFC18)がセットされる
  EXPECT_EQ(test_data.at(0), 0x18);
  EXPECT_EQ(test_data.at(1), 0xFC);
}
