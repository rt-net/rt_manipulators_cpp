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

#include "gtest/gtest.h"
#include "rt_manipulators_cpp/config_file_parser.hpp"
#include "rt_manipulators_cpp/hardware_joints.hpp"


TEST(ConfigFileParserTest, invalid_file_path) {
  hardware_joints::Joints parsed_joints;
  EXPECT_FALSE(config_file_parser::parse("", parsed_joints));
}

TEST(ConfigFileParserTest, blank_config_file) {
  hardware_joints::Joints parsed_joints;
  ASSERT_TRUE(config_file_parser::parse("../config/ok_blank.yaml", parsed_joints));
  EXPECT_EQ(parsed_joints.groups().size(), 0);
}

TEST(ConfigFileParserTest, single_joint) {
  // ジョイントグループが正しく生成されていることを期待
  // ジョイントのパラメータが正しく設定されていることを期待
  hardware_joints::Joints parsed_joints;
  ASSERT_TRUE(config_file_parser::parse("../config/ok_single_joint.yaml", parsed_joints));
  EXPECT_EQ(parsed_joints.groups().size(), 1);
  EXPECT_TRUE(parsed_joints.group("test_group")->sync_read_position_enabled());
  EXPECT_TRUE(parsed_joints.group("test_group")->sync_read_velocity_enabled());
  EXPECT_TRUE(parsed_joints.group("test_group")->sync_read_current_enabled());
  EXPECT_TRUE(parsed_joints.group("test_group")->sync_read_voltage_enabled());
  EXPECT_TRUE(parsed_joints.group("test_group")->sync_read_temperature_enabled());
  EXPECT_TRUE(parsed_joints.group("test_group")->sync_write_position_enabled());
  EXPECT_FALSE(parsed_joints.group("test_group")->sync_write_velocity_enabled());
  EXPECT_FALSE(parsed_joints.group("test_group")->sync_write_current_enabled());
  EXPECT_EQ(parsed_joints.group("test_group")->joint_names().size(), 1);

  EXPECT_EQ(parsed_joints.joint("joint1")->id(), 0);
  EXPECT_EQ(parsed_joints.joint("joint1")->operating_mode(), 3);
}

TEST(ConfigFileParserTest, two_groups_four_joints) {
  // 複数のグループ、ジョイントが生成されていることを期待
  hardware_joints::Joints parsed_joints;
  ASSERT_TRUE(config_file_parser::parse("../config/ok_two_groups_four_joints.yaml", parsed_joints));
  EXPECT_EQ(parsed_joints.groups().size(), 2);
  EXPECT_TRUE(parsed_joints.has_group("test_group1"));
  EXPECT_TRUE(parsed_joints.has_group("test_group2"));
  EXPECT_EQ(parsed_joints.group("test_group1")->joint_names().size(), 2);
  EXPECT_EQ(parsed_joints.group("test_group2")->joint_names().size(), 2);
  EXPECT_EQ(parsed_joints.joint("joint1")->id(), 0);
  EXPECT_EQ(parsed_joints.joint("joint2")->id(), 1);
  EXPECT_EQ(parsed_joints.joint("joint3")->id(), 2);
  EXPECT_EQ(parsed_joints.joint("joint4")->id(), 3);
}

TEST(ConfigFileParserTest, has_dynamixel_name) {
  // ジョイントにDynamixelの情報がセットされていることを期待
  hardware_joints::Joints parsed_joints;
  ASSERT_TRUE(config_file_parser::parse("../config/ok_has_dynamixel_name.yaml", parsed_joints));
  EXPECT_EQ(parsed_joints.joint("joint1")->dxl->get_id(), 1);
  EXPECT_EQ(parsed_joints.joint("joint1")->dxl->get_name(), "XM430");
  EXPECT_EQ(parsed_joints.joint("joint2")->dxl->get_name(), "XM540");
  EXPECT_EQ(parsed_joints.joint("joint3")->dxl->get_name(), "XH430");
  EXPECT_EQ(parsed_joints.joint("joint4")->dxl->get_name(), "XH540");
  EXPECT_EQ(parsed_joints.joint("joint5")->dxl->get_name(), "PH42");
}

TEST(ConfigFileParserTest, has_same_groups) {
  // 同じグループ名が２つあると、失敗することを期待
  hardware_joints::Joints parsed_joints;
  ASSERT_FALSE(config_file_parser::parse("../config/ng_has_same_groups.yaml", parsed_joints));
}

TEST(ConfigFileParserTest, no_joints) {
  // グループにjointsが設定されていないと、失敗することを期待
  hardware_joints::Joints parsed_joints;
  ASSERT_FALSE(config_file_parser::parse("../config/ng_no_joints.yaml", parsed_joints));
}

TEST(ConfigFileParserTest, has_same_joints) {
  // グループに同じ名前のジョイントがセットされていると、失敗することを期待
  hardware_joints::Joints parsed_joints;
  ASSERT_FALSE(config_file_parser::parse("../config/ng_has_same_joints.yaml", parsed_joints));
}

TEST(ConfigFileParserTest, no_joint_config) {
  // グループにセットされたジョイントの設定項目が無いと、失敗することを期待
  hardware_joints::Joints parsed_joints;
  ASSERT_FALSE(config_file_parser::parse("../config/ng_no_joint_config.yaml", parsed_joints));
}

TEST(ConfigFileParserTest, no_joint_id) {
  // ジョイントのIDが設定されていないと、失敗することを期待
  hardware_joints::Joints parsed_joints;
  ASSERT_FALSE(config_file_parser::parse("../config/ng_no_joint_id.yaml", parsed_joints));
}

TEST(ConfigFileParserTest, no_joint_operating_mode) {
  // ジョイントのoperating modeが設定されていないと、失敗することを期待
  hardware_joints::Joints parsed_joints;
  ASSERT_FALSE(config_file_parser::parse("../config/ng_no_joint_operating_mode.yaml",
    parsed_joints));
}

TEST(ConfigFileParserTest, write_velocity_without_reading_position) {
  // グループのsync_writeにvelocityがセットされているが、
  // sync_readにpositionがセットされていないと、失敗することを期待
  hardware_joints::Joints parsed_joints;
  ASSERT_FALSE(config_file_parser::parse(
    "../config/ng_write_velocity_without_reading_position.yaml",
    parsed_joints));
}

TEST(ConfigFileParserTest, write_current_without_reading_position) {
  // グループのsync_writeにcurrentがセットされているが、
  // sync_readにpositionがセットされていないと、失敗することを期待
  hardware_joints::Joints parsed_joints;
  ASSERT_FALSE(config_file_parser::parse(
    "../config/ng_write_current_without_reading_position.yaml",
    parsed_joints));
}
