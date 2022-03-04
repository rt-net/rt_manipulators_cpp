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
  ASSERT_TRUE(config_file_parser::parse("../config/blank.yaml", parsed_joints));
  EXPECT_EQ(parsed_joints.groups().size(), 0);
}

TEST(ConfigFileParserTest, single_joint) {
  // ジョイントグループが正しく生成されていることを期待
  // ジョイントのパラメータが正しく設定されていることを期待
  hardware_joints::Joints parsed_joints;
  ASSERT_TRUE(config_file_parser::parse("../config/single_joint.yaml", parsed_joints));
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
