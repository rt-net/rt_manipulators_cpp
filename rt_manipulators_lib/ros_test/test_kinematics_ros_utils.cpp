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

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "gtest/gtest.h"
#include "kinematics_ros_utils.hpp"

class KinematicsROSUtilsFixture: public ::testing::Test {
 protected:
  virtual void SetUp() {
    const auto path = ament_index_cpp::get_package_share_directory("rt_manipulators_cpp") + "/urdf/test_robot.urdf";
    links = kinematics_ros_utils::parse_urdf_file(path);
  }

  virtual void TearDown() {
  }

  kinematics_utils::links_t links;
};

TEST_F(KinematicsROSUtilsFixture, load_link_names) {
  EXPECT_EQ(links[1].name, "base_link");
  EXPECT_EQ(links[2].name, "r_leg_link1");
  EXPECT_EQ(links[3].name, "r_leg_link2");
}
