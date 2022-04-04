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

#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "rt_manipulators_cpp/kinematics.hpp"
#include "rt_manipulators_cpp/kinematics_utils.hpp"
#include "rt_manipulators_cpp/link.hpp"
#include "rt_manipulators_dynamics.hpp"


class X7KinematicsFixture: public ::testing::Test {
 protected:
  virtual void SetUp() {
    links = kinematics_utils::parse_link_config_file("../../config/crane-x7_links.csv");
    kinematics::forward_kinematics(links, 1);

    // 目標値と判定値との許容誤差を設定
    TOLERANCE_Q = 0.001;
  }

  virtual void TearDown() {
  }

  std::vector<manipulators_link::Link> links;
  double TOLERANCE_Q;
};

class S17KinematicsFixture: public ::testing::Test {
 protected:
  virtual void SetUp() {
    links = kinematics_utils::parse_link_config_file("../../config/sciurus17_links.csv");
    kinematics::forward_kinematics(links, 1);
  }

  virtual void TearDown() {
  }

  std::vector<manipulators_link::Link> links;
};

TEST_F(X7KinematicsFixture, x7_gravity_compensation) {
  kinematics_utils::q_list_t q_list;

  samples03_dynamics::torque_to_current_t torque_to_current = {
    {2, 1.0},
    {3, 1.0},
    {4, 1.0},
    {5, 1.0},
    {6, 1.0},
    {7, 1.0},
    {8, 1.0}
  };

  // 垂直姿勢ではトルクが0となることを期待
  kinematics::forward_kinematics(links, 1);
  EXPECT_TRUE(samples03_dynamics::x7_gravity_compensation(links, torque_to_current, q_list));
  EXPECT_EQ(q_list.size(), 7);
  EXPECT_NEAR(q_list[2], 0.0, TOLERANCE_Q);
  EXPECT_NEAR(q_list[3], 0.0, TOLERANCE_Q);
  EXPECT_NEAR(q_list[4], 0.0, TOLERANCE_Q);
  EXPECT_NEAR(q_list[5], 0.0, TOLERANCE_Q);
  EXPECT_NEAR(q_list[6], 0.0, TOLERANCE_Q);
  EXPECT_NEAR(q_list[7], 0.0, TOLERANCE_Q);
  EXPECT_NEAR(q_list[8], 0.0, TOLERANCE_Q);

  // トルク電流比に抜けがあればfalseを返す
  samples03_dynamics::torque_to_current_t invalid_values = {
    {3, 1.0},
    {4, 1.0},
    {5, 1.0},
    {6, 1.0},
    {7, 1.0},
    {8, 1.0}
  };
  EXPECT_FALSE(samples03_dynamics::x7_gravity_compensation(links, invalid_values, q_list));

  // 適当に関節を曲げ、意図した関節にトルクが発生していることを期待
  links[3].q = -M_PI_4;
  links[5].q = -M_PI_4;
  kinematics::forward_kinematics(links, 1);
  EXPECT_TRUE(samples03_dynamics::x7_gravity_compensation(links, torque_to_current, q_list));
  EXPECT_NEAR(q_list[2], 0.0, TOLERANCE_Q);
  EXPECT_NEAR(q_list[3], 2.247, TOLERANCE_Q);
  EXPECT_NEAR(q_list[4], 0.0, TOLERANCE_Q);
  EXPECT_NEAR(q_list[5], 0.907, TOLERANCE_Q);
  EXPECT_NEAR(q_list[6], 0.0, TOLERANCE_Q);
  EXPECT_NEAR(q_list[7], 0.023, TOLERANCE_Q);
  EXPECT_NEAR(q_list[8], 0.0, TOLERANCE_Q);

  // 適当に関節を曲げ、意図した関節にトルクが発生していることを期待
  links[3].q = -M_PI_2;
  links[4].q = -M_PI_2;
  links[5].q = -M_PI_2;
  links[7].q = -M_PI_2;
  kinematics::forward_kinematics(links, 1);
  EXPECT_TRUE(samples03_dynamics::x7_gravity_compensation(links, torque_to_current, q_list));
  EXPECT_NEAR(q_list[2], 0.0, TOLERANCE_Q);
  EXPECT_NEAR(q_list[3], 1.872, TOLERANCE_Q);
  EXPECT_NEAR(q_list[4], -0.885, TOLERANCE_Q);
  EXPECT_NEAR(q_list[5], 0.0, TOLERANCE_Q);
  EXPECT_NEAR(q_list[6], -0.023, TOLERANCE_Q);
  EXPECT_NEAR(q_list[7], 0.0, TOLERANCE_Q);
  EXPECT_NEAR(q_list[8], 0.0, TOLERANCE_Q);
}
