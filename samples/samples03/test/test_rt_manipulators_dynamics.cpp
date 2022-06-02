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

    // 目標値と判定値との許容誤差を設定
    TOLERANCE_Q = 0.001;
  }

  virtual void TearDown() {
  }

  std::vector<manipulators_link::Link> links;
  double TOLERANCE_Q;
};

TEST_F(X7KinematicsFixture, gravity_compensation) {
  kinematics_utils::q_list_t q_list;
  kinematics_utils::link_id_t target_id = 8;

  // トルク電流比に抜けがあればfalseを返す
  samples03_dynamics::torque_to_current_t invalid_values = {
    {3, 1.0},
    {4, 1.0},
    {5, 1.0},
    {6, 1.0},
    {7, 1.0},
    {8, 1.0}
  };
  EXPECT_FALSE(samples03_dynamics::gravity_compensation(
    links, target_id, invalid_values, q_list));

  samples03_dynamics::torque_to_current_t torque_to_current = {
    {2, 1.0},
    {3, 1.0},
    {4, 1.0},
    {5, 1.0},
    {6, 1.0},
    {7, 1.0},
    {8, 1.0}
  };

  // 原点姿勢（腕を垂直に延ばした姿勢）
  kinematics::forward_kinematics(links, 1);
  EXPECT_TRUE(samples03_dynamics::gravity_compensation(
    links, target_id, torque_to_current, q_list));
  EXPECT_EQ(q_list.size(), 7);
  EXPECT_NEAR(q_list[2], 0.0, TOLERANCE_Q);
  EXPECT_NEAR(q_list[3], -0.055, TOLERANCE_Q);
  EXPECT_NEAR(q_list[4], 0.0, TOLERANCE_Q);
  EXPECT_NEAR(q_list[5], -0.012, TOLERANCE_Q);
  EXPECT_NEAR(q_list[6], 0.0, TOLERANCE_Q);
  EXPECT_NEAR(q_list[7], 0.008, TOLERANCE_Q);
  EXPECT_NEAR(q_list[8], 0.0, TOLERANCE_Q);

  // 適当に関節を曲げ、意図した関節にトルクが発生していることを期待
  links[3].q = -M_PI_2;
  links[4].q = -M_PI_2;
  links[5].q = -M_PI_2;
  links[7].q = -M_PI_2;
  kinematics::forward_kinematics(links, 1);
  EXPECT_TRUE(samples03_dynamics::gravity_compensation(
    links, target_id, torque_to_current, q_list));
  EXPECT_NEAR(q_list[2], 0.0, TOLERANCE_Q);
  EXPECT_NEAR(q_list[3], 2.218, TOLERANCE_Q);
  EXPECT_NEAR(q_list[4], -1.137, TOLERANCE_Q);
  EXPECT_NEAR(q_list[5], 0.0, TOLERANCE_Q);
  EXPECT_NEAR(q_list[6], -0.042, TOLERANCE_Q);
  EXPECT_NEAR(q_list[7], 0.0, TOLERANCE_Q);
  EXPECT_NEAR(q_list[8], 0.0, TOLERANCE_Q);
}

TEST_F(S17KinematicsFixture, gravity_compensation) {
  kinematics_utils::q_list_t q_list;
  kinematics_utils::link_id_t target_id_right = 11;
  kinematics_utils::link_id_t target_id_left = 20;

  samples03_dynamics::torque_to_current_t torque_to_current_right = {
    {2, 1.0},
    {5, 1.0},
    {6, 1.0},
    {7, 1.0},
    {8, 1.0},
    {9, 1.0},
    {10, 1.0},
    {11, 1.0}
  };

  samples03_dynamics::torque_to_current_t torque_to_current_left = {
    {2, 1.0},
    {14, 1.0},
    {15, 1.0},
    {16, 1.0},
    {17, 1.0},
    {18, 1.0},
    {19, 1.0},
    {20, 1.0}
  };

  // 原点姿勢（腕を左右に延ばした姿勢）
  kinematics::forward_kinematics(links, 1);
  EXPECT_TRUE(samples03_dynamics::gravity_compensation(
    links, target_id_right, torque_to_current_right, q_list));
  EXPECT_EQ(q_list.size(), 8);
  EXPECT_NEAR(q_list[2], 0.0, TOLERANCE_Q);
  EXPECT_NEAR(q_list[5], -0.061, TOLERANCE_Q);
  EXPECT_NEAR(q_list[6], 3.909, TOLERANCE_Q);
  EXPECT_NEAR(q_list[7], -0.060, TOLERANCE_Q);
  EXPECT_NEAR(q_list[8], 0.0, TOLERANCE_Q);
  EXPECT_NEAR(q_list[9], 0.021, TOLERANCE_Q);
  EXPECT_NEAR(q_list[10], 0.0, TOLERANCE_Q);
  EXPECT_NEAR(q_list[11], 0.0, TOLERANCE_Q);
  q_list.clear();

  EXPECT_TRUE(samples03_dynamics::gravity_compensation(
    links, target_id_left, torque_to_current_left, q_list));
  EXPECT_EQ(q_list.size(), 8);
  EXPECT_NEAR(q_list[2], 0.0, TOLERANCE_Q);
  EXPECT_NEAR(q_list[14], 0.061, TOLERANCE_Q);
  EXPECT_NEAR(q_list[15], -3.862, TOLERANCE_Q);
  EXPECT_NEAR(q_list[16], 0.060, TOLERANCE_Q);
  EXPECT_NEAR(q_list[17], 0.0, TOLERANCE_Q);
  EXPECT_NEAR(q_list[18], -0.021, TOLERANCE_Q);
  EXPECT_NEAR(q_list[19], 0.0, TOLERANCE_Q);
  EXPECT_NEAR(q_list[20], 0.0, TOLERANCE_Q);
  q_list.clear();

  // 適当に関節を曲げ、意図した関節にトルクが発生していることを期待
  links[5].q = -M_PI_2;
  kinematics::forward_kinematics(links, 1);
  EXPECT_TRUE(samples03_dynamics::gravity_compensation(
    links, target_id_right, torque_to_current_right, q_list));
  EXPECT_NEAR(q_list[2], 0.0, TOLERANCE_Q);
  EXPECT_NEAR(q_list[5], 0.002, TOLERANCE_Q);
  EXPECT_NEAR(q_list[6], 0.0, TOLERANCE_Q);
  EXPECT_NEAR(q_list[7], 0.003, TOLERANCE_Q);
  EXPECT_NEAR(q_list[8], -1.309, TOLERANCE_Q);
  EXPECT_NEAR(q_list[9], 0.002, TOLERANCE_Q);
  EXPECT_NEAR(q_list[10], -0.071, TOLERANCE_Q);
  EXPECT_NEAR(q_list[11], 0.001, TOLERANCE_Q);

  links[14].q = M_PI_2;
  kinematics::forward_kinematics(links, 1);
  EXPECT_TRUE(samples03_dynamics::gravity_compensation(
    links, target_id_left, torque_to_current_left, q_list));
  EXPECT_NEAR(q_list[2], 0.0, TOLERANCE_Q);
  EXPECT_NEAR(q_list[14], 0.0, TOLERANCE_Q);
  EXPECT_NEAR(q_list[15], 0.0, TOLERANCE_Q);
  EXPECT_NEAR(q_list[16], 0.0, TOLERANCE_Q);
  EXPECT_NEAR(q_list[17], 1.262, TOLERANCE_Q);
  EXPECT_NEAR(q_list[18], 0.001, TOLERANCE_Q);
  EXPECT_NEAR(q_list[19], 0.022, TOLERANCE_Q);
  EXPECT_NEAR(q_list[20], 0.001, TOLERANCE_Q);
}
