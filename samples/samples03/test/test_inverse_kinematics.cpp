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
#include "inverse_kinematics.hpp"


class X7KinematicsFixture: public ::testing::Test {
 protected:
  virtual void SetUp() {
    links = kinematics_utils::parse_link_config_file("../../config/crane-x7_links.csv");
  }

  virtual void TearDown() {
  }

  std::vector<manipulators_link::Link> links;
};

TEST_F(X7KinematicsFixture, 3dof_inverse_kinematics) {
  kinematics::forward_kinematics(links, 1);
  // 関節の可動範囲を制限
  links[2].min_q = -150 * M_PI / 180.0;
  links[2].max_q = 150 * M_PI / 180.0;
  links[3].min_q = -90 * M_PI / 180.0;
  links[3].max_q = 90 * M_PI / 180.0;
  links[4].min_q = -150 * M_PI / 180.0;
  links[4].max_q = 150 * M_PI / 180.0;
  links[5].min_q = -160 * M_PI / 180.0;
  links[5].max_q = 0 * M_PI / 180.0;
  links[6].min_q = -150 * M_PI / 180.0;
  links[6].max_q = 150 * M_PI / 180.0;
  links[7].min_q = -90 * M_PI / 180.0;
  links[7].max_q = 90 * M_PI / 180.0;
  links[8].min_q = -160 * M_PI / 180.0;
  links[8].max_q = 160 * M_PI / 180.0;

  // 目標角度と判定値との許容誤差を設定
  const double TOLERANCE_Q = 0.1 * M_PI / 180.0;

  Eigen::Vector3d target_p;
  Eigen::Matrix3d target_R;
  kinematics_utils::q_list_t q_list;

  // 目標角度が可動範囲外になることを期待
  target_p << 0.0, 0.0, 0.0;
  EXPECT_FALSE(samples03::x7_3dof_inverse_kinematics(links, target_p, q_list));

  target_p << -0.2, 0.0, 0.3;
  EXPECT_FALSE(samples03::x7_3dof_inverse_kinematics(links, target_p, q_list));

  // 解が求まらないことを期待
  target_p << 1.0, 0.0, 1.0;
  EXPECT_FALSE(samples03::x7_3dof_inverse_kinematics(links, target_p, q_list));

  // 解が求まることを期待
  target_p << 0.2, 0.0, 0.3;
  EXPECT_TRUE(samples03::x7_3dof_inverse_kinematics(links, target_p, q_list));
  EXPECT_NEAR(q_list[2], 0.0, TOLERANCE_Q);
  EXPECT_NEAR(q_list[3], 0.180, TOLERANCE_Q);
  EXPECT_NEAR(q_list[5], -1.956, TOLERANCE_Q);

  target_p << -0.2, 0.2, 0.3;
  EXPECT_TRUE(samples03::x7_3dof_inverse_kinematics(links, target_p, q_list));
  EXPECT_NEAR(q_list[2], 2.356, TOLERANCE_Q);
  EXPECT_NEAR(q_list[3], -0.154, TOLERANCE_Q);
  EXPECT_NEAR(q_list[5], -1.627, TOLERANCE_Q);

  target_p << -0.2, -0.2, 0.3;
  EXPECT_TRUE(samples03::x7_3dof_inverse_kinematics(links, target_p, q_list));
  EXPECT_NEAR(q_list[2], -2.356, TOLERANCE_Q);
  EXPECT_NEAR(q_list[3], -0.154, TOLERANCE_Q);
  EXPECT_NEAR(q_list[5], -1.627, TOLERANCE_Q);
}
