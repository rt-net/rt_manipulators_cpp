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
#include "rt_manipulators_ik.hpp"


class X7KinematicsFixture: public ::testing::Test {
 protected:
  virtual void SetUp() {
    links = kinematics_utils::parse_link_config_file("../../config/crane-x7_links.csv");
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
    TOLERANCE_Q = 0.1 * M_PI / 180.0;
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
    // 関節の可動範囲を制限
    // 腰
    links[2].min_q = -150 * M_PI / 180.0;
    links[2].max_q = 150 * M_PI / 180.0;
    // 首
    links[3].min_q = -90 * M_PI / 180.0;
    links[3].max_q = 90 * M_PI / 180.0;
    links[4].min_q = -90 * M_PI / 180.0;
    links[4].max_q = 90 * M_PI / 180.0;
    // 右腕
    links[5].min_q = -150 * M_PI / 180.0;
    links[5].max_q = 150 * M_PI / 180.0;
    links[6].min_q = -90 * M_PI / 180.0;
    links[6].max_q = 90 * M_PI / 180.0;
    links[7].min_q = -150 * M_PI / 180.0;
    links[7].max_q = 150 * M_PI / 180.0;
    links[8].min_q = 0 * M_PI / 180.0;
    links[8].max_q = 150 * M_PI / 180.0;
    links[9].min_q = -150 * M_PI / 180.0;
    links[9].max_q = 150 * M_PI / 180.0;
    links[10].min_q = -120 * M_PI / 180.0;
    links[10].max_q = 60 * M_PI / 180.0;
    links[11].min_q = -160 * M_PI / 180.0;
    links[11].max_q = 160 * M_PI / 180.0;
    // 左腕
    links[14].min_q = -150 * M_PI / 180.0;
    links[14].max_q = 150 * M_PI / 180.0;
    links[15].min_q = -90 * M_PI / 180.0;
    links[15].max_q = 90 * M_PI / 180.0;
    links[16].min_q = -150 * M_PI / 180.0;
    links[16].max_q = 150 * M_PI / 180.0;
    links[17].min_q = -150 * M_PI / 180.0;
    links[17].max_q = 0 * M_PI / 180.0;
    links[18].min_q = -150 * M_PI / 180.0;
    links[18].max_q = 150 * M_PI / 180.0;
    links[19].min_q = -60 * M_PI / 180.0;
    links[19].max_q = 120 * M_PI / 180.0;
    links[20].min_q = -160 * M_PI / 180.0;
    links[20].max_q = 160 * M_PI / 180.0;

    // 目標角度と判定値との許容誤差を設定
    TOLERANCE_Q = 0.1 * M_PI / 180.0;
  }

  virtual void TearDown() {
  }

  std::vector<manipulators_link::Link> links;
  double TOLERANCE_Q;
};

TEST_F(X7KinematicsFixture, 3dof_inverse_kinematics) {
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

TEST_F(X7KinematicsFixture, 3dof_picking_inverse_kinematics) {
  Eigen::Vector3d target_p;
  Eigen::Matrix3d target_R;
  kinematics_utils::q_list_t q_list;

  // 目標角度が可動範囲外になることを期待
  target_p << 0.0, 0.0, 0.0;
  EXPECT_FALSE(samples03::x7_3dof_picking_inverse_kinematics(links, target_p, q_list));

  target_p << -0.2, 0.0, 0.3;
  EXPECT_FALSE(samples03::x7_3dof_picking_inverse_kinematics(links, target_p, q_list));

  // 解が求まらないことを期待
  target_p << 1.0, 0.0, 1.0;
  EXPECT_FALSE(samples03::x7_3dof_picking_inverse_kinematics(links, target_p, q_list));

  // 解が求まることを期待
  target_p << 0.2, 0.0, 0.0;
  EXPECT_TRUE(samples03::x7_3dof_picking_inverse_kinematics(links, target_p, q_list));
  EXPECT_NEAR(q_list[2], 0.0, TOLERANCE_Q);
  EXPECT_NEAR(q_list[3], -0.422, TOLERANCE_Q);
  EXPECT_NEAR(q_list[5], -2.319, TOLERANCE_Q);
  EXPECT_NEAR(q_list[7], -0.402, TOLERANCE_Q);
  EXPECT_NEAR(q_list[8], 0.0, TOLERANCE_Q);

  target_p << -0.2, 0.2, 0.0;
  EXPECT_TRUE(samples03::x7_3dof_picking_inverse_kinematics(links, target_p, q_list));
  EXPECT_NEAR(q_list[2], 2.356, TOLERANCE_Q);
  EXPECT_NEAR(q_list[3], -0.608, TOLERANCE_Q);
  EXPECT_NEAR(q_list[5], -1.939, TOLERANCE_Q);
  EXPECT_NEAR(q_list[7], -0.594, TOLERANCE_Q);
  EXPECT_NEAR(q_list[8], 2.356, TOLERANCE_Q);

  target_p << -0.2, -0.2, 0.0;
  EXPECT_TRUE(samples03::x7_3dof_picking_inverse_kinematics(links, target_p, q_list));
  EXPECT_NEAR(q_list[2], -2.356, TOLERANCE_Q);
  EXPECT_NEAR(q_list[3], -0.608, TOLERANCE_Q);
  EXPECT_NEAR(q_list[5], -1.939, TOLERANCE_Q);
  EXPECT_NEAR(q_list[7], -0.594, TOLERANCE_Q);
  EXPECT_NEAR(q_list[8], -2.356, TOLERANCE_Q);
}

TEST_F(S17KinematicsFixture, 3dof_right_arm_inverse_kinematics) {
  Eigen::Vector3d target_p;
  Eigen::Matrix3d target_R;
  kinematics_utils::q_list_t q_list;

  // 目標角度が可動範囲外になることを期待
  target_p << 0.0, 0.0, 0.0;
  EXPECT_FALSE(samples03::s17_3dof_right_arm_inverse_kinematics(links, target_p, q_list));

  target_p << 0.2, 0.0, 0.2;
  EXPECT_FALSE(samples03::s17_3dof_right_arm_inverse_kinematics(links, target_p, q_list));

  // 解が求まらないことを期待
  target_p << 1.0, 0.0, 1.0;
  EXPECT_FALSE(samples03::s17_3dof_right_arm_inverse_kinematics(links, target_p, q_list));

  // 解が求まらないことを期待
  target_p << 0.2, -0.4, 0.3;
  EXPECT_FALSE(samples03::s17_3dof_right_arm_inverse_kinematics(links, target_p, q_list));

  // 解が求まることを期待
  target_p << 0.2, -0.2, 0.2;
  EXPECT_TRUE(samples03::s17_3dof_right_arm_inverse_kinematics(links, target_p, q_list));
  EXPECT_NEAR(q_list[5], -0.565, TOLERANCE_Q);
  EXPECT_NEAR(q_list[6], -1.258, TOLERANCE_Q);
  EXPECT_NEAR(q_list[8], 2.079, TOLERANCE_Q);

  target_p << 0.0, -0.3, 0.2;
  EXPECT_TRUE(samples03::s17_3dof_right_arm_inverse_kinematics(links, target_p, q_list));
  EXPECT_NEAR(q_list[5], -1.700, TOLERANCE_Q);
  EXPECT_NEAR(q_list[6], -0.356, TOLERANCE_Q);
  EXPECT_NEAR(q_list[8], 1.988, TOLERANCE_Q);

  target_p << -0.2, -0.2, 0.2;
  EXPECT_TRUE(samples03::s17_3dof_right_arm_inverse_kinematics(links, target_p, q_list));
  EXPECT_NEAR(q_list[5], -1.682, TOLERANCE_Q);
  EXPECT_NEAR(q_list[6], -1.417, TOLERANCE_Q);
  EXPECT_NEAR(q_list[8], 1.540, TOLERANCE_Q);
}

TEST_F(S17KinematicsFixture, 3dof_right_arm_picking_inverse_kinematics) {
  Eigen::Vector3d target_p;
  Eigen::Matrix3d target_R;
  kinematics_utils::q_list_t q_list;

  // 目標角度が可動範囲外になることを期待
  target_p << 0.0, 0.0, 0.0;
  EXPECT_FALSE(samples03::s17_3dof_right_arm_picking_inverse_kinematics(links, target_p, q_list));

  // 解が求まらないことを期待
  target_p << 1.0, 0.0, 1.0;
  EXPECT_FALSE(samples03::s17_3dof_right_arm_picking_inverse_kinematics(links, target_p, q_list));

  // 解が求まることを期待
  target_p << 0.2, -0.2, 0.0;
  EXPECT_TRUE(samples03::s17_3dof_right_arm_picking_inverse_kinematics(links, target_p, q_list));
  EXPECT_NEAR(q_list[5], -0.468, TOLERANCE_Q);
  EXPECT_NEAR(q_list[6], -1.401, TOLERANCE_Q);
  EXPECT_NEAR(q_list[8], 1.641, TOLERANCE_Q);
  EXPECT_NEAR(q_list[9], 0.165, TOLERANCE_Q);
  EXPECT_NEAR(q_list[10], -1.171, TOLERANCE_Q);
  EXPECT_NEAR(q_list[11], 0.013, TOLERANCE_Q);

  target_p << 0.0, -0.3, 0.0;
  EXPECT_TRUE(samples03::s17_3dof_right_arm_picking_inverse_kinematics(links, target_p, q_list));
  EXPECT_NEAR(q_list[5], -1.119, TOLERANCE_Q);
  EXPECT_NEAR(q_list[6], -0.987, TOLERANCE_Q);
  EXPECT_NEAR(q_list[8], 1.559, TOLERANCE_Q);
  EXPECT_NEAR(q_list[9], 0.597, TOLERANCE_Q);
  EXPECT_NEAR(q_list[10], -0.442, TOLERANCE_Q);
  EXPECT_NEAR(q_list[11], -0.015, TOLERANCE_Q);

  target_p << -0.1, -0.2, 0.0;
  EXPECT_TRUE(samples03::s17_3dof_right_arm_picking_inverse_kinematics(links, target_p, q_list));
  EXPECT_NEAR(q_list[5], -1.270, TOLERANCE_Q);
  EXPECT_NEAR(q_list[6], -1.424, TOLERANCE_Q);
  EXPECT_NEAR(q_list[8], 1.493, TOLERANCE_Q);
  EXPECT_NEAR(q_list[9], 0.196, TOLERANCE_Q);
  EXPECT_NEAR(q_list[10], -0.224, TOLERANCE_Q);
  EXPECT_NEAR(q_list[11], -0.051, TOLERANCE_Q);
}
