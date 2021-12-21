// Copyright 2021 RT Corporation
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


void expect_matrix_approximation(
  const Eigen::Matrix3d & actual, const Eigen::Matrix3d & expected,
  const std::string & message = "") {
  EXPECT_TRUE(actual.isApprox(expected))
    << message << std::endl
    << "actual:" << std::endl << actual << std::endl
    << "expected:" << std::endl << expected;
}

void expect_vector_approximation(
  const Eigen::Vector3d & actual, const Eigen::Vector3d & expected,
  const std::string & message = "") {
  EXPECT_TRUE(actual.isApprox(expected))
    << message << std::endl
    << "actual:" << std::endl << actual << std::endl
    << "expected:" << std::endl << expected;
}

void expect_FK(
  std::vector<manipulators_link::Link> & links,
  const int & target_link, const double & target_q,
  const Eigen::Vector3d & expected_p, const Eigen::Matrix3d & expected_R,
  const std::string & message = "", const int & expect_link = 8) {
  links[target_link].q = target_q;
  kinematics::forward_kinematics(links, 1);
  expect_vector_approximation(links[expect_link].p, expected_p, message);
  expect_matrix_approximation(links[expect_link].R, expected_R, message);
  links[target_link].q = 0.0;
}

class KinematicsFixture: public ::testing::Test {
 protected:
  virtual void SetUp() {
    links = kinematics_utils::parse_link_config_file("../config/test_links.csv");
  }

  virtual void TearDown() {
  }

  std::vector<manipulators_link::Link> links;
};

class X7KinematicsFixture: public ::testing::Test {
 protected:
  virtual void SetUp() {
    links = kinematics_utils::parse_link_config_file("../config/crane-x7_links.csv");
  }

  virtual void TearDown() {
  }

  std::vector<manipulators_link::Link> links;
};


class S17KinematicsFixture: public ::testing::Test {
 protected:
  virtual void SetUp() {
    links = kinematics_utils::parse_link_config_file("../config/sciurus17_links.csv");
  }

  virtual void TearDown() {
  }

  std::vector<manipulators_link::Link> links;
};

TEST_F(KinematicsFixture, forward_kinematics) {
  // 手先リンクの位置・姿勢を検査する
  kinematics::forward_kinematics(links, 1);
  Eigen::Vector3d expected_p;
  expected_p << 0.0, -0.028, 0.28;
  expect_vector_approximation(links[8].p, expected_p);

  Eigen::Matrix3d expected_R;
  expected_R << 1, 0, 0,
                0, 1, 0,
                0, 0, 1;
  expect_matrix_approximation(links[8].R, expected_R);

  expected_p << 0.027, -0.001, 0.28;
  expected_R << 0, -1, 0,
                1, 0, 0,
                0, 0, 1;
  expect_FK(links, 2, M_PI_2, expected_p, expected_R, "link1 回転軸方向:Z+");

  expected_p << -0.025, -0.003, 0.28;
  expected_R << 0, 1, 0,
                -1, 0, 0,
                0, 0, 1;
  expect_FK(links, 3, M_PI_2, expected_p, expected_R, "link2 回転軸方向:Z-");

  expected_p << 0.22, -0.028, 0.06;
  expected_R << 0, 0, 1,
                0, 1, 0,
                -1, 0, 0;
  expect_FK(links, 4, M_PI_2, expected_p, expected_R, "link3 回転軸方向:Y+");

  expected_p << -0.18, -0.028, 0.10;
  expected_R << 0, 0, -1,
                0, 1, 0,
                1, 0, 0;
  expect_FK(links, 5, M_PI_2, expected_p, expected_R, "link4 回転軸方向:Y-");

  expected_p << 0.0, -0.145, 0.137;
  expected_R << 1, 0, 0,
                0, 0, -1,
                0, 1, 0;
  expect_FK(links, 6, M_PI_2, expected_p, expected_R, "link5 回転軸方向:X+");

  expected_p << 0.0, 0.049, 0.217;
  expected_R << 1, 0, 0,
                0, 0, 1,
                0, -1, 0;
  expect_FK(links, 7, M_PI_2, expected_p, expected_R, "link6 回転軸方向:X-");

  expected_p << 0.0, -0.028, 0.28;
  expected_R << 0, -1, 0,
                1, 0, 0,
                0, 0, 1;
  expect_FK(links, 8, M_PI_2, expected_p, expected_R, "link7 回転軸方向:Z+");
}

TEST_F(X7KinematicsFixture, inverse_kinematics_LM) {
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

  Eigen::Vector3d target_p;
  Eigen::Matrix3d target_R;
  kinematics_utils::q_list_t q_list;

  // 5方向で0.4m(x or y) * 0.4m(z)の対角線を描く
  const int STEPS = 20;
  // 正面方向(手先は正面を向ける)
  target_R = kinematics_utils::rotation_from_euler_ZYX(0, M_PI_2, 0);
  for (int s = 0; s < STEPS; s++) {
    // 左上から右下
    target_p << 0.2,
                0.2 - 0.4 * s / static_cast<double>(STEPS),
                0.4 - 0.4 * s / static_cast<double>(STEPS);
    EXPECT_TRUE(kinematics::inverse_kinematics_LM(
      links, 8, target_p, target_R, q_list)) << "正面：解けなかった位置:" << std::endl << target_p;
  }
  // 左面方向(手先は左面を向ける)
  target_R = kinematics_utils::rotation_from_euler_ZYX(M_PI_2, M_PI_2, 0);
  for (int s = 0; s < STEPS; s++) {
    // 左上から右下
    target_p << 0.2 - 0.4 * s / static_cast<double>(STEPS),
                0.2,
                0.4 - 0.4 * s / static_cast<double>(STEPS);
    EXPECT_TRUE(kinematics::inverse_kinematics_LM(
      links, 8, target_p, target_R, q_list)) << "左面：解けなかった位置:" << std::endl << target_p;
  }
  // 右面方向(手先は右面を向ける)
  target_R = kinematics_utils::rotation_from_euler_ZYX(-M_PI_2, M_PI_2, 0);
  for (int s = 0; s < STEPS; s++) {
    // 左上から右下
    target_p << 0.2 - 0.4 * s / static_cast<double>(STEPS),
                -0.2,
                0.4 - 0.4 * s / static_cast<double>(STEPS);
    EXPECT_TRUE(kinematics::inverse_kinematics_LM(
      links, 8, target_p, target_R, q_list)) << "右面：解けなかった位置:" << std::endl << target_p;
  }
  // 後面方向(手先は後面を向ける)
  // target_R = kinematics_utils::rotation_from_euler_ZYX(0, -M_PI_2, 0);
  // for (int s = 0; s < STEPS; s++) {
  //   // 左上から右下
  //   target_p << -0.2,
  //               0.2 - 0.4 * s / static_cast<double>(STEPS),
  //               0.2 - 0.2 * s / static_cast<double>(STEPS);
  //   EXPECT_TRUE(kinematics::inverse_kinematics_LM(
  //     links, 8, target_p, target_R, q_list)) << "後面：解けなかった位置:"
  //     << std::endl << target_p;
  // }

  // 上面方向(手先は上面を向ける)
  target_R = kinematics_utils::rotation_from_euler_ZYX(0, 0, 0);
  for (int s = 0; s < STEPS; s++) {
    // 左上から右下
    if (s == STEPS / 2) {
      // 特異姿勢を回避
      continue;
    }
    target_p << 0.2 - 0.4 * s / static_cast<double>(STEPS),
                0.2 - 0.4 * s / static_cast<double>(STEPS),
                0.4;
    EXPECT_TRUE(kinematics::inverse_kinematics_LM(
      links, 8, target_p, target_R, q_list)) << "上面：解けなかった位置:" << std::endl << target_p;
  }
}

TEST_F(S17KinematicsFixture, inverse_kinematics_LM) {
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
  links[8].min_q = -150 * M_PI / 180.0;
  links[8].max_q = 0 * M_PI / 180.0;
  links[9].min_q = -150 * M_PI / 180.0;
  links[9].max_q = 150 * M_PI / 180.0;
  links[10].min_q = -60 * M_PI / 180.0;
  links[10].max_q = 120 * M_PI / 180.0;
  links[11].min_q = -160 * M_PI / 180.0;
  links[11].max_q = 160 * M_PI / 180.0;
  // 左腕
  links[14].min_q = -150 * M_PI / 180.0;
  links[14].max_q = 150 * M_PI / 180.0;
  links[15].min_q = -90 * M_PI / 180.0;
  links[15].max_q = 90 * M_PI / 180.0;
  links[16].min_q = -150 * M_PI / 180.0;
  links[16].max_q = 150 * M_PI / 180.0;
  links[17].min_q = 0 * M_PI / 180.0;
  links[17].max_q = 150 * M_PI / 180.0;
  links[18].min_q = -150 * M_PI / 180.0;
  links[18].max_q = 150 * M_PI / 180.0;
  links[19].min_q = -120 * M_PI / 180.0;
  links[19].max_q = 60 * M_PI / 180.0;
  links[20].min_q = -160 * M_PI / 180.0;
  links[20].max_q = 160 * M_PI / 180.0;

  Eigen::Vector3d target_p;
  Eigen::Matrix3d target_R;
  kinematics_utils::q_list_t q_list;
  int right_target_link_id = 11;
  int left_target_link_id = 20;

  // 右腕と左腕、5方向で0.4m(x or y) * 0.4m(z)の対角線を描く
  const int STEPS = 20;
  // 正面方向(手先は正面を向ける)
  for (int s = 0; s < STEPS; s++) {
    // 右腕
    // 左上から右下
    target_p << 0.3,
                0.0 - 0.4 * s / static_cast<double>(STEPS),
                0.4 - 0.4 * s / static_cast<double>(STEPS);
    target_R = kinematics_utils::rotation_from_euler_ZYX(M_PI_2, 0, 0);
    EXPECT_TRUE(kinematics::inverse_kinematics_LM(
      links, right_target_link_id, target_p, target_R, q_list))
      << "右腕: 正面：解けなかった位置:" << std::endl << target_p;

    // 左腕
    // 右上から左下
    target_p << 0.3,
                0.0 + 0.4 * s / static_cast<double>(STEPS),
                0.4 - 0.4 * s / static_cast<double>(STEPS);
    target_R = kinematics_utils::rotation_from_euler_ZYX(-M_PI_2, 0, 0);
    EXPECT_TRUE(kinematics::inverse_kinematics_LM(
      links, left_target_link_id, target_p, target_R, q_list))
      << "左腕: 正面：解けなかった位置:" << std::endl << target_p;
  }

  // 左面方向(左手先は左面を向ける)
  target_R = kinematics_utils::rotation_from_euler_ZYX(0, 0, 0);
  for (int s = 0; s < STEPS; s++) {
    // 左上から右下
    target_p << 0.2 - 0.4 * s / static_cast<double>(STEPS),
                0.4,
                0.4 - 0.4 * s / static_cast<double>(STEPS);
    EXPECT_TRUE(kinematics::inverse_kinematics_LM(
      links, left_target_link_id, target_p, target_R, q_list))
      << "左腕: 左面：解けなかった位置:" << std::endl << target_p;
  }

  // 右面方向(右手先は右面を向ける)
  target_R = kinematics_utils::rotation_from_euler_ZYX(0, 0, 0);
  for (int s = 0; s < STEPS; s++) {
    // 左上から右下
    target_p << 0.2 - 0.4 * s / static_cast<double>(STEPS),
                -0.4,
                0.4 - 0.4 * s / static_cast<double>(STEPS);
    EXPECT_TRUE(kinematics::inverse_kinematics_LM(
      links, right_target_link_id, target_p, target_R, q_list))
      << "右腕: 右面：解けなかった位置:" << std::endl << target_p;
  }

  // 後面方向(手先は後面を向ける)
  for (int s = 0; s < STEPS; s++) {
    // 右腕
    // 左上から右下
    target_p << -0.30,
                0.05 - 0.4 * s / static_cast<double>(STEPS),
                0.4 - 0.4 * s / static_cast<double>(STEPS);
    target_R = kinematics_utils::rotation_from_euler_ZYX(-M_PI_2, 0, 0);
    EXPECT_TRUE(kinematics::inverse_kinematics_LM(
      links, right_target_link_id, target_p, target_R, q_list))
      << "右腕: 後面：解けなかった位置:" << std::endl << target_p;

    // 左腕
    // 右上から左下
    target_p << -0.30,
                -0.05 + 0.4 * s / static_cast<double>(STEPS),
                0.4 - 0.4 * s / static_cast<double>(STEPS);
    target_R = kinematics_utils::rotation_from_euler_ZYX(M_PI_2, 0, 0);
    EXPECT_TRUE(kinematics::inverse_kinematics_LM(
      links, left_target_link_id, target_p, target_R, q_list))
      << "左腕: 後面：解けなかった位置:" << std::endl << target_p;
  }

  // 上面方向(手先は上面を向ける)
  target_R = kinematics_utils::rotation_from_euler_ZYX(0, 0, 0);
  for (int s = 0; s < STEPS; s++) {
    // 右腕
    target_p << 0.4 - 0.4 * s / static_cast<double>(STEPS),
                0.0 - 0.4 * s / static_cast<double>(STEPS),
                0.5;
    target_R = kinematics_utils::rotation_from_euler_ZYX(0, 0, -M_PI_2);
    EXPECT_TRUE(kinematics::inverse_kinematics_LM(
      links, right_target_link_id, target_p, target_R, q_list))
      << "右腕: 上面：解けなかった位置:" << std::endl << target_p;

    // 左腕
    target_p << 0.4 - 0.4 * s / static_cast<double>(STEPS),
                0.0 + 0.4 * s / static_cast<double>(STEPS),
                0.5;
    target_R = kinematics_utils::rotation_from_euler_ZYX(0, 0, M_PI_2);
    EXPECT_TRUE(kinematics::inverse_kinematics_LM(
      links, left_target_link_id, target_p, target_R, q_list))
      << "左腕: 上面：解けなかった位置:" << std::endl << target_p;
  }
}
