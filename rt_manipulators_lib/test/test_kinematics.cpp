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
  // 手先リンクの位置・姿勢を検査する
  kinematics::forward_kinematics(links, 1);
  Eigen::Vector3d target_p;
  Eigen::Matrix3d target_R;
  kinematics_utils::q_list_t q_list;
  target_p << 0.2, 0.0, 0.2;
  target_R = kinematics_utils::rotation_from_euler_ZYX(0, M_PI_2, 0);
  EXPECT_TRUE(kinematics::inverse_kinematics_LM(links, 8, target_p, target_R, q_list));

  // target_p << 0.2, -0.1, 0.2;
  // target_R = kinematics_utils::rotation_from_euler_ZYX(0, M_PI_2, 0);
  // EXPECT_TRUE(kinematics::inverse_kinematics_LM(links, 8, target_p, target_R, q_list));
  // for (const auto & [target_id, q_value] : q_list) {
  //   std::cout << target_id << ":" << q_value << std::endl;
  // }
  // // 目標角度をセットしFKを実行したあと、目標位置・姿勢に到達したかを求める
  // kinematics_utils::set_q_list(links, q_list);
  // kinematics::forward_kinematics(links, 1);
  // expect_matrix_approximation(links[8].R, target_R);
  // expect_vector_approximation(links[8].p, target_p);

  // link2の90 deg回転
  // target_p << 0.027, -0.001, 0.28;
  // target_R << 0, -1, 0,
  //             1, 0, 0,
  //             0, 0, 1;
  // EXPECT_TRUE(kinematics::inverse_kinematics_LM(links, 8, target_p, target_R, q_list));
  // kinematics_utils::set_q_list(links, q_list);
  // kinematics::forward_kinematics(links, 1);
  // kinematics_utils::calc_error_p(links[8].p, target_p);

  // expect_matrix_approximation(links[8].R, target_R);
  // expect_vector_approximation(links[8].p, target_p);

  // expected_p << -0.025, -0.003, 0.28;
  // expected_R << 0, 1, 0,
  //               -1, 0, 0,
  //               0, 0, 1;
  // expect_FK(links, 3, M_PI_2, expected_p, expected_R, "link2 回転軸方向:Z-");

  // expected_p << 0.22, -0.028, 0.06;
  // expected_R << 0, 0, 1,
  //               0, 1, 0,
  //               -1, 0, 0;
  // expect_FK(links, 4, M_PI_2, expected_p, expected_R, "link3 回転軸方向:Y+");

  // expected_p << -0.18, -0.028, 0.10;
  // expected_R << 0, 0, -1,
  //               0, 1, 0,
  //               1, 0, 0;
  // expect_FK(links, 5, M_PI_2, expected_p, expected_R, "link4 回転軸方向:Y-");

  // expected_p << 0.0, -0.145, 0.137;
  // expected_R << 1, 0, 0,
  //               0, 0, -1,
  //               0, 1, 0;
  // expect_FK(links, 6, M_PI_2, expected_p, expected_R, "link5 回転軸方向:X+");

  // expected_p << 0.0, 0.049, 0.217;
  // expected_R << 1, 0, 0,
  //               0, 0, 1,
  //               0, -1, 0;
  // expect_FK(links, 7, M_PI_2, expected_p, expected_R, "link6 回転軸方向:X-");

  // expected_p << 0.0, -0.028, 0.28;
  // expected_R << 0, -1, 0,
  //               1, 0, 0,
  //               0, 0, 1;
  // expect_FK(links, 8, M_PI_2, expected_p, expected_R, "link7 回転軸方向:Z+");
}
