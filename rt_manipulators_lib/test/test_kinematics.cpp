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

class KinematicsFixture: public ::testing::Test {
 protected:
  virtual void SetUp() {
    links = kinematics_utils::parse_link_config_file("../config/test_links.csv");
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

  links[2].q = M_PI_2;
  kinematics::forward_kinematics(links, 1);
  expected_p << 0.027, -0.001, 0.28;
  expect_vector_approximation(links[8].p, expected_p, "link1 回転軸方向:Z+");
  expected_R << 0, -1, 0,
                1, 0, 0,
                0, 0, 1;
  expect_matrix_approximation(links[8].R, expected_R, "link1 回転軸方向:Z+");
}
