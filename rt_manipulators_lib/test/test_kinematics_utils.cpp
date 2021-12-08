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

class KinematicsUtilsFixture: public ::testing::Test {
 protected:
  virtual void SetUp() {
    links = kinematics_utils::parse_link_config_file("../config/test_links.csv");
  }

  virtual void TearDown() {
  }

  std::vector<manipulators_link::Link> links;
};

TEST_F(KinematicsUtilsFixture, load_link_names) {
  EXPECT_EQ(links[1].name, "test_base");
  EXPECT_EQ(links[2].name, "test_link1");
  EXPECT_EQ(links[9].name, "test_handA");
  EXPECT_EQ(links[10].name, "test_handB");
}

TEST_F(KinematicsUtilsFixture, load_link_siblings) {
  EXPECT_EQ(links[1].sibling, 0);
  EXPECT_EQ(links[2].sibling, 0);
  EXPECT_EQ(links[9].sibling, 10);
  EXPECT_EQ(links[10].sibling, 0);
}

TEST_F(KinematicsUtilsFixture, load_link_child) {
  EXPECT_EQ(links[1].child, 2);
  EXPECT_EQ(links[2].child, 3);
  EXPECT_EQ(links[9].child, 0);
  EXPECT_EQ(links[10].child, 0);
}

TEST_F(KinematicsUtilsFixture, load_link_parent) {
  EXPECT_EQ(links[1].parent, 0);
  EXPECT_EQ(links[2].parent, 1);
  EXPECT_EQ(links[9].parent, 8);
  EXPECT_EQ(links[10].parent, 8);
}

TEST_F(KinematicsUtilsFixture, load_link_b) {
  EXPECT_DOUBLE_EQ(links[1].b[0], 0);
  EXPECT_DOUBLE_EQ(links[1].b[1], 0);
  EXPECT_DOUBLE_EQ(links[1].b[2], 0);

  EXPECT_DOUBLE_EQ(links[2].b[0], 0);
  EXPECT_DOUBLE_EQ(links[2].b[1], -0.001);
  EXPECT_DOUBLE_EQ(links[2].b[2], 0.01);

  EXPECT_DOUBLE_EQ(links[9].b[0], -0.01);
  EXPECT_DOUBLE_EQ(links[9].b[1], -0.008);
  EXPECT_DOUBLE_EQ(links[9].b[2], 0.08);

  EXPECT_DOUBLE_EQ(links[10].b[0], 0.01);
  EXPECT_DOUBLE_EQ(links[10].b[1], -0.008);
  EXPECT_DOUBLE_EQ(links[10].b[2], 0.08);
}

TEST_F(KinematicsUtilsFixture, load_link_m) {
  EXPECT_DOUBLE_EQ(links[1].m, 0.0);
  EXPECT_DOUBLE_EQ(links[2].m, 0.1);
  EXPECT_DOUBLE_EQ(links[9].m, 0.8);
  EXPECT_DOUBLE_EQ(links[10].m, 0.8);
}

// TEST_F(KinematicsUtilsFixture, load_link_c) {
//   double x = 0.001;  // meters
//   double y = 0.002;  // meters
//   double z = 0.003;  // meters

//   EXPECT_DOUBLE_EQ(links[1].c[0], x) << "回転軸方向:-";
//   EXPECT_DOUBLE_EQ(links[1].c[1], y) << "回転軸方向:-";
//   EXPECT_DOUBLE_EQ(links[1].c[2], z) << "回転軸方向:-";

//   EXPECT_DOUBLE_EQ(links[2].c[0], x) << "回転軸方向:Z+";
//   EXPECT_DOUBLE_EQ(links[2].c[1], y) << "回転軸方向:Z+";
//   EXPECT_DOUBLE_EQ(links[2].c[2], z) << "回転軸方向:Z+";

//   // Z-は座標系をX軸回りに180 deg回転して表現する
//   EXPECT_DOUBLE_EQ(links[3].c[0], x) << "回転軸方向:Z-";
//   EXPECT_DOUBLE_EQ(links[3].c[1], -y) << "回転軸方向:Z-";
//   EXPECT_DOUBLE_EQ(links[3].c[2], -z) << "回転軸方向:Z-";

//   EXPECT_DOUBLE_EQ(links[4].c[0], x) << "回転軸方向:Y+";
//   EXPECT_DOUBLE_EQ(links[4].c[1], z) << "回転軸方向:Y+";
//   EXPECT_DOUBLE_EQ(links[4].c[2], -y) << "回転軸方向:Y+";

//   EXPECT_DOUBLE_EQ(links[5].c[0], x) << "回転軸方向:Y-";
//   EXPECT_DOUBLE_EQ(links[5].c[1], -z) << "回転軸方向:Y-";
//   EXPECT_DOUBLE_EQ(links[5].c[2], y) << "回転軸方向:Y-";

//   EXPECT_DOUBLE_EQ(links[6].c[0], z) << "回転軸方向:X+";
//   EXPECT_DOUBLE_EQ(links[6].c[1], y) << "回転軸方向:X+";
//   EXPECT_DOUBLE_EQ(links[6].c[2], -x) << "回転軸方向:X+";

//   EXPECT_DOUBLE_EQ(links[7].c[0], -z) << "回転軸方向:X-";
//   EXPECT_DOUBLE_EQ(links[7].c[1], y) << "回転軸方向:X-";
//   EXPECT_DOUBLE_EQ(links[7].c[2], x) << "回転軸方向:X-";
// }

// TEST_F(KinematicsUtilsFixture, load_link_I) {
//   Eigen::Matrix3d expected;
//   expected << 1, 2, 4,
//               2, 3, 5,
//               4, 5, 6;
//   expect_matrix_approximation(links[1].I, expected, "回転軸方向:-");

//   expected << 1, 2, 4,
//               2, 3, 5,
//               4, 5, 6;
//   expect_matrix_approximation(links[2].I, expected, "回転軸方向:Z+");

//   // Z-は座標系をX軸回りに180 deg回転して表現する
//   expected << 1, -2, -4,
//               -2, 3, 5,
//               -4, 5, 6;
//   expect_matrix_approximation(links[3].I, expected, "回転軸方向:Z-");

//   expected << 1, 4, -2,
//               4, 6, -5,
//               -2, -5, 3;
//   expect_matrix_approximation(links[4].I, expected, "回転軸方向:Y+");

//   expected << 1, -4, 2,
//               -4, 6, -5,
//               2, -5, 3;
//   expect_matrix_approximation(links[5].I, expected, "回転軸方向:Y-");

//   expected << 6, 5, -4,
//               5, 3, -2,
//               -4, -2, 1;
//   expect_matrix_approximation(links[6].I, expected, "回転軸方向:X+");

//   expected << 6, -5, -4,
//               -5, 3, 2,
//               -4, 2, 1;
//   expect_matrix_approximation(links[7].I, expected, "回転軸方向:X-");
// }

TEST_F(KinematicsUtilsFixture, load_link_a) {
  Eigen::Vector3d expected;
  expected << 0, 0, 0;
  expect_vector_approximation(links[1].a, expected, "回転軸方向:-");

  expected << 0, 0, 1;
  expect_vector_approximation(links[2].a, expected, "回転軸方向:Z+");

  expected << 0, 0, -1;
  expect_vector_approximation(links[3].a, expected, "回転軸方向:Z-");

  expected << 0, 1, 0;
  expect_vector_approximation(links[4].a, expected, "回転軸方向:Y+");

  expected << 0, -1, 0;
  expect_vector_approximation(links[5].a, expected, "回転軸方向:Y-");

  expected << 1, 0, 0;
  expect_vector_approximation(links[6].a, expected, "回転軸方向:X+");

  expected << -1, 0, 0;
  expect_vector_approximation(links[7].a, expected, "回転軸方向:X-");
}

TEST(KinematicsUtilsFunctions, skew_symmetric_matrix) {
  Eigen::Vector3d vec;
  vec << 0, 0, 0;
  Eigen::Matrix3d actual = kinematics_utils::skew_symmetric_matrix(vec);
  Eigen::Matrix3d expected;
  expected << 0, 0, 0,
              0, 0, 0,
              0, 0, 0;
  expect_matrix_approximation(actual, expected);

  vec << 1, 2, 3;
  actual = kinematics_utils::skew_symmetric_matrix(vec);
  expected << 0, -3, 2,
              3, 0, -1,
              -2, 1, 0;
  expect_matrix_approximation(actual, expected);
}

TEST(KinematicsUtilsFunctions, rodrigues) {
  Eigen::Vector3d vec;
  vec << 1, 0, 0;
  Eigen::Matrix3d actual = kinematics_utils::rodrigues(vec, 0.0);
  Eigen::Matrix3d expected;
  expected << 1, 0, 0,
              0, 1, 0,
              0, 0, 1;
  expect_matrix_approximation(actual, expected);

  // X軸回りの回転
  vec << 1, 0, 0;
  actual = kinematics_utils::rodrigues(vec, M_PI_2);
  expected << 1, 0, 0,
              0, std::cos(M_PI_2), -std::sin(M_PI_2),
              0, std::sin(M_PI_2), std::cos(M_PI_2);
  expect_matrix_approximation(actual, expected);

  // Y軸回りの回転
  vec << 0, 1, 0;
  actual = kinematics_utils::rodrigues(vec, M_PI_2);
  expected << std::cos(M_PI_2), 0, std::sin(M_PI_2),
              0, 1, 0,
              -std::sin(M_PI_2), 0, std::cos(M_PI_2);
  expect_matrix_approximation(actual, expected);

  // Z軸回りの回転
  vec << 0, 0, 1;
  actual = kinematics_utils::rodrigues(vec, M_PI_2);
  expected << std::cos(M_PI_2), -std::sin(M_PI_2), 0,
              std::sin(M_PI_2), std::cos(M_PI_2), 0,
              0, 0, 1;
  expect_matrix_approximation(actual, expected);
}

TEST(KinematicsUtilsFunctions, rotation_to_euler_ZYX) {
  Eigen::Matrix3d rot;
  rot << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;
  Eigen::Vector3d actual = kinematics_utils::rotation_to_euler_ZYX(rot);
  Eigen::Vector3d expected;
  expected << 0, 0, 0;
  expect_vector_approximation(actual, expected);

  // X軸回りの回転
  rot << 1, 0, 0,
         0, std::cos(M_PI_2), -std::sin(M_PI_2),
         0, std::sin(M_PI_2), std::cos(M_PI_2);
  actual = kinematics_utils::rotation_to_euler_ZYX(rot);
  expected << 0, 0, M_PI_2;
  expect_vector_approximation(actual, expected);

  // Y軸回りの回転
  rot << std::cos(M_PI_2), 0, std::sin(M_PI_2),
         0, 1, 0,
         -std::sin(M_PI_2), 0, std::cos(M_PI_2);
  actual = kinematics_utils::rotation_to_euler_ZYX(rot);
  expected << 0, M_PI_2, 0;
  expect_vector_approximation(actual, expected);

  // Z軸回りの回転
  rot << std::cos(M_PI_2), -std::sin(M_PI_2), 0,
         std::sin(M_PI_2), std::cos(M_PI_2), 0,
         0, 0, 1;
  actual = kinematics_utils::rotation_to_euler_ZYX(rot);
  expected <<  M_PI_2, 0, 0;
  expect_vector_approximation(actual, expected);
}


TEST(KinematicsUtilsFunctions, rotation_from_euler) {
  Eigen::Matrix3d actual = kinematics_utils::rotation_from_euler(0, 0, 0);
  Eigen::Matrix3d expected;
  expected << 1, 0, 0,
              0, 1, 0,
              0, 0, 1;
  expect_matrix_approximation(actual, expected);

  // X軸回りの回転
  actual = kinematics_utils::rotation_from_euler(M_PI_2, 0, 0);
  expected << 1, 0, 0,
              0, std::cos(M_PI_2), -std::sin(M_PI_2),
              0, std::sin(M_PI_2), std::cos(M_PI_2);
  expect_matrix_approximation(actual, expected);

  // Y軸回りの回転
  actual = kinematics_utils::rotation_from_euler(0, M_PI_2, 0);
  expected << std::cos(M_PI_2), 0, std::sin(M_PI_2),
              0, 1, 0,
              -std::sin(M_PI_2), 0, std::cos(M_PI_2);
  expect_matrix_approximation(actual, expected);

  // Z軸回りの回転
  actual = kinematics_utils::rotation_from_euler(0, 0, M_PI_2);
  expected << std::cos(M_PI_2), -std::sin(M_PI_2), 0,
              std::sin(M_PI_2), std::cos(M_PI_2), 0,
              0, 0, 1;
  expect_matrix_approximation(actual, expected);
}
