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

TEST_F(KinematicsUtilsFixture, load_link_dxl_id) {
  EXPECT_EQ(links[1].dxl_id, 0);
  EXPECT_EQ(links[2].dxl_id, 2);
  EXPECT_EQ(links[3].dxl_id, 3);
  EXPECT_EQ(links[9].dxl_id, 9);
  EXPECT_EQ(links[10].dxl_id, 0);
}

TEST(KinematicsUtilsFunctions, skew_symmetric_matrix_for_cross_product) {
  Eigen::Vector3d vec;
  vec << 0, 0, 0;
  Eigen::Matrix3d actual = kinematics_utils::skew_symmetric_matrix_for_cross_product(vec);
  Eigen::Matrix3d expected;
  expected << 0, 0, 0,
              0, 0, 0,
              0, 0, 0;
  expect_matrix_approximation(actual, expected);

  vec << 1, 2, 3;
  actual = kinematics_utils::skew_symmetric_matrix_for_cross_product(vec);
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
  expected << M_PI_2, 0, 0;
  expect_vector_approximation(actual, expected);

  // Z軸、Y軸周りの回転
  // オイラー角が一意に定まらないため、回転行列に戻し、もとの回転行列と一致するかを判定する
  double z = M_PI_2;
  double y = M_PI;
  Eigen::Matrix3d actual_rot;
  actual_rot << std::cos(z) * std::cos(y), -std::sin(z), std::cos(z) * std::sin(y),
                std::sin(z) * std::cos(y), std::cos(z), std::sin(z) * std::sin(y),
                -std::sin(y), 0, std::cos(y);
  auto euler_zyx = kinematics_utils::rotation_to_euler_ZYX(actual_rot);
  auto expected_R = kinematics_utils::rotation_from_euler_ZYX(
    euler_zyx[0], euler_zyx[1], euler_zyx[2]);
  expect_matrix_approximation(actual_rot, expected_R, "Z軸 Y軸周りの回転");

  // Y軸、X軸周りの回転
  // オイラー角が一意に定まらないため、回転行列に戻し、もとの回転行列と一致するかを判定する
  double x = M_PI_4;
  actual_rot << std::cos(y), std::sin(y) * std::sin(x), std::sin(y) * std::cos(x),
                0, std::cos(x), -std::sin(x),
                -std::sin(y), std::cos(y) * std::sin(x), std::cos(y) * std::cos(x);
  euler_zyx = kinematics_utils::rotation_to_euler_ZYX(actual_rot);
  expected_R = kinematics_utils::rotation_from_euler_ZYX(
    euler_zyx[0], euler_zyx[1], euler_zyx[2]);
  expect_matrix_approximation(actual_rot, expected_R, "Y軸, X軸周りの回転");
}

TEST(KinematicsUtilsFunctions, rotation_from_euler_ZYX) {
  Eigen::Matrix3d actual = kinematics_utils::rotation_from_euler_ZYX(0, 0, 0);
  Eigen::Matrix3d expected;
  expected << 1, 0, 0,
              0, 1, 0,
              0, 0, 1;
  expect_matrix_approximation(actual, expected);

  // X軸回りの回転
  actual = kinematics_utils::rotation_from_euler_ZYX(0, 0, M_PI_2);
  expected << 1, 0, 0,
              0, std::cos(M_PI_2), -std::sin(M_PI_2),
              0, std::sin(M_PI_2), std::cos(M_PI_2);
  expect_matrix_approximation(actual, expected, "X軸周り");

  // Y軸回りの回転
  actual = kinematics_utils::rotation_from_euler_ZYX(0, M_PI_2, 0);
  expected << std::cos(M_PI_2), 0, std::sin(M_PI_2),
              0, 1, 0,
              -std::sin(M_PI_2), 0, std::cos(M_PI_2);
  expect_matrix_approximation(actual, expected, "Y軸周り");

  // Z軸回りの回転
  actual = kinematics_utils::rotation_from_euler_ZYX(M_PI_2, 0, 0);
  expected << std::cos(M_PI_2), -std::sin(M_PI_2), 0,
              std::sin(M_PI_2), std::cos(M_PI_2), 0,
              0, 0, 1;
  expect_matrix_approximation(actual, expected, "Z軸周り");

  // Z軸、Y軸周りの回転
  double z = M_PI_2;
  double y = M_PI;
  actual = kinematics_utils::rotation_from_euler_ZYX(z, y, 0);
  expected << std::cos(z) * std::cos(y), -std::sin(z), std::cos(z) * std::sin(y),
              std::sin(z) * std::cos(y), std::cos(z), std::sin(z) * std::sin(y),
              -std::sin(y), 0, std::cos(y);
  expect_matrix_approximation(actual, expected, "Z軸、Y軸周り");

  // Y軸、X軸周りの回転
  double x = M_PI_4;
  actual = kinematics_utils::rotation_from_euler_ZYX(0, y, x);
  expected << std::cos(y), std::sin(y) * std::sin(x), std::sin(y) * std::cos(x),
              0, std::cos(x), -std::sin(x),
              -std::sin(y), std::cos(y) * std::sin(x), std::cos(y) * std::cos(x);
  expect_matrix_approximation(actual, expected);
}

TEST_F(KinematicsUtilsFixture, find_route) {
  std::vector<unsigned int> actual = kinematics_utils::find_route(links, 2);
  std::vector<unsigned int> expected = {2};
  EXPECT_TRUE(actual == expected);

  actual = kinematics_utils::find_route(links, 3);
  expected = {2, 3};
  EXPECT_TRUE(actual == expected);

  actual = kinematics_utils::find_route(links, 9);
  expected = {2, 3, 4, 5, 6, 7, 8, 9};
  EXPECT_TRUE(actual == expected);

  actual = kinematics_utils::find_route(links, 10);
  expected = {2, 3, 4, 5, 6, 7, 8, 10};
  EXPECT_TRUE(actual == expected);

  // 例外処理
  actual = kinematics_utils::find_route(links, 0);
  expected = {};
  EXPECT_TRUE(actual == expected);

  actual = kinematics_utils::find_route(links, 1);
  expected = {};
  EXPECT_TRUE(actual == expected);

  actual = kinematics_utils::find_route(links, 11);
  expected = {};
  EXPECT_TRUE(actual == expected);
}

TEST_F(KinematicsUtilsFixture, get_q_list) {
  links[2].q = 2.0;
  auto q_list = kinematics_utils::get_q_list(links, {2});
  EXPECT_DOUBLE_EQ(2.0, q_list[2]);
  EXPECT_EQ(1, q_list.size());

  links[3].q = 3.0;
  links[4].q = 4.0;
  links[5].q = 5.0;
  links[6].q = 6.0;
  q_list = kinematics_utils::get_q_list(links, {3, 4, 5, 6});
  EXPECT_DOUBLE_EQ(3.0, q_list[3]);
  EXPECT_DOUBLE_EQ(4.0, q_list[4]);
  EXPECT_DOUBLE_EQ(5.0, q_list[5]);
  EXPECT_DOUBLE_EQ(6.0, q_list[6]);
  EXPECT_EQ(4, q_list.size());

  // 例外処理
  q_list = kinematics_utils::get_q_list(links, {11});
  EXPECT_EQ(0, q_list.size());
}

TEST_F(KinematicsUtilsFixture, set_q_list) {
  kinematics_utils::q_list_t q_list;
  q_list[2] = 2.0;
  EXPECT_TRUE(kinematics_utils::set_q_list(links, q_list));
  EXPECT_DOUBLE_EQ(2.0, links[2].q);

  q_list.clear();
  q_list[3] = 3.0;
  q_list[4] = 4.0;
  q_list[5] = 5.0;
  q_list[6] = 6.0;
  EXPECT_TRUE(kinematics_utils::set_q_list(links, q_list));
  EXPECT_DOUBLE_EQ(3.0, links[3].q);
  EXPECT_DOUBLE_EQ(4.0, links[4].q);
  EXPECT_DOUBLE_EQ(5.0, links[5].q);
  EXPECT_DOUBLE_EQ(6.0, links[6].q);

  // 例外処理
  q_list.clear();
  q_list[11] = 3.0;
  EXPECT_FALSE(kinematics_utils::set_q_list(links, q_list));
}

TEST(KinematicsUtilsFunctions, calc_error_R) {
  Eigen::Matrix3d targetR;
  targetR << 1, 0, 0,
             0, 1, 0,
             0, 0, 1;
  Eigen::Matrix3d currentR;
  currentR << 1, 0, 0,
              0, 1, 0,
              0, 0, 1;
  Eigen::Vector3d actual = kinematics_utils::calc_error_R(targetR, currentR);
  Eigen::Vector3d expected(0, 0, 0);
  expect_vector_approximation(actual, expected);

  // X軸回りの回転
  currentR << 1, 0, 0,
              0, std::cos(-M_PI_2), -std::sin(-M_PI_2),
              0, std::sin(-M_PI_2), std::cos(-M_PI_2);
  targetR << 1, 0, 0,
             0, std::cos(M_PI_2), -std::sin(M_PI_2),
             0, std::sin(M_PI_2), std::cos(M_PI_2);
  actual = kinematics_utils::calc_error_R(targetR, currentR);
  expected << M_PI, 0, 0;
  expect_vector_approximation(actual, expected);
  // 反対方向
  actual = kinematics_utils::calc_error_R(currentR, targetR);
  expected << -M_PI, 0, 0;
  expect_vector_approximation(actual, expected);

  // Y軸回りの回転
  currentR << std::cos(-M_PI_2), 0, std::sin(-M_PI_2),
              0, 1, 0,
              -std::sin(-M_PI_2), 0, std::cos(-M_PI_2);
  targetR << std::cos(M_PI_2), 0, std::sin(M_PI_2),
              0, 1, 0,
              -std::sin(M_PI_2), 0, std::cos(M_PI_2);
  actual = kinematics_utils::calc_error_R(targetR, currentR);
  expected << 0, M_PI, 0;
  expect_vector_approximation(actual, expected);
  // 反対方向
  actual = kinematics_utils::calc_error_R(currentR, targetR);
  expected << 0, -M_PI, 0;
  expect_vector_approximation(actual, expected);

  // Z軸回りの回転
  currentR << std::cos(-M_PI_2), -std::sin(-M_PI_2), 0,
             std::sin(-M_PI_2), std::cos(-M_PI_2), 0,
             0, 0, 1;
  targetR << std::cos(M_PI_2), -std::sin(M_PI_2), 0,
             std::sin(M_PI_2), std::cos(M_PI_2), 0,
             0, 0, 1;
  actual = kinematics_utils::calc_error_R(targetR, currentR);
  expected << 0, 0, M_PI;
  expect_vector_approximation(actual, expected);
  // 反対方向
  actual = kinematics_utils::calc_error_R(currentR, targetR);
  expected << 0, 0, -M_PI;
  expect_vector_approximation(actual, expected);

  // Z軸回りの回転と、Z軸、Y軸周りの回転
  double z = M_PI_2;
  double y = M_PI;
  currentR << std::cos(z), -std::sin(z), 0,
              std::sin(z), std::cos(z), 0,
              0, 0, 1;
  targetR << std::cos(z) * std::cos(y), -std::sin(z), std::cos(z) * std::sin(y),
             std::sin(z) * std::cos(y), std::cos(z), std::sin(z) * std::sin(y),
             -std::sin(y), 0, std::cos(y);
  actual = kinematics_utils::calc_error_R(targetR, currentR);
  expected << 0, y, 0;
  expect_vector_approximation(actual, expected);
  // 反対方向
  actual = kinematics_utils::calc_error_R(currentR, targetR);
  expected << 0, -y, 0;
  expect_vector_approximation(actual, expected);

  // Y軸周りの回転と、Y軸、X軸周りの回転
  double x = M_PI_4;
  currentR << std::cos(y), 0, std::sin(y),
              0, 1, 0,
              -std::sin(y), 0, std::cos(y);
  targetR << std::cos(y), std::sin(y) * std::sin(x), std::sin(y) * std::cos(x),
             0, std::cos(x), -std::sin(x),
             -std::sin(y), std::cos(y) * std::sin(x), std::cos(y) * std::cos(x);
  actual = kinematics_utils::calc_error_R(targetR, currentR);
  expected << x, 0, 0;
  expect_vector_approximation(actual, expected);
  // 反対方向
  actual = kinematics_utils::calc_error_R(currentR, targetR);
  expected << -x, 0, 0;
  expect_vector_approximation(actual, expected);
}

TEST(KinematicsUtilsFunctions, calc_error_p) {
  Eigen::Vector3d target_p(0, 0, 0);
  Eigen::Vector3d current_p(0, 0, 0);
  Eigen::Vector3d actual = kinematics_utils::calc_error_p(target_p, current_p);
  Eigen::Vector3d expected(0, 0, 0);
  expect_vector_approximation(actual, expected);

  target_p << 1, 2, 3;
  current_p << -1, -2, -3;
  actual = kinematics_utils::calc_error_p(target_p, current_p);
  expected << 2, 4, 6;
  expect_vector_approximation(actual, expected);
  // 反対方向
  actual = kinematics_utils::calc_error_p(current_p, target_p);
  expected << -2, -4, -6;
  expect_vector_approximation(actual, expected);
}
