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

#include <vector>

#include "gtest/gtest.h"
#include "rt_manipulators_cpp/kinematics_utils.hpp"
#include "rt_manipulators_cpp/link.hpp"

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


