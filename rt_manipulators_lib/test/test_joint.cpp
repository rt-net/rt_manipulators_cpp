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

#include "gtest/gtest.h"
#include "rt_manipulators_cpp/joint.hpp"


TEST(JointTest, initialize_id) {
  uint8_t id = 0;
  uint8_t operating_mode = 1;
  double min_position_limit = 2.0;
  double max_position_limit = 3.0;
  double current_limit_when_position_exceeds_limit = 3.0;
  joint::Joint test_joint(
    id, operating_mode, max_position_limit, min_position_limit,
    current_limit_when_position_exceeds_limit);
  EXPECT_EQ(test_joint.id(), 0);
}
