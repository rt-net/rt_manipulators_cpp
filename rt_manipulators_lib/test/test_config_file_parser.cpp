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

#include "gtest/gtest.h"
#include "rt_manipulators_cpp/config_file_parser.hpp"
#include "rt_manipulators_cpp/hardware_joints.hpp"


TEST(ConfigFileParserTest, invalid_file_path) {
  hardware_joints::Joints parsed_joints;
  EXPECT_FALSE(config_file_parser::parse("", parsed_joints));
}
