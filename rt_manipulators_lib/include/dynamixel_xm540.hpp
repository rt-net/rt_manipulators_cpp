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

#ifndef RT_MANIPULATORS_LIB_INCLUDE_DYNAMIXEL_XM540_HPP_
#define RT_MANIPULATORS_LIB_INCLUDE_DYNAMIXEL_XM540_HPP_

#include "dynamixel_x.hpp"

namespace dynamixel_xm540 {

class DynamixelXM540 : public dynamixel_x::DynamixelX {
 public:
  explicit DynamixelXM540(const uint8_t id);
};

}  // namespace dynamixel_xm540

#endif  // RT_MANIPULATORS_LIB_INCLUDE_DYNAMIXEL_XM540_HPP_
