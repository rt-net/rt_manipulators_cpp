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

#ifndef RT_MANIPULATORS_LIB_INCLUDE_DYNAMIXEL_PH42_HPP_
#define RT_MANIPULATORS_LIB_INCLUDE_DYNAMIXEL_PH42_HPP_

#include "dynamixel_p.hpp"

namespace dynamixel_ph42 {

class DynamixelPH42 : public dynamixel_p::DynamixelP {
 public:
  explicit DynamixelPH42(const uint8_t id);
};

}  // namespace dynamixel_ph42

#endif  // RT_MANIPULATORS_LIB_INCLUDE_DYNAMIXEL_PH42_HPP_
