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

#ifndef RT_MANIPULATORS_LIB_INCLUDE_DYNAMIXEL_XH430_HPP_
#define RT_MANIPULATORS_LIB_INCLUDE_DYNAMIXEL_XH430_HPP_

#include "dynamixel_x.hpp"

namespace dynamixel_xh430 {

class DynamixelXH430 : public dynamixel_x::DynamixelX {
 public:
  explicit DynamixelXH430(const uint8_t id);
  double to_current_ampere(const int current) override;
  unsigned int from_current_ampere(const double current_ampere) override;
};

}  // namespace dynamixel_xh430

#endif  // RT_MANIPULATORS_LIB_INCLUDE_DYNAMIXEL_XH430_HPP_
