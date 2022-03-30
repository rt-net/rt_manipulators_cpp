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


#include "dynamixel_xh430.hpp"


namespace dynamixel_xh430 {

const double TO_CURRENT_AMPERE = 0.00134;
const double TO_DXL_CURRENT = 1.0 / TO_CURRENT_AMPERE;

DynamixelXH430::DynamixelXH430(const uint8_t id)
  : dynamixel_x::DynamixelX(id) {
  name_ = "XH430";
}

double DynamixelXH430::to_current_ampere(const int current) {
  return current * TO_CURRENT_AMPERE;
}

unsigned int DynamixelXH430::from_current_ampere(const double current_ampere) {
  return current_ampere * TO_DXL_CURRENT;
}

}  // namespace dynamixel_xh430
