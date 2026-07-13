// Copyright 2024 RT Corporation
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


#include <cmath>
#include "dynamixel_ph54.hpp"


namespace dynamixel_ph54 {

const double TO_ACCELERATION_REV_PER_MM = 1.0;
const double TO_ACCELERATION_TO_RAD_PER_MM = TO_ACCELERATION_REV_PER_MM * 2.0 * M_PI;
const double TO_ACCELERATION_TO_RAD_PER_SS = TO_ACCELERATION_TO_RAD_PER_MM / 3600.0;
const double DXL_ACCELERATION_FROM_RAD_PER_SS = 1.0 / TO_ACCELERATION_TO_RAD_PER_SS;
const int DXL_MAX_ACCELERATION = 4255632;
const double TO_RADIANS = (180.0 / 501923.0) * M_PI / 180.0;
const double TO_DXL_POS = 1.0 / TO_RADIANS;

DynamixelPH54::DynamixelPH54(const uint8_t id)
  : dynamixel_p::DynamixelP(id) {
  name_ = "PH54";
}

unsigned int DynamixelPH54::to_profile_acceleration(const double acceleration_rpss) {
  int dxl_acceleration = DXL_ACCELERATION_FROM_RAD_PER_SS * acceleration_rpss;
  if (dxl_acceleration > DXL_MAX_ACCELERATION) {
    dxl_acceleration = DXL_MAX_ACCELERATION;
  } else if (dxl_acceleration <= 0) {
    // PHシリーズでは、'0'が最大加速度を意味する
    // よって、加速度の最小値は'1'である
    dxl_acceleration = 1;
  }

  return static_cast<unsigned int>(dxl_acceleration);
}

double DynamixelPH54::to_position_radian(const int position) {
  return (position - HOME_POSITION_) * TO_RADIANS;
}

unsigned int DynamixelPH54::from_position_radian(const double position_rad) {
  return position_rad * TO_DXL_POS + HOME_POSITION_;
}

}  // namespace dynamixel_ph54
