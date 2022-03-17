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

#ifndef RT_MANIPULATORS_LIB_INCLUDE_DYNAMIXEL_XM_HPP_
#define RT_MANIPULATORS_LIB_INCLUDE_DYNAMIXEL_XM_HPP_

#include "dynamixel_base.hpp"

namespace dynamixel_xm {

class DynamixelXM : public dynamixel_base::DynamixelBase  {
 public:
  explicit DynamixelXM(const uint8_t id);
  bool write_torque_enable(const dynamixel_base::comm_t & comm, const bool enable);
  bool write_position_p_gain(const dynamixel_base::comm_t & comm, const unsigned int gain);
  bool write_position_i_gain(const dynamixel_base::comm_t & comm, const unsigned int gain);
  bool write_position_d_gain(const dynamixel_base::comm_t & comm, const unsigned int gain);
};

}  // namespace dynamixel_xm

#endif  // RT_MANIPULATORS_LIB_INCLUDE_DYNAMIXEL_XM_HPP_
