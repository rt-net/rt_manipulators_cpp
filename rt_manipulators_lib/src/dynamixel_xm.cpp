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


#include "dynamixel_xm.hpp"


namespace dynamixel_xm {

const uint16_t ADDR_TORQUE_ENABLE = 64;

DynamixelXM::DynamixelXM(const uint8_t id)
  : dynamixel_base::DynamixelBase(id) {
  name_ = "XM";
}

bool DynamixelXM::write_torque_enable(const dynamixel_base::comm_t & comm, const bool enable) {
  return comm->write_byte_data(id_, ADDR_TORQUE_ENABLE, enable);
}

}  // namespace dynamixel_xm
