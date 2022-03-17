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

#include <cmath>

#include "dynamixel_xm.hpp"


namespace dynamixel_xm {

const uint16_t ADDR_OPERATING_MODE = 11;
const uint16_t ADDR_TORQUE_ENABLE = 64;
const uint16_t ADDR_VELOCITY_I_GAIN = 76;
const uint16_t ADDR_VELOCITY_P_GAIN = 78;
const uint16_t ADDR_POSITION_D_GAIN = 80;
const uint16_t ADDR_POSITION_I_GAIN = 82;
const uint16_t ADDR_POSITION_P_GAIN = 84;
const uint16_t ADDR_PROFILE_ACCELERATION = 108;
const uint16_t ADDR_PROFILE_VELOCITY = 112;

const double TO_ACCELERATION_REV_PER_MM = 214.577;
const double TO_ACCELERATION_TO_RAD_PER_MM = TO_ACCELERATION_REV_PER_MM * 2.0 * M_PI;
const double TO_ACCELERATION_TO_RAD_PER_SS = TO_ACCELERATION_TO_RAD_PER_MM / 3600.0;
const double DXL_ACCELERATION_FROM_RAD_PER_SS = 1.0 / TO_ACCELERATION_TO_RAD_PER_SS;
const int DXL_MAX_ACCELERATION = 32767;
const double TO_VELOCITY_REV_PER_MIN = 0.229;
const double TO_VELOCITY_RAD_PER_MIN = TO_VELOCITY_REV_PER_MIN * 2.0 * M_PI;
const double TO_VELOCITY_RAD_PER_SEC = TO_VELOCITY_RAD_PER_MIN / 60.0;
const double DXL_VELOCITY_FROM_RAD_PER_SEC = 1.0 / TO_VELOCITY_RAD_PER_SEC;
const int DXL_MAX_VELOCITY = 32767;

DynamixelXM::DynamixelXM(const uint8_t id)
  : dynamixel_base::DynamixelBase(id) {
  name_ = "XM";
}

bool DynamixelXM::read_operating_mode(const dynamixel_base::comm_t & comm, uint8_t & mode) {
  return comm->read_byte_data(id_, ADDR_OPERATING_MODE, mode);
}

bool DynamixelXM::write_operating_mode(const dynamixel_base::comm_t & comm, const uint8_t mode) {
  return comm->write_byte_data(id_, ADDR_OPERATING_MODE, mode);
}

bool DynamixelXM::write_torque_enable(const dynamixel_base::comm_t & comm, const bool enable) {
  return comm->write_byte_data(id_, ADDR_TORQUE_ENABLE, enable);
}

bool DynamixelXM::write_velocity_i_gain(
  const dynamixel_base::comm_t & comm, const unsigned int gain) {
  return comm->write_word_data(id_, ADDR_VELOCITY_I_GAIN, static_cast<uint16_t>(gain));
}

bool DynamixelXM::write_velocity_p_gain(
  const dynamixel_base::comm_t & comm, const unsigned int gain) {
  return comm->write_word_data(id_, ADDR_VELOCITY_P_GAIN, static_cast<uint16_t>(gain));
}

bool DynamixelXM::write_position_d_gain(
  const dynamixel_base::comm_t & comm, const unsigned int gain) {
  return comm->write_word_data(id_, ADDR_POSITION_D_GAIN, static_cast<uint16_t>(gain));
}

bool DynamixelXM::write_position_i_gain(
  const dynamixel_base::comm_t & comm, const unsigned int gain) {
  return comm->write_word_data(id_, ADDR_POSITION_I_GAIN, static_cast<uint16_t>(gain));
}

bool DynamixelXM::write_position_p_gain(
  const dynamixel_base::comm_t & comm, const unsigned int gain) {
  return comm->write_word_data(id_, ADDR_POSITION_P_GAIN, static_cast<uint16_t>(gain));
}

bool DynamixelXM::write_profile_acceleration(
  const dynamixel_base::comm_t & comm, const double acceleration_rpss) {
  return comm->write_double_word_data(
    id_,
    ADDR_PROFILE_ACCELERATION,
    static_cast<uint32_t>(to_profile_acceleration(acceleration_rpss)));
}

bool DynamixelXM::write_profile_velocity(
  const dynamixel_base::comm_t & comm, const double velocity_rps) {
  return comm->write_double_word_data(
    id_,
    ADDR_PROFILE_VELOCITY,
    static_cast<uint32_t>(to_profile_velocity(velocity_rps)));
}

unsigned int DynamixelXM::to_profile_acceleration(const double acceleration_rpss) {
  int dxl_acceleration = DXL_ACCELERATION_FROM_RAD_PER_SS * acceleration_rpss;
  if (dxl_acceleration > DXL_MAX_ACCELERATION) {
    dxl_acceleration = DXL_MAX_ACCELERATION;
  } else if (dxl_acceleration <= 0) {
    // XMシリーズでは、'0'が最大加速度を意味する
    // よって、加速度の最小値は'1'である
    dxl_acceleration = 1;
  }

  return static_cast<unsigned int>(dxl_acceleration);
}

unsigned int DynamixelXM::to_profile_velocity(const double velocity_rps) {
  int dxl_velocity = DXL_VELOCITY_FROM_RAD_PER_SEC * velocity_rps;
  if (dxl_velocity > DXL_MAX_VELOCITY) {
    dxl_velocity = DXL_MAX_VELOCITY;
  } else if (dxl_velocity <= 0) {
    // XMシリーズでは、'0'が最大速度を意味する
    // よって、速度の最小値は'1'である
    dxl_velocity = 1;
  }

  return static_cast<unsigned int>(dxl_velocity);
}

}  // namespace dynamixel_xm
