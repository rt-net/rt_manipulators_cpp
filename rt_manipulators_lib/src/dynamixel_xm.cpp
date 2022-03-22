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
const uint16_t ADDR_CURRENT_LIMIT = 38;
const uint16_t ADDR_MAX_POSITION_LIMIT = 48;
const uint16_t ADDR_MIN_POSITION_LIMIT = 52;
const uint16_t ADDR_TORQUE_ENABLE = 64;
const uint16_t ADDR_VELOCITY_I_GAIN = 76;
const uint16_t ADDR_VELOCITY_P_GAIN = 78;
const uint16_t ADDR_POSITION_D_GAIN = 80;
const uint16_t ADDR_POSITION_I_GAIN = 82;
const uint16_t ADDR_POSITION_P_GAIN = 84;
const uint16_t ADDR_PROFILE_ACCELERATION = 108;
const uint16_t ADDR_PROFILE_VELOCITY = 112;
const uint16_t ADDR_PRESENT_CURRENT = 126;
const uint16_t ADDR_PRESENT_VELOCITY = 128;
const uint16_t ADDR_PRESENT_POSITION = 132;
const uint16_t ADDR_PRESENT_VOLTAGE = 144;
const uint16_t ADDR_PRESENT_TEMPERATURE = 146;
const uint16_t ADDR_INDIRECT_ADDRESS_1 = 168;
const uint16_t ADDR_INDIRECT_DATA_1 = 224;
const uint16_t ADDR_INDIRECT_ADDRESS_29 = 578;
const uint16_t ADDR_INDIRECT_DATA_29 = 634;

const uint16_t ADDR_START_INDIRECT_ADDR_READ = ADDR_INDIRECT_ADDRESS_1;
const uint16_t ADDR_START_INDIRECT_DATA_READ = ADDR_INDIRECT_DATA_1;
const uint16_t ADDR_START_INDIRECT_ADDR_WRITE = ADDR_INDIRECT_ADDRESS_29;
const uint16_t ADDR_START_INDIRECT_DATA_WRITE = ADDR_INDIRECT_DATA_29;

const uint16_t LEN_PRESENT_CURRENT = 2;
const uint16_t LEN_PRESENT_VELOCITY = 4;
const uint16_t LEN_PRESENT_POSITION = 4;
const uint16_t LEN_PRESENT_VOLTAGE = 2;
const uint16_t LEN_PRESENT_TEMPERATURE = 1;
const uint16_t LEN_INDIRECT_ADDRESS = 2;

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
const double TO_RADIANS = (180.0 / 2048.0) * M_PI / 180.0;
const double TO_CURRENT_AMPERE = 0.00269;
const double TO_VOLTAGE_VOLT = 0.1;
const double TO_DXL_POS = 1.0 / TO_RADIANS;
const double TO_DXL_CURRENT = 1.0 / TO_CURRENT_AMPERE;

DynamixelXM::DynamixelXM(const uint8_t id, const int home_position)
  : dynamixel_base::DynamixelBase(id), HOME_POSITION_(home_position),
    total_length_of_indirect_addr_read_(0), indirect_addr_of_present_position_(0),
    indirect_addr_of_present_velocity_(0), indirect_addr_of_present_current_(0),
    indirect_addr_of_present_input_voltage_(0), indirect_addr_of_present_temperature_(0) {
  name_ = "XM";
}

bool DynamixelXM::read_operating_mode(const dynamixel_base::comm_t & comm, uint8_t & mode) {
  return comm->read_byte_data(id_, ADDR_OPERATING_MODE, mode);
}

bool DynamixelXM::write_operating_mode(const dynamixel_base::comm_t & comm, const uint8_t mode) {
  return comm->write_byte_data(id_, ADDR_OPERATING_MODE, mode);
}

bool DynamixelXM::read_current_limit(
  const dynamixel_base::comm_t & comm, double & limit_ampere) {
  uint16_t dxl_current_limit = 0;
  bool retval = comm->read_word_data(id_, ADDR_CURRENT_LIMIT, dxl_current_limit);
  limit_ampere = to_current_ampere(dxl_current_limit);
  return retval;
}

bool DynamixelXM::read_max_position_limit(
  const dynamixel_base::comm_t & comm, double & limit_radian) {
  uint16_t dxl_position_limit = 0;
  bool retval = comm->read_word_data(id_, ADDR_MAX_POSITION_LIMIT, dxl_position_limit);
  limit_radian = to_position_radian(dxl_position_limit);
  return retval;
}

bool DynamixelXM::read_min_position_limit(
  const dynamixel_base::comm_t & comm, double & limit_radian) {
  uint16_t dxl_position_limit = 0;
  bool retval = comm->read_word_data(id_, ADDR_MIN_POSITION_LIMIT, dxl_position_limit);
  limit_radian = to_position_radian(dxl_position_limit);
  return retval;
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

double DynamixelXM::to_position_radian(const int position) {
  return (position - HOME_POSITION_) * TO_RADIANS;
}

double DynamixelXM::to_velocity_rps(const int velocity) {
  return velocity * TO_VELOCITY_RAD_PER_SEC;
}

double DynamixelXM::to_current_ampere(const int current) {
  return current * TO_CURRENT_AMPERE;
}

double DynamixelXM::to_voltage_volt(const int voltage) {
  return voltage * TO_VOLTAGE_VOLT;
}

unsigned int DynamixelXM::from_position_radian(const double position_rad) {
  return position_rad * TO_DXL_POS + HOME_POSITION_;
}

unsigned int DynamixelXM::from_velocity_rps(const double velocity_rps) {
  return velocity_rps * DXL_VELOCITY_FROM_RAD_PER_SEC;
}

unsigned int DynamixelXM::from_current_ampere(const double current_ampere) {
  return current_ampere * TO_DXL_CURRENT;
}

bool DynamixelXM::auto_set_indirect_address_of_present_position(
  const dynamixel_base::comm_t & comm) {
  return set_indirect_address_read(
    comm, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION, indirect_addr_of_present_position_);
}

bool DynamixelXM::auto_set_indirect_address_of_present_velocity(
  const dynamixel_base::comm_t & comm) {
  return set_indirect_address_read(
    comm, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY, indirect_addr_of_present_velocity_);
}

bool DynamixelXM::auto_set_indirect_address_of_present_current(
  const dynamixel_base::comm_t & comm) {
  return set_indirect_address_read(
    comm, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT, indirect_addr_of_present_current_);
}

bool DynamixelXM::auto_set_indirect_address_of_present_input_voltage(
  const dynamixel_base::comm_t & comm) {
  return set_indirect_address_read(
    comm, ADDR_PRESENT_VOLTAGE, LEN_PRESENT_VOLTAGE, indirect_addr_of_present_input_voltage_);
}

bool DynamixelXM::auto_set_indirect_address_of_present_temperature(
  const dynamixel_base::comm_t & comm) {
  return set_indirect_address_read(
    comm, ADDR_PRESENT_TEMPERATURE, LEN_PRESENT_TEMPERATURE, indirect_addr_of_present_temperature_);
}

unsigned int DynamixelXM::indirect_addr_of_present_position(void) {
  return indirect_addr_of_present_position_;
}

unsigned int DynamixelXM::indirect_addr_of_present_velocity(void) {
  return indirect_addr_of_present_velocity_;
}

unsigned int DynamixelXM::indirect_addr_of_present_current(void) {
  return indirect_addr_of_present_current_;
}

unsigned int DynamixelXM::indirect_addr_of_present_input_voltage(void) {
  return indirect_addr_of_present_input_voltage_;
}

unsigned int DynamixelXM::indirect_addr_of_present_temperature(void) {
  return indirect_addr_of_present_temperature_;
}

unsigned int DynamixelXM::start_address_for_indirect_read(void) {
  return ADDR_START_INDIRECT_DATA_READ;
}

unsigned int DynamixelXM::length_of_indirect_data_read(void) {
  return total_length_of_indirect_addr_read_;
}

unsigned int DynamixelXM::next_indirect_addr_read(void) const {
  return ADDR_START_INDIRECT_ADDR_READ +
    LEN_INDIRECT_ADDRESS * total_length_of_indirect_addr_read_;
}

bool DynamixelXM::extract_present_position_from_sync_read(
    const dynamixel_base::comm_t & comm, const std::string & group_name,
    double & position_rad) {
  uint32_t data = 0;
  if (!comm->get_sync_read_data(
    group_name, id_, indirect_addr_of_present_position_, LEN_PRESENT_POSITION, data)) {
    return false;
  }
  position_rad = to_position_radian(static_cast<int32_t>(data));
  return true;
}

bool DynamixelXM::set_indirect_address_read(
    const dynamixel_base::comm_t & comm, const uint16_t addr, const uint16_t len,
    uint16_t & indirect_addr) {
  bool retval = true;
  for (int i = 0; i < len; i++) {
    uint16_t target_indirect_address = next_indirect_addr_read() + LEN_INDIRECT_ADDRESS * i;
    uint16_t target_data_address = addr + i;
    if (!comm->write_word_data(
      id_, target_indirect_address, target_data_address)) {
      retval = false;
    }
  }
  // テストしやすくするため、write_word_dataに失敗しても変数を更新する
  indirect_addr = ADDR_START_INDIRECT_DATA_READ
    + total_length_of_indirect_addr_read_;
  total_length_of_indirect_addr_read_ += len;
  return retval;
}

}  // namespace dynamixel_xm
