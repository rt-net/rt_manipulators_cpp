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

#include "dynamixel_p.hpp"


namespace dynamixel_p {

const uint16_t ADDR_OPERATING_MODE = 11;
const uint16_t ADDR_CURRENT_LIMIT = 38;
const uint16_t ADDR_MAX_POSITION_LIMIT = 48;
const uint16_t ADDR_MIN_POSITION_LIMIT = 52;
const uint16_t ADDR_TORQUE_ENABLE = 512;
const uint16_t ADDR_VELOCITY_I_GAIN = 524;
const uint16_t ADDR_VELOCITY_P_GAIN = 526;
const uint16_t ADDR_POSITION_D_GAIN = 528;
const uint16_t ADDR_POSITION_I_GAIN = 530;
const uint16_t ADDR_POSITION_P_GAIN = 532;
const uint16_t ADDR_GOAL_CURRENT = 550;
const uint16_t ADDR_GOAL_VELOCITY = 552;
const uint16_t ADDR_PROFILE_ACCELERATION = 556;
const uint16_t ADDR_PROFILE_VELOCITY = 560;
const uint16_t ADDR_GOAL_POSITION = 564;
const uint16_t ADDR_PRESENT_CURRENT = 574;
const uint16_t ADDR_PRESENT_VELOCITY = 576;
const uint16_t ADDR_PRESENT_POSITION = 580;
const uint16_t ADDR_PRESENT_VOLTAGE = 592;
const uint16_t ADDR_PRESENT_TEMPERATURE = 594;
const uint16_t ADDR_INDIRECT_ADDRESS_1 = 168;
const uint16_t ADDR_INDIRECT_DATA_1 = 634;
const uint16_t ADDR_INDIRECT_ADDRESS_16 = 198;
const uint16_t ADDR_INDIRECT_DATA_16 = 649;

// XMシリーズと同じアドレスで通信するため、インダイレクトアドレスの使用範囲を絞る
const uint16_t ADDR_START_INDIRECT_ADDR_READ = ADDR_INDIRECT_ADDRESS_1;
const uint16_t ADDR_START_INDIRECT_DATA_READ = ADDR_INDIRECT_DATA_1;
const uint16_t ADDR_START_INDIRECT_ADDR_WRITE = ADDR_INDIRECT_ADDRESS_16;
const uint16_t ADDR_START_INDIRECT_DATA_WRITE = ADDR_INDIRECT_DATA_16;

const uint16_t LEN_PRESENT_CURRENT = 2;
const uint16_t LEN_PRESENT_VELOCITY = 4;
const uint16_t LEN_PRESENT_POSITION = 4;
const uint16_t LEN_PRESENT_VOLTAGE = 2;
const uint16_t LEN_PRESENT_TEMPERATURE = 1;
const uint16_t LEN_GOAL_CURRENT = 2;
const uint16_t LEN_GOAL_VELOCITY = 4;
const uint16_t LEN_GOAL_POSITION = 4;
const uint16_t LEN_INDIRECT_ADDRESS = 2;

const double TO_ACCELERATION_REV_PER_MM = 1.0;
const double TO_ACCELERATION_TO_RAD_PER_MM = TO_ACCELERATION_REV_PER_MM * 2.0 * M_PI;
const double TO_ACCELERATION_TO_RAD_PER_SS = TO_ACCELERATION_TO_RAD_PER_MM / 3600.0;
const double DXL_ACCELERATION_FROM_RAD_PER_SS = 1.0 / TO_ACCELERATION_TO_RAD_PER_SS;
const int DXL_MAX_ACCELERATION = 4306173;
const double TO_VELOCITY_REV_PER_MIN = 0.01;
const double TO_VELOCITY_RAD_PER_MIN = TO_VELOCITY_REV_PER_MIN * 2.0 * M_PI;
const double TO_VELOCITY_RAD_PER_SEC = TO_VELOCITY_RAD_PER_MIN / 60.0;
const double DXL_VELOCITY_FROM_RAD_PER_SEC = 1.0 / TO_VELOCITY_RAD_PER_SEC;
const int DXL_MAX_VELOCITY = 2920;
const double TO_RADIANS = (180.0 / 303750.0) * M_PI / 180.0;
const double TO_CURRENT_AMPERE = 0.001;
const double TO_VOLTAGE_VOLT = 0.1;
const double TO_DXL_POS = 1.0 / TO_RADIANS;
const double TO_DXL_CURRENT = 1.0 / TO_CURRENT_AMPERE;

DynamixelP::DynamixelP(const uint8_t id, const int home_position)
  : dynamixel_base::DynamixelBase(id), HOME_POSITION_(home_position),
    total_length_of_indirect_addr_read_(0), total_length_of_indirect_addr_write_(0),
    indirect_addr_of_present_position_(0), indirect_addr_of_present_velocity_(0),
    indirect_addr_of_present_current_(0), indirect_addr_of_present_input_voltage_(0),
    indirect_addr_of_present_temperature_(0), indirect_addr_of_goal_position_(0),
    indirect_addr_of_goal_velocity_(0), indirect_addr_of_goal_current_(0) {
  name_ = "P";
}

bool DynamixelP::read_operating_mode(const dynamixel_base::comm_t & comm, uint8_t & mode) {
  return comm->read_byte_data(id_, ADDR_OPERATING_MODE, mode);
}

bool DynamixelP::write_operating_mode(const dynamixel_base::comm_t & comm, const uint8_t mode) {
  return comm->write_byte_data(id_, ADDR_OPERATING_MODE, mode);
}

bool DynamixelP::read_current_limit(
  const dynamixel_base::comm_t & comm, double & limit_ampere) {
  uint16_t dxl_current_limit = 0;
  bool retval = comm->read_word_data(id_, ADDR_CURRENT_LIMIT, dxl_current_limit);
  limit_ampere = to_current_ampere(dxl_current_limit);
  return retval;
}

bool DynamixelP::read_max_position_limit(
  const dynamixel_base::comm_t & comm, double & limit_radian) {
  uint32_t dxl_position_limit = 0;
  bool retval = comm->read_double_word_data(id_, ADDR_MAX_POSITION_LIMIT, dxl_position_limit);
  limit_radian = to_position_radian(dxl_position_limit);
  return retval;
}

bool DynamixelP::read_min_position_limit(
  const dynamixel_base::comm_t & comm, double & limit_radian) {
  uint32_t dxl_position_limit = 0;
  bool retval = comm->read_double_word_data(id_, ADDR_MIN_POSITION_LIMIT, dxl_position_limit);
  limit_radian = to_position_radian(dxl_position_limit);
  return retval;
}

bool DynamixelP::write_torque_enable(const dynamixel_base::comm_t & comm, const bool enable) {
  return comm->write_byte_data(id_, ADDR_TORQUE_ENABLE, enable);
}

bool DynamixelP::write_velocity_i_gain(
  const dynamixel_base::comm_t & comm, const unsigned int gain) {
  return comm->write_word_data(id_, ADDR_VELOCITY_I_GAIN, static_cast<uint16_t>(gain));
}

bool DynamixelP::write_velocity_p_gain(
  const dynamixel_base::comm_t & comm, const unsigned int gain) {
  return comm->write_word_data(id_, ADDR_VELOCITY_P_GAIN, static_cast<uint16_t>(gain));
}

bool DynamixelP::write_position_d_gain(
  const dynamixel_base::comm_t & comm, const unsigned int gain) {
  return comm->write_word_data(id_, ADDR_POSITION_D_GAIN, static_cast<uint16_t>(gain));
}

bool DynamixelP::write_position_i_gain(
  const dynamixel_base::comm_t & comm, const unsigned int gain) {
  return comm->write_word_data(id_, ADDR_POSITION_I_GAIN, static_cast<uint16_t>(gain));
}

bool DynamixelP::write_position_p_gain(
  const dynamixel_base::comm_t & comm, const unsigned int gain) {
  return comm->write_word_data(id_, ADDR_POSITION_P_GAIN, static_cast<uint16_t>(gain));
}

bool DynamixelP::write_profile_acceleration(
  const dynamixel_base::comm_t & comm, const double acceleration_rpss) {
  return comm->write_double_word_data(
    id_,
    ADDR_PROFILE_ACCELERATION,
    static_cast<uint32_t>(to_profile_acceleration(acceleration_rpss)));
}

bool DynamixelP::write_profile_velocity(
  const dynamixel_base::comm_t & comm, const double velocity_rps) {
  return comm->write_double_word_data(
    id_,
    ADDR_PROFILE_VELOCITY,
    static_cast<uint32_t>(to_profile_velocity(velocity_rps)));
}

unsigned int DynamixelP::to_profile_acceleration(const double acceleration_rpss) {
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

unsigned int DynamixelP::to_profile_velocity(const double velocity_rps) {
  int dxl_velocity = DXL_VELOCITY_FROM_RAD_PER_SEC * velocity_rps;
  if (dxl_velocity > DXL_MAX_VELOCITY) {
    dxl_velocity = DXL_MAX_VELOCITY;
  } else if (dxl_velocity <= 0) {
    // PHシリーズでは、'0'が最大速度を意味する
    // よって、速度の最小値は'1'である
    dxl_velocity = 1;
  }

  return static_cast<unsigned int>(dxl_velocity);
}

double DynamixelP::to_position_radian(const int position) {
  return (position - HOME_POSITION_) * TO_RADIANS;
}

double DynamixelP::to_velocity_rps(const int velocity) {
  return velocity * TO_VELOCITY_RAD_PER_SEC;
}

double DynamixelP::to_current_ampere(const int current) {
  return current * TO_CURRENT_AMPERE;
}

double DynamixelP::to_voltage_volt(const int voltage) {
  return voltage * TO_VOLTAGE_VOLT;
}

unsigned int DynamixelP::from_position_radian(const double position_rad) {
  return position_rad * TO_DXL_POS + HOME_POSITION_;
}

unsigned int DynamixelP::from_velocity_rps(const double velocity_rps) {
  return velocity_rps * DXL_VELOCITY_FROM_RAD_PER_SEC;
}

unsigned int DynamixelP::from_current_ampere(const double current_ampere) {
  return current_ampere * TO_DXL_CURRENT;
}

bool DynamixelP::auto_set_indirect_address_of_present_position(
  const dynamixel_base::comm_t & comm) {
  return set_indirect_address_read(
    comm, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION, indirect_addr_of_present_position_);
}

bool DynamixelP::auto_set_indirect_address_of_present_velocity(
  const dynamixel_base::comm_t & comm) {
  return set_indirect_address_read(
    comm, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY, indirect_addr_of_present_velocity_);
}

bool DynamixelP::auto_set_indirect_address_of_present_current(
  const dynamixel_base::comm_t & comm) {
  return set_indirect_address_read(
    comm, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT, indirect_addr_of_present_current_);
}

bool DynamixelP::auto_set_indirect_address_of_present_input_voltage(
  const dynamixel_base::comm_t & comm) {
  return set_indirect_address_read(
    comm, ADDR_PRESENT_VOLTAGE, LEN_PRESENT_VOLTAGE, indirect_addr_of_present_input_voltage_);
}

bool DynamixelP::auto_set_indirect_address_of_present_temperature(
  const dynamixel_base::comm_t & comm) {
  return set_indirect_address_read(
    comm, ADDR_PRESENT_TEMPERATURE, LEN_PRESENT_TEMPERATURE, indirect_addr_of_present_temperature_);
}

bool DynamixelP::auto_set_indirect_address_of_goal_position(
  const dynamixel_base::comm_t & comm) {
  return set_indirect_address_write(
    comm, ADDR_GOAL_POSITION, LEN_GOAL_POSITION, indirect_addr_of_goal_position_);
}

bool DynamixelP::auto_set_indirect_address_of_goal_velocity(
  const dynamixel_base::comm_t & comm) {
  return set_indirect_address_write(
    comm, ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY, indirect_addr_of_goal_velocity_);
}

bool DynamixelP::auto_set_indirect_address_of_goal_current(
  const dynamixel_base::comm_t & comm) {
  return set_indirect_address_write(
    comm, ADDR_GOAL_CURRENT, LEN_GOAL_CURRENT, indirect_addr_of_goal_current_);
}

unsigned int DynamixelP::indirect_addr_of_present_position(void) {
  return indirect_addr_of_present_position_;
}

unsigned int DynamixelP::indirect_addr_of_present_velocity(void) {
  return indirect_addr_of_present_velocity_;
}

unsigned int DynamixelP::indirect_addr_of_present_current(void) {
  return indirect_addr_of_present_current_;
}

unsigned int DynamixelP::indirect_addr_of_present_input_voltage(void) {
  return indirect_addr_of_present_input_voltage_;
}

unsigned int DynamixelP::indirect_addr_of_present_temperature(void) {
  return indirect_addr_of_present_temperature_;
}

unsigned int DynamixelP::indirect_addr_of_goal_position(void) {
  return indirect_addr_of_goal_position_;
}

unsigned int DynamixelP::indirect_addr_of_goal_velocity(void) {
  return indirect_addr_of_goal_velocity_;
}

unsigned int DynamixelP::indirect_addr_of_goal_current(void) {
  return indirect_addr_of_goal_current_;
}

unsigned int DynamixelP::start_address_for_indirect_read(void) {
  return ADDR_START_INDIRECT_DATA_READ;
}

unsigned int DynamixelP::length_of_indirect_data_read(void) {
  return total_length_of_indirect_addr_read_;
}

unsigned int DynamixelP::next_indirect_addr_read(void) const {
  return ADDR_START_INDIRECT_ADDR_READ +
    LEN_INDIRECT_ADDRESS * total_length_of_indirect_addr_read_;
}

unsigned int DynamixelP::start_address_for_indirect_write(void) {
  return ADDR_START_INDIRECT_DATA_WRITE;
}

unsigned int DynamixelP::length_of_indirect_data_write(void) {
  return total_length_of_indirect_addr_write_;
}

unsigned int DynamixelP::next_indirect_addr_write(void) const {
  return ADDR_START_INDIRECT_ADDR_WRITE +
    LEN_INDIRECT_ADDRESS * total_length_of_indirect_addr_write_;
}

bool DynamixelP::extract_present_position_from_sync_read(
    const dynamixel_base::comm_t & comm, const std::string & group_name,
    double & position_rad) {
  uint32_t data = 0;
  if (!comm->get_sync_read_data(
    group_name, id_, indirect_addr_of_present_position(), LEN_PRESENT_POSITION, data)) {
    return false;
  }
  position_rad = to_position_radian(static_cast<int32_t>(data));
  return true;
}

bool DynamixelP::extract_present_velocity_from_sync_read(
    const dynamixel_base::comm_t & comm, const std::string & group_name,
    double & velocity_rps) {
  uint32_t data = 0;
  if (!comm->get_sync_read_data(
    group_name, id_, indirect_addr_of_present_velocity(), LEN_PRESENT_VELOCITY, data)) {
    return false;
  }
  velocity_rps = to_velocity_rps(static_cast<int32_t>(data));
  return true;
}

bool DynamixelP::extract_present_current_from_sync_read(
    const dynamixel_base::comm_t & comm, const std::string & group_name,
    double & current_ampere) {
  uint32_t data = 0;
  if (!comm->get_sync_read_data(
    group_name, id_, indirect_addr_of_present_current(), LEN_PRESENT_CURRENT, data)) {
    return false;
  }
  current_ampere = to_current_ampere(static_cast<int16_t>(data));
  return true;
}

bool DynamixelP::extract_present_input_voltage_from_sync_read(
    const dynamixel_base::comm_t & comm, const std::string & group_name,
    double & voltage_volt) {
  uint32_t data = 0;
  if (!comm->get_sync_read_data(
    group_name, id_, indirect_addr_of_present_input_voltage(), LEN_PRESENT_VOLTAGE, data)) {
    return false;
  }
  voltage_volt = to_voltage_volt(static_cast<int16_t>(data));
  return true;
}

bool DynamixelP::extract_present_temperature_from_sync_read(
    const dynamixel_base::comm_t & comm, const std::string & group_name,
    int & temperature_deg) {
  uint32_t data = 0;
  if (!comm->get_sync_read_data(
    group_name, id_, indirect_addr_of_present_temperature(), LEN_PRESENT_TEMPERATURE, data)) {
    return false;
  }
  temperature_deg = static_cast<int8_t>(data);
  return true;
}

void DynamixelP::push_back_position_for_sync_write(
    const double position_rad, std::vector<uint8_t> & write_data) {
  uint32_t dxl_position = from_position_radian(position_rad);
  write_data.push_back(DXL_LOBYTE(DXL_LOWORD(dxl_position)));
  write_data.push_back(DXL_HIBYTE(DXL_LOWORD(dxl_position)));
  write_data.push_back(DXL_LOBYTE(DXL_HIWORD(dxl_position)));
  write_data.push_back(DXL_HIBYTE(DXL_HIWORD(dxl_position)));
}

void DynamixelP::push_back_velocity_for_sync_write(
    const double velocity_rps, std::vector<uint8_t> & write_data) {
  uint32_t dxl_velocity = from_velocity_rps(velocity_rps);
  write_data.push_back(DXL_LOBYTE(DXL_LOWORD(dxl_velocity)));
  write_data.push_back(DXL_HIBYTE(DXL_LOWORD(dxl_velocity)));
  write_data.push_back(DXL_LOBYTE(DXL_HIWORD(dxl_velocity)));
  write_data.push_back(DXL_HIBYTE(DXL_HIWORD(dxl_velocity)));
}

void DynamixelP::push_back_current_for_sync_write(
    const double current_ampere, std::vector<uint8_t> & write_data) {
  uint16_t dxl_current = from_current_ampere(current_ampere);
  write_data.push_back(DXL_LOBYTE(dxl_current));
  write_data.push_back(DXL_HIBYTE(dxl_current));
}

bool DynamixelP::set_indirect_address_read(
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

bool DynamixelP::set_indirect_address_write(
    const dynamixel_base::comm_t & comm, const uint16_t addr, const uint16_t len,
    uint16_t & indirect_addr) {
  bool retval = true;
  for (int i = 0; i < len; i++) {
    uint16_t target_indirect_address = next_indirect_addr_write() + LEN_INDIRECT_ADDRESS * i;
    uint16_t target_data_address = addr + i;
    if (!comm->write_word_data(
      id_, target_indirect_address, target_data_address)) {
      retval = false;
    }
  }
  // テストしやすくするため、write_word_dataに失敗しても変数を更新する
  indirect_addr = ADDR_START_INDIRECT_DATA_WRITE
    + total_length_of_indirect_addr_write_;
  total_length_of_indirect_addr_write_ += len;
  return retval;
}

}  // namespace dynamixel_p
