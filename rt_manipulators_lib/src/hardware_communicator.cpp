// Copyright 2021 RT Corporation
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

#include <iostream>

#include "hardware_communicator.hpp"

namespace hardware_communicator {

const double PROTOCOL_VERSION = 2.0;

Communicator::Communicator(const std::string device_name) :
  is_connected_(false) {
  port_handler_ = std::shared_ptr<dynamixel::PortHandler>(
      dynamixel::PortHandler::getPortHandler(device_name.c_str()));
  packet_handler_ = std::shared_ptr<dynamixel::PacketHandler>(
      dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION));
}

Communicator::~Communicator() {
  disconnect();
}

bool Communicator::is_connected() { return is_connected_; }

bool Communicator::connect(const int baudrate) {
  if (!port_handler_->setBaudRate(baudrate)) {
    std::cerr << "Unable to set baudrate: " << std::to_string(baudrate) << std::endl;
    return false;
  }
  if (!port_handler_->openPort()) {
    std::cerr << "Unable to open port: " << port_handler_->getPortName() << std::endl;
    return false;
  }

  is_connected_ = true;
  return true;
}

void Communicator::disconnect() {
  if (is_connected_ == false) {
    return;
  }
  port_handler_->closePort();
  is_connected_ = false;
}

void Communicator::append_sync_read_group(
  const group_name_t & group_name, const dxl_address_t & start_address,
  const dxl_data_length_t & data_length) {
  auto group_ptr = std::make_shared<GroupSyncRead>(
    port_handler_.get(), packet_handler_.get(), start_address, data_length);
  sync_read_groups_.emplace(group_name, group_ptr);
}

void Communicator::append_sync_write_group(
  const group_name_t & group_name, const dxl_address_t & start_address,
  const dxl_data_length_t & data_length) {
  auto group_ptr = std::make_shared<GroupSyncWrite>(
    port_handler_.get(), packet_handler_.get(), start_address, data_length);
  sync_write_groups_.emplace(group_name, group_ptr);
}

std::shared_ptr<GroupSyncRead> Communicator::sync_read_group(const group_name_t & name) {
  return sync_read_groups_.at(name);
}

std::shared_ptr<GroupSyncWrite> Communicator::sync_write_group(const group_name_t & name) {
  return sync_write_groups_.at(name);
}

bool Communicator::send_sync_read_packet(const group_name_t & name) {
  if (!has_sync_read_group(name)) {
    std::cerr << name << "にはsync_read_groupが設定されていません." << std::endl;
    return false;
  }

  dxl_result_t dxl_result = sync_read_group(name)->txRxPacket();
  if (!parse_dxl_error(std::string(__func__), dxl_result)) {
    std::cerr << name << "のsync readに失敗しました." << std::endl;
    return false;
  }
  return true;
}

bool Communicator::send_sync_write_packet(const group_name_t & name) {
  if (!has_sync_write_group(name)) {
    std::cerr << name << "にはsync_write_groupが設定されていません." << std::endl;
    return false;
  }

  dxl_result_t dxl_result = sync_write_group(name)->txPacket();
  if (!parse_dxl_error(std::string(__func__), dxl_result)) {
    std::cerr << name << "のsync writeに失敗しました." << std::endl;
    return false;
  }
  return true;
}

bool Communicator::get_sync_read_data(
  const group_name_t & name, const dxl_id_t id, const dxl_address_t & address,
  const dxl_data_length_t & length, dxl_double_word_t & read_data) {
  if (!sync_read_group(name)->isAvailable(id, address, length)) {
    std::cerr << "id: " << std::to_string(id);
    std::cerr << ", addr: " << std::to_string(address);
    std::cerr << ", len: " << std::to_string(length);
    std::cerr << " is not available." << std::endl;
    return false;
  }
  read_data = sync_read_group(name)->getData(id, address, length);
  return true;
}

bool Communicator::set_sync_write_data(
  const group_name_t & name, const dxl_id_t id, std::vector<dxl_byte_t> & write_data) {
  if (!sync_write_group(name)->changeParam(id, write_data.data())) {
    std::cerr << name << ":" << std::to_string(id)
              << " のsyncWrite->changeParamに失敗しました." << std::endl;
    return false;
  }
  return true;
}

bool Communicator::write_byte_data(
  const dxl_id_t & id, const dxl_address_t & address, const dxl_byte_t & write_data) {
  dxl_error_t dxl_error = 0;
  dxl_result_t dxl_result =
      packet_handler_->write1ByteTxRx(port_handler_.get(), id, address, write_data, &dxl_error);

  if (!parse_dxl_error(std::string(__func__), id, address, dxl_result, dxl_error)) {
    return false;
  }
  return true;
}

bool Communicator::write_word_data(
  const dxl_id_t & id, const dxl_address_t & address, const dxl_word_t & write_data) {
  dxl_error_t dxl_error = 0;
  dxl_result_t dxl_result =
      packet_handler_->write2ByteTxRx(port_handler_.get(), id, address, write_data, &dxl_error);

  if (!parse_dxl_error(std::string(__func__), id, address, dxl_result, dxl_error)) {
    return false;
  }
  return true;
}

bool Communicator::write_double_word_data(
  const dxl_id_t & id, const dxl_address_t & address, const dxl_double_word_t & write_data) {
  dxl_error_t dxl_error = 0;
  dxl_result_t dxl_result =
      packet_handler_->write4ByteTxRx(port_handler_.get(), id, address, write_data, &dxl_error);

  if (!parse_dxl_error(std::string(__func__), id, address, dxl_result, dxl_error)) {
    return false;
  }
  return true;
}

bool Communicator::read_byte_data(
  const dxl_id_t & id, const dxl_address_t & address, dxl_byte_t & read_data) {
  dxl_error_t dxl_error = 0;
  dxl_byte_t data = 0;
  dxl_result_t dxl_result =
      packet_handler_->read1ByteTxRx(port_handler_.get(), id, address, &data, &dxl_error);

  if (!parse_dxl_error(std::string(__func__), id, address, dxl_result, dxl_error)) {
    return false;
  }
  read_data = data;
  return true;
}

bool Communicator::read_double_word_data(
  const dxl_id_t & id, const dxl_address_t & address, dxl_double_word_t & read_data) {
  dxl_error_t dxl_error = 0;
  dxl_double_word_t data = 0;
  dxl_result_t dxl_result =
      packet_handler_->read4ByteTxRx(port_handler_.get(), id, address, &data, &dxl_error);

  if (!parse_dxl_error(std::string(__func__), id, address, dxl_result, dxl_error)) {
    return false;
  }
  read_data = data;
  return true;
}

bool Communicator::parse_dxl_error(
  const std::string & func_name, const dxl_id_t & id, const dxl_address_t & address,
  const dxl_result_t & dxl_comm_result, const dxl_error_t & dxl_packet_error) {
  bool retval = true;

  if (dxl_comm_result != COMM_SUCCESS) {
    std::cerr << "Function:" << func_name;
    std::cerr << ", ID:" << std::to_string(id);
    std::cerr << ", Address:" << std::to_string(address);
    std::cerr << ", CommError:" << std::string(packet_handler_->getTxRxResult(dxl_comm_result))
              << std::endl;
    retval = false;
  }

  if (dxl_packet_error != 0) {
    std::cerr << "Function:" << func_name;
    std::cerr << ", ID:" << std::to_string(id);
    std::cerr << ", Address:" << std::to_string(address);
    std::cerr << ", PacketError:"
              << std::string(packet_handler_->getRxPacketError(dxl_packet_error)) << std::endl;
    retval = false;
  }

  return retval;
}

bool Communicator::parse_dxl_error(
  const std::string & func_name, const dxl_result_t & dxl_comm_result) {
  if (dxl_comm_result != COMM_SUCCESS) {
    std::cerr << "Function:" << func_name;
    std::cerr << ", CommError:" << std::string(packet_handler_->getTxRxResult(dxl_comm_result));
    std::cerr << std::endl;
    return false;
  }

  return true;
}

bool Communicator::has_sync_read_group(const group_name_t & name) {
  return sync_read_groups_.find(name) != sync_read_groups_.end();
}

bool Communicator::has_sync_write_group(const group_name_t & name) {
  return sync_write_groups_.find(name) != sync_write_groups_.end();
}

}  // namespace hardware_communicator
