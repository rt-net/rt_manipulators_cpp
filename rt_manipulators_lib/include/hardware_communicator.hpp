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

#ifndef RT_MANIPULATORS_LIB_INCLUDE_HARDWARE_COMMUNICATOR_HPP_
#define RT_MANIPULATORS_LIB_INCLUDE_HARDWARE_COMMUNICATOR_HPP_

#include <dynamixel_sdk/dynamixel_sdk.h>
#include <string>
#include <map>
#include <memory>
#include <vector>

namespace hardware_communicator {

using dxl_id_t = uint8_t;
using dxl_address_t = uint16_t;
using dxl_data_length_t = uint16_t;
using dxl_error_t = uint8_t;
using dxl_result_t = int;
using dxl_byte_t = uint8_t;
using dxl_word_t = uint16_t;
using dxl_double_word_t = uint32_t;
using group_name_t = std::string;
using GroupSyncRead = dynamixel::GroupSyncRead;
using GroupSyncWrite = dynamixel::GroupSyncWrite;

// ハードウェアとの通信を担うクラス
class Communicator{
 public:
  explicit Communicator(const std::string device_name);
  ~Communicator();
  bool is_connected();
  bool connect(const int baudrate = 3000000);
  void disconnect();
  void append_sync_read_group(const group_name_t & group_name, const dxl_address_t & start_address,
                              const dxl_data_length_t & data_length);
  void append_sync_write_group(const group_name_t & group_name, const dxl_address_t & start_address,
                               const dxl_data_length_t & data_length);
  bool append_id_to_sync_read_group(const group_name_t & group_name, const dxl_id_t & id);
  bool append_id_to_sync_write_group(const group_name_t & group_name, const dxl_id_t & id,
                                     std::vector<dxl_byte_t> & init_data);
  bool send_sync_read_packet(const group_name_t & group_name);
  bool send_sync_write_packet(const group_name_t & group_name);
  bool get_sync_read_data(const group_name_t & group_name, const dxl_id_t id,
                          const dxl_address_t & address, const dxl_data_length_t & length,
                          dxl_double_word_t & read_data);
  bool set_sync_write_data(const group_name_t & group_name, const dxl_id_t id,
                           std::vector<dxl_byte_t> & write_data);
  bool write_byte_data(const dxl_id_t & id, const dxl_address_t & address,
                       const dxl_byte_t & write_data);
  bool write_word_data(const dxl_id_t & id, const dxl_address_t & address,
                       const dxl_word_t & write_data);
  bool write_double_word_data(const dxl_id_t & id, const dxl_address_t & address,
                              const dxl_double_word_t & write_data);
  bool read_byte_data(const dxl_id_t & id, const dxl_address_t & address, dxl_byte_t & read_data);
  bool read_double_word_data(const dxl_id_t & id, const dxl_address_t & address,
                             dxl_double_word_t & read_data);

 private:
  std::shared_ptr<GroupSyncRead> sync_read_group(const group_name_t & name);
  std::shared_ptr<GroupSyncWrite> sync_write_group(const group_name_t & name);
  bool has_sync_read_group(const group_name_t & name);
  bool has_sync_write_group(const group_name_t & name);
  bool parse_dxl_error(const std::string & func_name, const dxl_id_t & id,
                       const dxl_address_t & address, const dxl_result_t & dxl_comm_result,
                       const dxl_error_t & dxl_packet_error);
  bool parse_dxl_error(const std::string & func_name, const dxl_result_t & dxl_comm_result);

  bool is_connected_;
  std::shared_ptr<dynamixel::PortHandler> port_handler_;
  std::shared_ptr<dynamixel::PacketHandler> packet_handler_;
  std::map<group_name_t, std::shared_ptr<GroupSyncRead>> sync_read_groups_;
  std::map<group_name_t, std::shared_ptr<GroupSyncWrite>> sync_write_groups_;
};

}  // namespace hardware_communicator


#endif  // RT_MANIPULATORS_LIB_INCLUDE_HARDWARE_COMMUNICATOR_HPP_

