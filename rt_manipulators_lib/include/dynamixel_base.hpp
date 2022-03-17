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

#ifndef RT_MANIPULATORS_LIB_INCLUDE_DYNAMIXEL_BASE_HPP_
#define RT_MANIPULATORS_LIB_INCLUDE_DYNAMIXEL_BASE_HPP_

#include <memory>
#include <string>

#include "hardware_communicator.hpp"

namespace dynamixel_base {

using comm_t = std::shared_ptr<hardware_communicator::Communicator>;

class DynamixelBase {
 public:
  explicit DynamixelBase(const uint8_t id) : id_(id), name_("base") {}
  ~DynamixelBase() {}

  uint8_t get_id() const { return id_; }
  std::string get_name() const { return name_; }
  virtual bool write_torque_enable(
    const dynamixel_base::comm_t & comm, const bool enable) { return false; }
  virtual bool write_position_p_gain(
    const dynamixel_base::comm_t & comm, const unsigned int gain) { return false; }
  virtual bool write_position_i_gain(
    const dynamixel_base::comm_t & comm, const unsigned int gain) { return false; }
  virtual bool write_position_d_gain(
    const dynamixel_base::comm_t & comm, const unsigned int gain) { return false; }

 protected:
  uint8_t id_;
  std::string name_;
};

}  // namespace dynamixel_base

#endif  // RT_MANIPULATORS_LIB_INCLUDE_DYNAMIXEL_BASE_HPP_
