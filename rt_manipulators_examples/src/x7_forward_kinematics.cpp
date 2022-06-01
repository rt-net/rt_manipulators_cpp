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

// This file was converted to ROS 2 from pure c++ file:
// https://github.com/rt-net/rt_manipulators_cpp/blob/v1.1.2/samples/samples02/src/x7_forward_kinematics.cpp

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rt_manipulators_cpp/hardware.hpp"
#include "rt_manipulators_cpp/kinematics.hpp"
#include "rt_manipulators_cpp/kinematics_utils.hpp"
#include "rt_manipulators_cpp/link.hpp"

using namespace std::chrono_literals;

class X7ReadPosition : public rclcpp::Node
{
public:
  X7ReadPosition()
  : Node("x7_read_position")
  {
    timer_ = this->create_wall_timer(10ms, std::bind(&X7ReadPosition::timer_callback, this));

    this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
    this->declare_parameter<int>("baudrate", 3000000);
    this->declare_parameter<std::string>("config_file_path", "config/crane-x7.yaml");
    this->declare_parameter<std::string>("link_file_path", "config/crane-x7_links.csv");
  }

  ~X7ReadPosition()
  {
    if (hardware_) {
      hardware_->stop_thread();
      hardware_->disconnect();
    }
  }

  bool init(void)
  {
    const auto port_name = this->get_parameter("port_name").get_value<std::string>();
    const auto baudrate = this->get_parameter("baudrate").get_value<int>();
    const auto config_file_path = this->get_parameter("config_file_path").get_value<std::string>();
    const auto link_file_path = this->get_parameter("link_file_path").get_value<std::string>();

    hardware_ = std::make_shared<rt_manipulators_cpp::Hardware>(port_name);
    if (!hardware_->connect(baudrate)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to connect a robot.");
      return false;
    }

    if (!hardware_->load_config_file(config_file_path)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to read a config file.");
      return false;
    }
    std::vector<std::string> group_names = {"arm", "hand"};
    if (!hardware_->start_thread(group_names, std::chrono::milliseconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Failed to start a thread.");
      return -1;
    }

    links_ = kinematics_utils::parse_link_config_file(link_file_path);
    return true;
  }

private:
  void timer_callback()
  {
    std::vector<double> positions;
    if (hardware_->get_positions("arm", positions)) {
      set_arm_joint_positions(links_, positions);
      kinematics::forward_kinematics(links_, 1);

      int target_link = 8;
      auto pos_xyz = links_[target_link].p;
      RCLCPP_INFO(
        this->get_logger(), "Pos x:%f, y:%f, z:%f",
        pos_xyz[0], pos_xyz[1], pos_xyz[2]);
      auto euler_zyx = kinematics_utils::rotation_to_euler_ZYX(links_[target_link].R);
      RCLCPP_INFO(
        this->get_logger(), "Rot z:%f, y:%f, x:%f",
        euler_zyx[0], euler_zyx[1], euler_zyx[2]);
    }
  }

  void set_arm_joint_positions(
    std::vector<manipulators_link::Link> & links,
    std::vector<double> positions)
  {
    int start_id = 2;  // Link1
    for (auto i = 0; i < positions.size(); i++) {
      links[start_id + i].q = positions[i];
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<rt_manipulators_cpp::Hardware> hardware_;
  kinematics_utils::links_t links_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<X7ReadPosition>();
  if (node->init()) {
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}
