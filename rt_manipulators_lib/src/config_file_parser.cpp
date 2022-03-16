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


#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>

#include "config_file_parser.hpp"
#include "joint.hpp"


namespace config_file_parser {

bool parse(const std::string& config_yaml, hardware_joints::Joints & parsed_joints) {
  std::ifstream fs(config_yaml);
  if (!fs.is_open()) {
    std::cerr << "コンフィグファイル:" << config_yaml << "が存在しません." << std::endl;
    return false;
  }

  YAML::Node config = YAML::LoadFile(config_yaml);
  for (const auto & config_joint_group : config["joint_groups"]) {
    auto group_name = config_joint_group.first.as<std::string>();
    if (parsed_joints.has_group(group_name)) {
      std::cerr << group_name << "グループが2つ以上存在します." << std::endl;
      return false;
    }

    if (!config["joint_groups"][group_name]["joints"]) {
      std::cerr << group_name << "グループに'joints'が設定されていせん。" << std::endl;
      return false;
    }

    std::vector<JointName> joint_names;
    for (const auto & config_joint : config["joint_groups"][group_name]["joints"]) {
      auto joint_name = config_joint.as<std::string>();
      if (parsed_joints.has_joint(joint_name)) {
        std::cerr << joint_name << "ジョイントが2つ以上存在します." << std::endl;
        return false;
      }

      if (!config[joint_name]) {
        std::cerr << joint_name << "ジョイントの設定が存在しません." << std::endl;
        return false;
      }

      if (!config[joint_name]["id"] || !config[joint_name]["operating_mode"]) {
        std::cerr << joint_name << "にidまたはoperating_modeが設定されていません." << std::endl;
        return false;
      }

      joint_names.push_back(joint_name);
      auto joint_id = config[joint_name]["id"].as<int>();
      auto ope_mode = config[joint_name]["operating_mode"].as<int>();
      std::string dynamixel_name = "";
      double position_limit_margin = 0;
      double current_limit_margin = 0;

      if (config[joint_name]["dynamixel"]) {
        dynamixel_name = config[joint_name]["dynamixel"].as<std::string>();
      }
      if (config[joint_name]["pos_limit_margin"]) {
        position_limit_margin = config[joint_name]["pos_limit_margin"].as<double>();
      }
      if (config[joint_name]["current_limit_margin"]) {
        current_limit_margin = config[joint_name]["current_limit_margin"].as<double>();
      }

      auto joint = joint::Joint(joint_id, ope_mode, dynamixel_name);
      joint.set_position_limit_margin(position_limit_margin);
      joint.set_current_limit_margin(current_limit_margin);
      parsed_joints.append_joint(joint_name, joint);
    }

    std::vector<std::string> sync_read_targets;
    std::vector<std::string> sync_write_targets;
    if (config["joint_groups"][group_name]["sync_read"]) {
      sync_read_targets =
          config["joint_groups"][group_name]["sync_read"].as<std::vector<std::string>>();
    }
    if (config["joint_groups"][group_name]["sync_write"]) {
      sync_write_targets =
          config["joint_groups"][group_name]["sync_write"].as<std::vector<std::string>>();
    }

    auto joint_group = joint::JointGroup(joint_names, sync_read_targets, sync_write_targets);

    // sync_readとsync_writeの関係をチェック
    if (joint_group.sync_write_velocity_enabled() && !joint_group.sync_read_position_enabled()) {
      std::cerr << group_name << "グループはvelocityをsync_writeしますが, ";
      std::cerr << "positionをsync_readしません." << std::endl;
      std::cerr << "positionもsync_readするようにコンフィグファイルを修正して下さい." << std::endl;
      return false;
    }
    if (joint_group.sync_write_current_enabled() && !joint_group.sync_read_position_enabled()) {
      std::cerr << group_name << "グループはcurrentをsync_writeしますが, ";
      std::cerr << "positionをsync_readしません." << std::endl;
      std::cerr << "positionもsync_readするようにコンフィグファイルを修正して下さい." << std::endl;
      return false;
    }

    parsed_joints.append_group(group_name, joint_group);
  }

  return true;
}

}  // namespace config_file_parser
