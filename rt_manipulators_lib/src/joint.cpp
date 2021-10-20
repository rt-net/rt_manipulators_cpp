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

#include "joint.hpp"

namespace joint
{

Joint::Joint(const uint8_t id, const uint8_t operating_mode) : 
  id_(id), operating_mode_(operating_mode)
{

}

uint8_t Joint::id() const
{
  return id_;
}

uint8_t Joint::operating_mode() const
{
  return operating_mode_;
}

void Joint::set_present_position(const double position_radian)
{
  present_position_ = position_radian;
}

double Joint::get_present_position() const
{
  return present_position_;
}

void Joint::set_goal_position(const double position_radian)
{
  goal_position_ = position_radian;
}

double Joint::get_goal_position() const
{
  return goal_position_;
}


JointGroup::JointGroup(const std::vector<std::string> & joint_names,
  const std::vector<std::string> & sync_read_targets,
  const std::vector<std::string> & sync_write_targets) :
  joint_names_(joint_names),
  sync_read_position_enabled_(false),
  sync_read_velocity_enabled_(false),
  sync_read_current_enabled_(false),
  sync_read_temperature_enabled_(false),
  sync_write_position_enabled_(false),
  sync_write_velocity_enabled_(false),
  sync_write_current_enabled_(false)
{
  for(auto target : sync_read_targets){
    if(target == "position") sync_read_position_enabled_ = true;
    if(target == "velocity") sync_read_velocity_enabled_ = true;
    if(target == "current") sync_read_current_enabled_ = true;
    if(target == "temperature") sync_read_temperature_enabled_ = true;
  }

  for(auto target : sync_write_targets){
    if(target == "position") sync_write_position_enabled_ = true;
    if(target == "velocity") sync_write_velocity_enabled_ = true;
    if(target == "current") sync_write_current_enabled_ = true;
  }
}

std::vector<std::string> JointGroup::joint_names() const
{
  return joint_names_;
}

bool JointGroup::sync_read_position_enabled() const
{
  return sync_read_position_enabled_;
}

bool JointGroup::sync_read_velocity_enabled() const
{
  return sync_read_velocity_enabled_;
}

bool JointGroup::sync_read_current_enabled() const
{
  return sync_read_current_enabled_;
}

bool JointGroup::sync_read_temperature_enabled() const
{
  return sync_read_temperature_enabled_;
}

bool JointGroup::sync_write_position_enabled() const
{
  return sync_write_position_enabled_;
}

bool JointGroup::sync_write_velocity_enabled() const
{
  return sync_write_velocity_enabled_;
}

bool JointGroup::sync_write_current_enabled() const
{
  return sync_write_current_enabled_;
}

}  // namespace rt_manipulators_cpp
