
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

void Joint::set_position(const double position_radian)
{
  position_ = position_radian;
}

double Joint::get_position() const
{
  return position_;
}


JointGroup::JointGroup(const std::vector<std::string> & joint_names,
  const std::vector<std::string> & sync_read_targets) :
  joint_names_(joint_names),
  sync_read_position_enabled_(false),
  sync_read_velocity_enabled_(false),
  sync_read_current_enabled_(false),
  sync_read_temperature_enabled_(false)
{
  for(auto target : sync_read_targets){
    if(target == "position") sync_read_position_enabled_ = true;
    if(target == "velocity") sync_read_velocity_enabled_ = true;
    if(target == "current") sync_read_current_enabled_ = true;
    if(target == "temperature") sync_read_temperature_enabled_ = true;
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

}  // namespace rt_manipulators_cpp
