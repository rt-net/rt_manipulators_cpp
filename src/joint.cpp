
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


JointGroup::JointGroup(const std::vector<std::string> & joint_names) :
  joint_names_(joint_names)
{

}

std::vector<std::string> JointGroup::joint_names() const
{
  return joint_names_;
}

}  // namespace rt_manipulators_cpp
