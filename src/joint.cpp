
#include "joint.hpp"

namespace joint
{

Joint::Joint(int id, int operating_mode) : 
  id_(id), operating_mode_(operating_mode)
{

}

int Joint::id() const
{
  return id_;
}

int Joint::operating_mode() const
{
  return operating_mode_;
}

}  // namespace rt_manipulators_cpp
