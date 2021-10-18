#ifndef JOINT_HPP_
#define JOINT_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace joint
{

class Joint
{
public:

  Joint(const uint8_t id, const uint8_t operating_mode);
  uint8_t id() const;
  uint8_t operating_mode() const;
  void set_position(const double position_radian);
  double get_position() const;

private:
  uint8_t id_;
  uint8_t operating_mode_;
  double position_;
};

class JointGroup
{
public:
  JointGroup(const std::vector<std::string> & joint_names);
  std::vector<std::string> joint_names() const;

private:
  std::vector<std::string> joint_names_;
};

}  // namespace rt_manipulators_cpp

#endif // JOINT_HPP_