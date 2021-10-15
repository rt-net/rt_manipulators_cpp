#ifndef JOINT_HPP_
#define JOINT_HPP_

#include <cstdint>

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

}  // namespace rt_manipulators_cpp

#endif // JOINT_HPP_