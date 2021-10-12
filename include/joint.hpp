#ifndef JOINT_HPP_
#define JOINT_HPP_

namespace joint
{

class Joint
{
public:
  Joint(int id, int operating_mode);
  int id() const;
  int operating_mode() const;

private:
  int id_;
  int operating_mode_;
};

}  // namespace rt_manipulators_cpp

#endif // JOINT_HPP_