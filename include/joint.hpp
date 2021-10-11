#ifndef JOINT_HPP_
#define JOINT_HPP_

#include <string>
#include <vector>

namespace joint
{

class Joint
{
public:
  Joint(std::string name, int id, int operating_mode) : 
    name_(name), id_(id), operating_mode_(operating_mode) {}
  std::string name() const { return name_; }
  int id() const { return id_; }
  int operating_mode() const { return operating_mode_; }

private:
  std::string name_;
  int id_;
  int operating_mode_;
};

class JointGroup
{
public:
  JointGroup(std::string name) :
    name_(name) {}
  std::string name() const { return name_; }
  void push_back(const Joint & joint){ joints_.push_back(joint); }
  std::vector<Joint> joints() const { return joints_; }

private:
  std::string name_;
  std::vector<Joint> joints_;
};

}  // namespace rt_manipulators_cpp

#endif // JOINT_HPP_