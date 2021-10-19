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
  void set_present_position(const double position_radian);
  double get_present_position() const;
  void set_goal_position(const double position_radian);
  double get_goal_position() const;

private:
  uint8_t id_;
  uint8_t operating_mode_;
  double present_position_;
  double goal_position_;
};

class JointGroup
{
public:
  JointGroup(const std::vector<std::string> & joint_names,
    const std::vector<std::string> & sync_read_targets,
    const std::vector<std::string> & sync_write_targets);
  std::vector<std::string> joint_names() const;
  bool sync_read_position_enabled() const;
  bool sync_read_velocity_enabled() const;
  bool sync_read_current_enabled() const;
  bool sync_read_temperature_enabled() const;
  bool sync_write_position_enabled() const;
  bool sync_write_velocity_enabled() const;
  bool sync_write_current_enabled() const;

private:
  std::vector<std::string> joint_names_;
  bool sync_read_position_enabled_;
  bool sync_read_velocity_enabled_;
  bool sync_read_current_enabled_;
  bool sync_read_temperature_enabled_;
  bool sync_write_position_enabled_;
  bool sync_write_velocity_enabled_;
  bool sync_write_current_enabled_;
};

}  // namespace rt_manipulators_cpp

#endif // JOINT_HPP_