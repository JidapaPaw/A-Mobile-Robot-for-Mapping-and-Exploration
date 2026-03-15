// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from serial_motor_demo_msgs:msg/MotorCommand.idl
// generated code does not contain a copyright notice

#ifndef SERIAL_MOTOR_DEMO_MSGS__MSG__DETAIL__MOTOR_COMMAND__BUILDER_HPP_
#define SERIAL_MOTOR_DEMO_MSGS__MSG__DETAIL__MOTOR_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "serial_motor_demo_msgs/msg/detail/motor_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace serial_motor_demo_msgs
{

namespace msg
{

namespace builder
{

class Init_MotorCommand_right
{
public:
  explicit Init_MotorCommand_right(::serial_motor_demo_msgs::msg::MotorCommand & msg)
  : msg_(msg)
  {}
  ::serial_motor_demo_msgs::msg::MotorCommand right(::serial_motor_demo_msgs::msg::MotorCommand::_right_type arg)
  {
    msg_.right = std::move(arg);
    return std::move(msg_);
  }

private:
  ::serial_motor_demo_msgs::msg::MotorCommand msg_;
};

class Init_MotorCommand_left
{
public:
  Init_MotorCommand_left()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorCommand_right left(::serial_motor_demo_msgs::msg::MotorCommand::_left_type arg)
  {
    msg_.left = std::move(arg);
    return Init_MotorCommand_right(msg_);
  }

private:
  ::serial_motor_demo_msgs::msg::MotorCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::serial_motor_demo_msgs::msg::MotorCommand>()
{
  return serial_motor_demo_msgs::msg::builder::Init_MotorCommand_left();
}

}  // namespace serial_motor_demo_msgs

#endif  // SERIAL_MOTOR_DEMO_MSGS__MSG__DETAIL__MOTOR_COMMAND__BUILDER_HPP_
