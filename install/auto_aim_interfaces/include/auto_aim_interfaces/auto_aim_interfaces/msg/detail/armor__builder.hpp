// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from auto_aim_interfaces:msg/Armor.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__ARMOR__BUILDER_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__ARMOR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "auto_aim_interfaces/msg/detail/armor__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace auto_aim_interfaces
{

namespace msg
{

namespace builder
{

class Init_Armor_pose
{
public:
  explicit Init_Armor_pose(::auto_aim_interfaces::msg::Armor & msg)
  : msg_(msg)
  {}
  ::auto_aim_interfaces::msg::Armor pose(::auto_aim_interfaces::msg::Armor::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Armor msg_;
};

class Init_Armor_distance_to_image_center
{
public:
  explicit Init_Armor_distance_to_image_center(::auto_aim_interfaces::msg::Armor & msg)
  : msg_(msg)
  {}
  Init_Armor_pose distance_to_image_center(::auto_aim_interfaces::msg::Armor::_distance_to_image_center_type arg)
  {
    msg_.distance_to_image_center = std::move(arg);
    return Init_Armor_pose(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Armor msg_;
};

class Init_Armor_number
{
public:
  explicit Init_Armor_number(::auto_aim_interfaces::msg::Armor & msg)
  : msg_(msg)
  {}
  Init_Armor_distance_to_image_center number(::auto_aim_interfaces::msg::Armor::_number_type arg)
  {
    msg_.number = std::move(arg);
    return Init_Armor_distance_to_image_center(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Armor msg_;
};

class Init_Armor_color
{
public:
  Init_Armor_color()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Armor_number color(::auto_aim_interfaces::msg::Armor::_color_type arg)
  {
    msg_.color = std::move(arg);
    return Init_Armor_number(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Armor msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::auto_aim_interfaces::msg::Armor>()
{
  return auto_aim_interfaces::msg::builder::Init_Armor_color();
}

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__ARMOR__BUILDER_HPP_
