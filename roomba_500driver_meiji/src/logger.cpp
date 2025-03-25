// Copyright 2023 amsl

#include <chrono>
#include <cmath>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "roomba_500driver_meiji/msg/roomba500_state.hpp"

class RoombaLogger : public rclcpp::Node
{
public:
  RoombaLogger()
      : Node("multi_roomba_charging_logger"), clock_(RCL_SYSTEM_TIME), pre_enc_r_(0), pre_enc_l_(0)
  {

    roomba_states_ = this->create_subscription<roomba_500driver_meiji::msg::Roomba500State>(
        "/roomba/states",
        10,
        std::bind(&RoombaLogger::handle_roomba_control, this, std::placeholders::_1));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    current_time_ = clock_.now();
    last_time_ = clock_.now();

    // Initialization of timer should be last.
    timer_ = this->create_wall_timer(1000ms, std::bind(&RoombaLogger::timer_callback, this));
  }

  ~RoombaLogger()
  {
    if (roomba_)
    {
      roomba_->powerOff();
      roomba_->time_->sleep(1);
    }
  }

private:
  void timer_callback()
  {
    current_time_ = clock_.now();
    const double dt = current_time_.seconds() - last_time_.seconds();

    roomba_500driver_meiji::msg::Roomba500State sens;
    sens.header.stamp = current_time_;

    RCLCPP_INFO(
        get_logger(),
        "charging_state: %d, voltage: %f, current: %f, temperature: %d, charge: %f, capacity: %f, distance: %f",
        sens.charging_state, sens.voltage, sens.current, sens.temperature, sens.charge, sens.capacity, sens.distance;
        last_time_ = current_time_;);
  }

  rclcpp::Subscription<roomba_500driver_meiji::msg::Roomba500State>::SharedPtr roomba_states_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Clock clock_;
  rclcpp::Time current_time_;
  rclcpp::Time last_time_;
  int pre_enc_r_;
  int pre_enc_l_;
};
