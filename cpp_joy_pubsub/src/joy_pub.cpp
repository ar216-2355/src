// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sensor_msgs/msg/joy.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher") //, count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    this->controller_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&MinimalPublisher::controller_callback, this, std::placeholders::_1));
  }

private:
  void controller_callback(const sensor_msgs::msg::Joy & msg)
  {
    auto message = std_msgs::msg::String();
    //message.data = "Hello, world! " + std::to_string(count_++);
    message.data = "Hello, world! x: " + std::to_string(msg.buttons[0]);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr controller_;
  //size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
