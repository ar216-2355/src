#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sensor_msgs/msg/joy.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "robomas_plugins/msg/frame.hpp"
#include "harurobo_package/harurobo_package_utils.hpp"

class Harurobo2 : public rclcpp::Node
{
private:
  void controller_callback(const sensor_msgs::msg::Joy & msg) const
  {
    if(msg.buttons[7]){
      can_setting_->publish(can_conection::solenoid_utils::to_can_mode(0x250,1));
    }
    else if(msg.buttons[6]){
      can_setting_->publish(can_conection::solenoid_utils::to_can_mode(0x250,0));
    }
    if(msg.buttons[2]){
      can_setting_->publish(can_conection::solenoid_utils::to_can_mode(0x251,1));
    }//エアシリンダー
    else if(msg.buttons[3]){
      can_setting_->publish(can_conection::solenoid_utils::to_can_mode(0x251,0));
    }//エアシリンダー
    if(msg.buttons[10]){
      can_setting_->publish(can_conection::solenoid_utils::to_can_mode(0x252,1));
    }//LED
    else if(msg.buttons[9]){
      can_setting_->publish(can_conection::solenoid_utils::to_can_mode(0x252,0));
    }//LED
  } 
  
public:
  Harurobo2()
  : Node("harurobo2")
  {
    this->controller_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&Harurobo2::controller_callback, this, std::placeholders::_1));
    this->can_setting_ = this->create_publisher<robomas_plugins::msg::Frame>("robomas_can_tx", 10);
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr controller_;
  rclcpp::Publisher<robomas_plugins::msg::Frame>::SharedPtr can_setting_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Harurobo2>());
  rclcpp::shutdown();
  return 0;
}