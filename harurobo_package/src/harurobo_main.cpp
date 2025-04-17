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
    //startボタンで活動モード
    if(msg.buttons[7])
    {
      omuni_setting_->publish(harurobo2::robomas_utils::to_velocity_mode(0));
      omuni_setting_->publish(harurobo2::robomas_utils::to_velocity_mode(1));
      omuni_setting_->publish(harurobo2::robomas_utils::to_velocity_mode(2));
    }
    //backボタンで停止モード
    if(msg.buttons[6])
    {
      omuni_setting_->publish(harurobo2::robomas_utils::to_disable_mode(0));
      omuni_setting_->publish(harurobo2::robomas_utils::to_disable_mode(1));
      omuni_setting_->publish(harurobo2::robomas_utils::to_disable_mode(2));
    }

    //sokudono saidaiti
    int max_v = 100;
    int max_R = 100;

    //オムニそれぞれの速度変数を指定
    float v1 = max_v, v2 = max_v, v3 = max_v;

    v1 *= msg.axes[3] + max_v * (axes[5] - axes[2]);
    v2 *= msg.axes[3] * (-0.5) + msg.axes[4] * (-0.866) + max_v * (axes[5] - axes[2]);
    v3 *= msg.axes[3] * (-0.5) + msg.axes[4] * 0.866 + max_v * (axes[5] - axes[2]);

    //make_target関数を使用
    auto message1 = harurobo2::robomas_utils::make_target(v1);
    auto message2 = harurobo2::robomas_utils::make_target(v2);
    auto message3 = harurobo2::robomas_utils::make_target(v3);

    //メッセージをrobomasにパブリッシュ
    omuni1_->publish(message1);
    omuni2_->publish(message2);
    omuni3_->publish(message3);
  } 
  
public:
  Harurobo2()
  : Node("harurobo2")
  {
    this->controller_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&Harurobo2::controller_callback, this, std::placeholders::_1));
    this->omuni1_ = this->create_publisher<robomas_plugins::msg::RobomasTarget>("robomas_target1", 10);
    this->omuni2_ = this->create_publisher<robomas_plugins::msg::RobomasTarget>("robomas_target2", 10);
    this->omuni3_ = this->create_publisher<robomas_plugins::msg::RobomasTarget>("robomas_target3", 10);
    this->omuni_setting_  = this->create_publisher<robomas_plugins::msg::RobomasFrame>("robomas_frame", 10);
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr controller_;
  rclcpp::Publisher<robomas_plugins::msg::RobomasTarget>::SharedPtr omuni1_;
  rclcpp::Publisher<robomas_plugins::msg::RobomasTarget>::SharedPtr omuni2_;
  rclcpp::Publisher<robomas_plugins::msg::RobomasTarget>::SharedPtr omuni3_;
  rclcpp::Publisher<robomas_plugins::msg::RobomasFrame>::SharedPtr omuni_setting_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Harurobo2>());
  rclcpp::shutdown();
  return 0;
}