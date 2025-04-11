#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class JoySubscriber : public rclcpp::Node {
public:
    JoySubscriber() : Node("joy_subscriber") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", rclcpp::QoS(10), 
            std::bind(&JoySubscriber::joy_callback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Joy Subscriber Node has started.");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Axes: [%f, %f]  Buttons: [%d, %d]", 
                    msg->axes[0], msg->axes[1], 
                    msg->buttons[0], msg->buttons[1]);
    }
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoySubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
