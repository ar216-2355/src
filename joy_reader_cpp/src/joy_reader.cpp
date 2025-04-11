#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class JoyReader : public rclcpp::Node
{
public:
    JoyReader() : Node("joy_reader")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy & msg>(
            "joy", 10, std::bind(&JoyReader::joy_callback, this, std::placeholders::_1));
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Buttons: ");
        for (size_t i = 0; i < msg->buttons.size(); i++)
        {
            RCLCPP_INFO(this->get_logger(), "Button[%ld]: %d", i, msg->buttons[i]);
        }

        RCLCPP_INFO(this->get_logger(), "Axes: ");
        for (size_t i = 0; i < msg->axes.size(); i++)
        {
            RCLCPP_INFO(this->get_logger(), "Axis[%ld]: %.2f", i, msg->axes[i]);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyReader>());
    rclcpp::shutdown();
    return 0;
}
