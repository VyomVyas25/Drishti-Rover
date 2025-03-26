#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

class CombinedControl : public rclcpp::Node
{
public:
    CombinedControl()
    : Node("combined_control")
    {
        bot_cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        arm_cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_1", 10);
        bot_joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&CombinedControl::bot_joy_callback, this, std::placeholders::_1));
        arm_joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&CombinedControl::arm_joy_callback, this, std::placeholders::_1));
    }

private:
    void bot_joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
    {
        auto bot_twist = geometry_msgs::msg::Twist();
        bot_twist.linear.x = joy_msg->axes[4];  // Left joystick vertical axis
        bot_twist.angular.z = joy_msg->axes[3]; // Right joystick horizontal axis
        bot_cmd_vel_publisher_->publish(bot_twist);
    }

    void arm_joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
    {
        auto arm_twist = geometry_msgs::msg::Twist();

        arm_twist.linear.x = joy_msg->buttons[0] ? 1.0 : 0.0;  // Button 0 controls motor 1
        arm_twist.linear.y = joy_msg->buttons[1] ? 1.0 : 0.0;  // Button 1 controls motor 2
        arm_twist.linear.z = joy_msg->buttons[2] ? 1.0 : 0.0;  // Button 2 controls motor 3
        arm_twist.angular.x = joy_msg->buttons[3] ? 1.0 : 0.0; // Button 3 controls motor 4
        arm_twist.angular.y = joy_msg->buttons[4] ? 1.0 : 0.0; // Button 4 controls motor 5
        
        // Set angular.z based on the analog stick (for turning direction)
        arm_twist.angular.z = joy_msg->axes[1];

        arm_cmd_vel_publisher_->publish(arm_twist);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr bot_cmd_vel_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr arm_cmd_vel_publisher_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr bot_joy_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr arm_joy_subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CombinedControl>());
    rclcpp::shutdown();
    return 0;
}
