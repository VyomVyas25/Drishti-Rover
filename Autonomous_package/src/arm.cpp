#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

class ManipulatorController : public rclcpp::Node
{
public:
    ManipulatorController() : Node("manipulator_controller")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("manipulator_cmd", 10);
        ManipulatorController::move_to_position0();
    }

private:
void move_to_position0()
    {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 0.0;  
        message.linear.y = 0.0;  
        message.linear.z = 0.0;  
        message.angular.x = 0.0; 
        message.angular.y = 0.0; 
        message.angular.z = 1.0;
        publisher_->publish(message);
        timer_ = this->create_wall_timer(4s, std::bind(&ManipulatorController::move_to_position1, this));
    }
    void move_to_position1()
    {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 0.6;  
        message.linear.y = 0.0;  
        message.linear.z = 0.0;  
        message.angular.x = 0.0; 
        message.angular.y = 0.0; 
        message.angular.z = 1.0;
        publisher_->publish(message);
        timer_ = this->create_wall_timer(5s, std::bind(&ManipulatorController::move_to_position2, this));
    }

    void move_to_position2()
    {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 0.0;  
        message.linear.y = 0.6;  
        message.linear.z = 0.0;  
        message.angular.x = 0.0; 
        message.angular.y = 0.0;
        message.angular.z = 1.0; 
        publisher_->publish(message);
        timer_ = this->create_wall_timer(12s, std::bind(&ManipulatorController::move_to_position3, this));
    }

    void move_to_position3()
    {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 0.0;  
        message.linear.y = 0.0;  
        message.linear.z = 0.6;  
        message.angular.x = 0.0; 
        message.angular.y = 0.0;
        message.angular.z = 1.0; 
        publisher_->publish(message);
        timer_ = this->create_wall_timer(12s, std::bind(&ManipulatorController::move_to_position4, this));
    }

    void move_to_position4()
    {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 0.0;  
        message.linear.y = 0.0;  
        message.linear.z = 0.0;  
        message.angular.x = 0.5; 
        message.angular.y = 0.0;
        message.angular.z = 1.0;
        publisher_->publish(message);
        timer_ = this->create_wall_timer(8s, std::bind(&ManipulatorController::move_to_position5, this));
    }

    void move_to_position5()
    {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 0.0;  
        message.linear.y = 0.0;  
        message.linear.z = 0.0;  
        message.angular.x = 0.0; 
        message.angular.y = 0.6;
        message.angular.z = 1.0;
        publisher_->publish(message);
        timer_ = this->create_wall_timer(6s, std::bind(&ManipulatorController::stop, this));
    }

    void stop()
    {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 0.0;
        message.linear.y = 0.0;
        message.linear.z = 0.0;
        message.angular.x = 0.0;
        message.angular.y = 0.0;
        message.angular.z = 0.0;
        publisher_->publish(message);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ManipulatorController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
