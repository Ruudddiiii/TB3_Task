#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class TurtleBotController : public rclcpp::Node
{
public:
    TurtleBotController() : Node("turtlebot_controller")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("tb3_1/cmd_vel", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&TurtleBotController::moveForward, this));
    }

private:
    void moveForward()
    {
        geometry_msgs::msg::Twist msg;
        msg.linear.x = 0.2;  // Linear velocity (m/s)
        msg.angular.z = 0.0; // Angular velocity (rad/s)
        publisher_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleBotController>());
    rclcpp::shutdown();
    return 0;
}
