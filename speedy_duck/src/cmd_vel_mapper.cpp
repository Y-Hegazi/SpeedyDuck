#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class Mapper : public rclcpp::Node
{
public:
    Mapper() : Node("converter")
    {
        publisher = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/diff_cont/cmd_vel", 10);
        subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&Mapper::sub_callback, this, std::placeholders::_1));
    }
private:
    void sub_callback(geometry_msgs::msg::Twist::SharedPtr msg)
    {
        geometry_msgs::msg::TwistStamped new_msg;
        new_msg.header.stamp = this->get_clock()->now();
        new_msg.header.frame_id = "base_link";
        new_msg.twist.linear = msg->linear;
        new_msg.twist.angular = msg->angular;
        publisher->publish(new_msg);
    }
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber;
};
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Mapper>());
    rclcpp::shutdown();
    return 0;
}