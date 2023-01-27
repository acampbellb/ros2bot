#include "rclcpp/rclcpp.hpp"
#include "ros2bot_startup_pkg/ros2bot_base_node.hpp"

class Ros2botBaseNode : public rclcpp::Node
{

public:

    Ros2botBaseNode() : Node("ros2bot_base_node")
    {
        RCLCPP_INFO(this->get_logger(), "Hello ros2bot base node");
    }

private:

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Ros2botBaseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}