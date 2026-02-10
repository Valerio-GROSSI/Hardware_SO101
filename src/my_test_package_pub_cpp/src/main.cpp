#include <rclcpp/rclcpp.hpp>

class MonNode : public rclcpp::Node
{
public:
  MonNode() : Node("mon_node")
  {
    RCLCPP_INFO(this->get_logger(), "Node démarré");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MonNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}