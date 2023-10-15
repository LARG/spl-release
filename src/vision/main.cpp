#include <VisionNode.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto vision_node = std::make_shared<VisionNode>();
  
  // Register the vision node and its sub nodes
  executor.add_node(vision_node);
  vision_node->register_stages(executor);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}