#include "rclcpp/rclcpp.hpp"
#include "rclcpp/experimental/executors/events_executor/events_executor.hpp"

// Grab Wanted Nodes 
#include "FDroneGroundControl/StateEstimatorNode.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Running Events Executor b/c high efficiency
  rclcpp::experimental::executors::EventsExecutor executor;
  auto stateNode = std::make_shared<StateEstimatorNode>();

  executor.add_node(stateNode);
  executor.spin();

  // Cleanup our running process
  rclcpp::shutdown();
}
