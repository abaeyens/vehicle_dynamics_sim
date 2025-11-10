#include <vehicle_dynamics_sim/sim.h>

#include <memory>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sim::SimNode>());
  rclcpp::shutdown();
  return 0;
}
