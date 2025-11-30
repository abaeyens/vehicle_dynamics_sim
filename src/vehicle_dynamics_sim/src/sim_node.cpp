#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <vehicle_dynamics_sim/sim.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vehicle_dynamics_sim::SimNode>());
  rclcpp::shutdown();
  return 0;
}
