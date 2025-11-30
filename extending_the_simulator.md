# Extending the Simulator

## Adding a New Vehicle Type

1. **Define the vehicle class** in [`src/vehicle_dynamics_sim/include/vehicle_dynamics_sim/vehicles.hpp`](
     src/vehicle_dynamics_sim/include/vehicle_dynamics_sim/vehicles.hpp):
   ```cpp
   class MyVehicle : public Vehicle {
   public:
     MyVehicle(rclcpp::Node & node, const std::string & ns);
     void update(const rclcpp::Time & time, 
                 const geometry_msgs::msg::TwistStamped & reference_twist) override;
     std::string get_robot_description() const override;
   private:
     // Your parameters and actuators
   };
   ```

2. **Implement kinematics** in [`src/vehicle_dynamics_sim/src/vehicles.cpp`](
     src/vehicle_dynamics_sim/src/vehicles.cpp):
   - Load parameters in constructor
   - Implement `update()` with your kinematic model
   - Generate URDF in `get_robot_description()`

3. **Register in factory**
   ([`vehicles.cpp`](src/vehicle_dynamics_sim/src/vehicles.cpp)):
   ```cpp
   case VehicleName::MY_VEHICLE:
     return std::make_unique<MyVehicle>(node, ns);
   ```

4. **Add to enum** in [`vehicles.hpp`](
     src/vehicle_dynamics_sim/include/vehicle_dynamics_sim/vehicles.hpp):
   ```cpp
   enum class VehicleName : uint8_t {
     BICYCLE, DIFFERENTIAL, OMNI, MY_VEHICLE
   };
   ```

5. **Update string conversion** functions ([`toString()` and `toVehicleName()`](
  src/vehicle_dynamics_sim/include/vehicle_dynamics_sim/vehicles.hpp))

6. **Add parameters** to [`src/vehicle_dynamics_sim/params/all_vehicles.yaml`](
  src/vehicle_dynamics_sim/params/all_vehicles.yaml)

## Adding Custom Actuators
Existinc examples: `DriveActuator` and `SteeringActuator` in [vehicles.hpp](
  src/vehicle_dynamics_sim/include/vehicle_dynamics_sim/vehicles.hpp).

Implement:
- Constructor with parameter loading
- State update method
- Internal state variables

Example use cases:
- Hydraulic actuators with different dynamics
- Pneumatic systems with pressure dynamics
