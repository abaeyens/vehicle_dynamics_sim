# vehicle dynamics simulator

## Get up and running
```bash
git clone https://github.com/abaeyens/vehicle_dynamics_sim.git
cd vehicle_dynamics_sim
git submodule update --init --recursive

./create_dot_env
docker compose build --pull
docker compose run --rm --name sim app bash

colcon build
source install/setup.bash

ros2 run vehicle_dynamics_sim sim_node

# and in another terminal
docker exec -it sim bash
ros2 run vehicle_dynamics_sim publish_some_velocities
```
