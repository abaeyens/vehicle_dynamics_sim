# vehicle dynamcics simulator

## Get up and running
```bash
git clone TODO
cd vehicle_dynamics_sim
git submodule update --init --recursive

echo -e USER_ID=$(id -u $USER)\\nGROUP_ID=$(id -g $USER) >> .env
docker compose build --pull
docker compose run --rm app bash

colcon build
source install/setup.bash
colcon test
```
