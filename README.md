# simulation_2d

This package provides a 2D simulator for mobile robots. The environment is based on a standard ROS map. Robots move through a `cmd_vel` topic and may publish a laser scan.

The motivation is to have a light weight simulator to demonstrate and test navigation capabilities, while using a simple map-based environment. The simulation is based on OpenCV.

The included launch files use the `simple_launch` package.

## Running the simulation

Just launch `simulation2d_launch.py` by passing the `map` argument that should point to the yaml file of the map. 

Setting `map_server:=true` also runs the corresponding map server.

The Simulation node advertises a service to spawn robots in. This service is wrapped into the `spawn` executable.

## Spawning a robot

The `spawn` node should be run in the same namespace as the `robot_description` topic. This namespace is used to publish topics.

This node takes the following arguments, which reflect the request part of a `Spawn` service:

- `x`, `y`, `theta`: where to spawn the robot
- `radius`: the size of the robot (used for display + laser detection from other robots)
- `shape`: `circle` or `square` 
- `robot_color`: a length-3 RGB (0-255) int array
- `laser_color`: a length-3 RGB (0-255) int array
- `force_scanner`: a Boolean (default True) to publish laser scans even if the URDF does not contain any laser scanner
- `static_tf_odom`: a Boolean (default False) to run a static TF broadcaster of the map -> odom frame of this robot
- `zero_joints`:a Boolean (default False) to publish 0-values for all non-fixed joints of the robot

If a Gazebo laser plugin is found, the laser scans will be published with the same specifications.

## Unspawning robots

Just double-clic on the robot you want to remove.

## Robot topics

The `cmd_vel` topic is subscribed by the simulator, for each robot according to their namespaces.

By default, the `scan` topic is used to publish laser scans, unless another topic is explicitely found in the URDF as a Gazebo plugin.

## Spawning an obstacle

The `spawn` node will spawn anything at the requested pose / size. If no `robot_description` topic is found, `cmd_vel` and `scan` topics will not be initialized but the spawned entity will be detected in the laser scans of other robots.

## Examples

See the `example` folder to see how to:
- spawn R2D2 with default laser props
- spawn Turtlebot3 with Gazebo-specified laser props
- run RViz to display robots, laserscans and the map
