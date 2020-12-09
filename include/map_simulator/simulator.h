#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <rclcpp/rclcpp.hpp>
#include <map_simulator/occupancy_grid.h>
#include <map_simulator/robot.h>
#include <map_simulator/srv/spawn.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace map_simulator
{

using map_simulator::srv::Spawn;

class SimulatorNode : public rclcpp::Node
{
public:
  SimulatorNode(rclcpp::NodeOptions options);

protected:
  OccupancyGrid grid;
  std::list<Robot> robots;
  rclcpp::TimerBase::SharedPtr refresh_timer;

  double dt;

  tf2_ros::TransformBroadcaster br;

  rclcpp::Service<Spawn>::SharedPtr spawn_srv;

  void addRobot(const Spawn::Request &spec);
  void removeRobotAt(int x, int y);

  void refresh(const rclcpp::Time &now);
};




}



#endif