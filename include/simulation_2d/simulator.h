#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <rclcpp/rclcpp.hpp>
#include <simulation_2d/occupancy_grid.h>
#include <simulation_2d/robot.h>
#include <simulation_2d/srv/spawn.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace simulation_2d
{

using simulation_2d::srv::Spawn;

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
