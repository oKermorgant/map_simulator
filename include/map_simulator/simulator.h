#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <rclcpp/rclcpp.hpp>
#include <map_simulator/occupancy_grid.h>
#include <map_simulator/robot.h>
#include <map_simulator/srv/spawn.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <map_simulator/srv/add_anchor.hpp>

namespace map_simulator
{

using map_simulator::srv::Spawn;

class SimulatorNode : public rclcpp::Node
{
public:
  SimulatorNode();

protected:
  OccupancyGrid grid;
  std::list<Robot> robots;
  rclcpp::TimerBase::SharedPtr refresh_timer, description_timer;

  double dt;

  tf2_ros::TransformBroadcaster br;
  rcl_time_point_value_t last_tf = 0;

  rclcpp::Service<Spawn>::SharedPtr spawn_srv;
  void addRobot(const Spawn::Request &spec);
  void removeRobotAt(int x, int y);

  void scanForDescriptions();

  rclcpp::Service<srv::AddAnchor>::SharedPtr anchor_srv;
  std::vector<Anchor> anchors;
  void addAnchor(const Anchor &anchor);

  void refresh(const rclcpp::Time &now);
};

}



#endif
