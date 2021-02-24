#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <ros/ros.h>
#include <map_simulator/occupancy_grid.h>
#include <map_simulator/robot.h>
#include <map_simulator/Spawn.h>
#include <tf2_ros/transform_broadcaster.h>

namespace map_simulator
{

using map_simulator::Spawn;

class SimulatorNode : public ros::NodeHandle
{
public:
  SimulatorNode();

protected:
  OccupancyGrid grid;
  std::list<Robot> robots;
  ros::Timer refresh_timer;

  double dt;

  tf2_ros::TransformBroadcaster br;

  ros::ServiceServer spawn_srv;

  bool addRobot(SpawnRequest &spec, SpawnResponse &);
  void removeRobotAt(int x, int y);

  void refresh(const ros::Time &now);
};




}



#endif
