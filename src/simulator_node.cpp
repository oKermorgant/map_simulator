#include <ros/ros.h>
#include <map_simulator/robot.h>
#include <map_simulator/simulator.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_simulator");
  map_simulator::SimulatorNode sim;
  ros::spin();
}
