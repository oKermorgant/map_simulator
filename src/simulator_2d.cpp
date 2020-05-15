#include <ros/ros.h>
#include <simulation_2d/robot2d.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "simulator_2d");
  ros::NodeHandle nh;

  ros::Rate loop(20);
  Robot2D robot(nh, loop);

  while(ros::ok())
  {
    robot.move();
    robot.publish();

    loop.sleep();
    ros::spinOnce();
  }
}
