#ifndef ROBOT2D_H
#define ROBOT2D_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <simulation_2d/occupancy_grid.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tinyxml2.h>

class Robot2D
{
  ros::Publisher scan_pub, odom_pub;
  ros::Subscriber cmd_sub;
  nav_msgs::Odometry odom;
  OccupancyGrid occupancy_grid;
  sensor_msgs::LaserScan scan;
  tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transform;

  const double dt;
  double theta=0;

  void cmd_callback(const geometry_msgs::TwistConstPtr &cmd)
  {
    odom.twist.twist = *cmd;
  }

  void loadModel(ros::NodeHandle &nh);
  std::string parseSensorURDF(tinyxml2::XMLElement * sensor_elem);

  void waitForTransform(ros::NodeHandle &priv);

  template <typename T>
  void readFrom(tinyxml2::XMLElement * root,
                std::vector<std::string> tag_sequence,
                T & val)
  {
    if(tag_sequence.size() == 0)
    {
      std::stringstream ss(root->GetText());
      ss >> val;
      return;
    }
    readFrom(root->FirstChildElement(tag_sequence.front().c_str()),
            {tag_sequence.begin()+1, tag_sequence.end()},
             val);
  }

public:
  Robot2D(ros::NodeHandle &nh, const ros::Rate & loop);

  void move()
  {
    odom.pose.pose.position.x += odom.twist.twist.linear.x*cos(theta)*dt;
    odom.pose.pose.position.y += odom.twist.twist.linear.x*sin(theta)*dt;
    theta += odom.twist.twist.angular.z*dt;

    occupancy_grid.computeLaserScan(static_cast<float>(odom.pose.pose.position.x),
                                    static_cast<float>(odom.pose.pose.position.y),
                                    static_cast<float>(theta),
                                    scan);
  }

  void publish()
  {
    odom.header.stamp = scan.header.stamp = transform.header.stamp = ros::Time::now();

    // build odom angle & publish as msg + tf
    odom.pose.pose.orientation.w = cos(theta/2);
    odom.pose.pose.orientation.z = sin(theta/2);
    odom_pub.publish(odom);

    // build transform
    transform.transform.translation.x = odom.pose.pose.position.x;
    transform.transform.translation.y = odom.pose.pose.position.y;
    transform.transform.rotation = odom.pose.pose.orientation;
    br.sendTransform(transform);

    scan_pub.publish(scan);
  }
};

#endif // ROBOT2D_H
