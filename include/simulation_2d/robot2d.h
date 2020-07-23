#ifndef ROBOT2D_H
#define ROBOT2D_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tinyxml.h>

#include <simulation_2d/occupancy_grid.h>

class Robot2D : public rclcpp::Node
{
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
  rclcpp::TimerBase::SharedPtr timer;

  nav_msgs::msg::Odometry odom;
  OccupancyGrid occupancy_grid;
  sensor_msgs::msg::LaserScan scan;
  tf2_ros::TransformBroadcaster br;
  geometry_msgs::msg::TransformStamped transform;

  const double dt;
  double theta=0;

  void loadModel();
  std::string parseSensorURDF(TiXmlElement * sensor_elem);

  void waitForTransform();

  template <typename T>
  void readFrom(TiXmlElement * root,
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
  Robot2D(uint rate);

  void move()
  {
    odom.pose.pose.position.x += odom.twist.twist.linear.x*cos(theta)*dt;
    odom.pose.pose.position.y += odom.twist.twist.linear.x*sin(theta)*dt;
    theta += odom.twist.twist.angular.z*dt;

    occupancy_grid.computeLaserScan(static_cast<float>(odom.pose.pose.position.x),
                                    static_cast<float>(odom.pose.pose.position.y),
                                    static_cast<float>(theta),
                                    scan.ranges);
  }

  void publish()
  {
    odom.header.stamp = scan.header.stamp = transform.header.stamp = get_clock()->now();

    // build odom angle & publish as msg + tf
    odom.pose.pose.orientation.w = cos(theta/2);
    odom.pose.pose.orientation.z = sin(theta/2);
    odom_pub->publish(odom);

    // build transform
    transform.transform.translation.x = odom.pose.pose.position.x;
    transform.transform.translation.y = odom.pose.pose.position.y;
    transform.transform.rotation = odom.pose.pose.orientation;
    br.sendTransform(transform);

    scan_pub->publish(scan);
  }
};

#endif // ROBOT2D_H
