#ifndef ROBOT_H
#define ROBOT_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <opencv2/core.hpp>
#include <tinyxml.h>

namespace simulation_2d
{

class Robot
{
  enum class Shape{Cirle, Square};

  static char n_robots;
  char id;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_sub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;

  nav_msgs::msg::Odometry odom;
  geometry_msgs::msg::TransformStamped transform;

  // robot specs
  std::string robot_namespace;
  Shape shape;
  double theta;
  // laser specs
  double scanner_x=0, scanner_y=0, scanner_theta=0;  // 2D laser offset / base_link

  void loadModel(const std::string &urdf_xml, bool force_scanner);

  template <typename T>
  static void readFrom(TiXmlElement * root,
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

  // parsing functions
  std::tuple<bool, uint, std::string> parseLaser(const std::string &urdf_xml);
  std::tuple<std::string,std::string> decomposeBaseLink(const std::string &urdf_xml) const;

public:

  Robot(std::string robot_namespace, double _x, double _y, double _theta, bool is_circle, double _radius, cv::Scalar _color, cv::Scalar _laser_color);

  void initFromURDF(bool force_scanner);

  bool operator==(const Robot &other) const
  {
    return id == other.id;
  }
  bool operator!=(const Robot &other) const
  {
    return id != other.id;
  }

  // grid access
  sensor_msgs::msg::LaserScan scan;
  cv::Point2f pos_pix;
  cv::Scalar laser_color;
  cv::Scalar color;

  float radius;
  double x() const {return odom.pose.pose.position.x;}
  double y() const {return odom.pose.pose.position.y;}
  float angle() const {return theta;}
  void display(cv::Mat &img) const;
  std::vector<cv::Point> contour() const
  {
    std::vector<cv::Point> poly;
    const double diagonal(radius / 1.414);
    for(auto corner: {0, 1, 2, 3})
    {
      float corner_angle = theta + M_PI/4 + M_PI/2*corner;
      poly.emplace_back(pos_pix.x + diagonal*cos(corner_angle),
                        pos_pix.y + diagonal*sin(corner_angle));
    }
  return poly;
  }

  bool collidesWith(int u, int v) const;

  double x_l() const {return x() + scanner_x*cos(scanner_theta) - scanner_y*sin(scanner_theta);}
  double y_l() const {return y() + scanner_x*sin(scanner_theta) + scanner_y*cos(scanner_theta);}
  float theta_l() const {return theta + scanner_theta;}

  // shared among robots
  static rclcpp::Node* sim_node;

  void move(double dt)
  {
    odom.pose.pose.position.x += odom.twist.twist.linear.x*cos(theta)*dt;
    odom.pose.pose.position.y += odom.twist.twist.linear.x*sin(theta)*dt;
    theta += odom.twist.twist.angular.z*dt;
  }

  bool connected() const
  {
    return odom_pub.get();
  }

  bool has_laser() const
  {
    return scan_pub.get();
  }

  void publish(const builtin_interfaces::msg::Time &stamp,
               tf2_ros::TransformBroadcaster *br)
  {
    odom.header.stamp = scan.header.stamp = transform.header.stamp = stamp;

    // build odom angle & publish as msg + tf
    odom.pose.pose.orientation.w = cos(theta/2);
    odom.pose.pose.orientation.z = sin(theta/2);
    odom_pub->publish(odom);

    // build transform
    transform.transform.translation.x = odom.pose.pose.position.x;
    transform.transform.translation.y = odom.pose.pose.position.y;
    transform.transform.rotation = odom.pose.pose.orientation;
    br->sendTransform(transform);

    if(has_laser())
      scan_pub->publish(scan);
  }
};

}

#endif // ROBOT_H
