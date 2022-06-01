#ifndef ROBOT_H
#define ROBOT_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <opencv2/core.hpp>
#include <tinyxml.h>
#include <random>

#ifdef WITH_ANCHORS
#include <anchor_msgs/msg/range_with_covariance.hpp>
#include <map_simulator/srv/add_anchor.hpp>
#endif

namespace map_simulator
{
    
#ifdef WITH_ANCHORS   
using Anchor = srv::AddAnchor::Request;
#endif

struct Pose2D
{
  double x=0, y=0, theta=0;
  inline void updateFrom(double vx, double vy, double wz, double dt)
  {
    theta += .5*wz*dt;
    x += (vx*cos(theta) - vy*sin(theta))*dt;
    y += (vx*sin(theta) + vy*cos(theta))*dt;
    theta += .5*wz*dt;
  }
};

class Robot
{
  enum class Shape{Cirle, Square};

  static char n_robots;
  static std::default_random_engine random_engine;
  static std::normal_distribution<double> unit_noise;
  static builtin_interfaces::msg::Time stamp;
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
  Pose2D pose;
  double linear_noise = 0, angular_noise = 0;
  // 2D laser offset / base_link
  Pose2D laser_pose;

  // optional publishers
  bool publish_gt{false};
  bool zero_joints = false;
  sensor_msgs::msg::JointState::SharedPtr joint_states;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub;
  static std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_br;
  static inline void initStaticTfBr()
  {
    if(!static_tf_br.get())
      static_tf_br = std::make_unique<tf2_ros::StaticTransformBroadcaster>(sim_node);
  }

  // anchors stuff
#ifdef WITH_ANCHORS
  rclcpp::Publisher<anchor_msgs::msg::RangeWithCovariance>::SharedPtr range_pub;
  anchor_msgs::msg::RangeWithCovariance rangeFrom(const Anchor &anchor);
#endif

  void loadModel(const std::string &urdf_xml, bool force_scanner, bool zero_joints, bool static_tf);

  template <typename T>
  static void readFrom(TiXmlElement * root,
                       std::vector<std::string> tag_sequence,
                       T & val)
  {
    if(tag_sequence.size() == 0)
    {
      if(root)
      {
        std::stringstream ss(root->GetText());
        ss >> val;
      }
      return;
    }
    readFrom(root->FirstChildElement(tag_sequence.front().c_str()),
    {tag_sequence.begin()+1, tag_sequence.end()},
             val);
  }

  std::tuple<bool, uint, std::string> parseLaser(const std::string &urdf_xml, const std::string &link_prefix);

public:

  Robot(const std::string &robot_namespace, const Pose2D _pose,
        bool is_circle, double _radius,
        cv::Scalar _color, cv::Scalar _laser_color,
        double _linear_noise, double _angular_noise);

  std::pair<std::string, char> initFromURDF(bool force_scanner, bool zero_joints, bool static_tf);

  bool operator==(const Robot &other) const
  {
    return id == other.id;
  }
  bool operator!=(const Robot &other) const
  {
    return id != other.id;
  }

  bool isTwin(const std::pair<std::string, char> &other) const
  {
    return robot_namespace == other.first && id != other.second;
  }

  // grid access
  sensor_msgs::msg::LaserScan scan;
  cv::Point2f pos_pix;
  cv::Scalar laser_color;
  cv::Scalar color;

  float radius;
  double x() const {return pose.x;}
  double y() const {return pose.y;}
  void display(cv::Mat &img) const;
  std::vector<cv::Point> contour() const
  {
    std::vector<cv::Point> poly;
    const double diagonal(radius / 1.414);
    for(auto corner: {0, 1, 2, 3})
    {
      float corner_angle = -pose.theta + M_PI/4 + M_PI/2*corner;
      poly.emplace_back(pos_pix.x + diagonal*cos(corner_angle),
                        pos_pix.y + diagonal*sin(corner_angle));
    }
    return poly;
  }

  bool collidesWith(int u, int v) const;

  double x_l() const {return x() + laser_pose.x*cos(laser_pose.theta) - laser_pose.y*sin(laser_pose.theta);}
  double y_l() const {return y() + laser_pose.x*sin(laser_pose.theta) + laser_pose.y*cos(laser_pose.theta);}
  float theta_l() const {return pose.theta + laser_pose.theta;}

  // shared among robots
  static rclcpp::Node* sim_node;
  static geometry_msgs::msg::TransformStamped pose_gt;
  inline static void refreshStamp() {stamp = sim_node->get_clock()->now();}
  inline static void publishStaticTF(const geometry_msgs::msg::TransformStamped &tr)
  {
    initStaticTfBr();
    static_tf_br->sendTransform(tr);
  }

  void move(double dt);

  // anchors stuff
#ifdef WITH_ANCHORS
  void publishRanges(const std::vector<Anchor> &anchors);
#endif  

  bool connected() const
  {
    return odom_pub.get();
  }

  bool hasLaser() const
  {
    return scan_pub.get();
  }

  void publish(tf2_ros::TransformBroadcaster &br);
};

}

#endif // ROBOT_H
