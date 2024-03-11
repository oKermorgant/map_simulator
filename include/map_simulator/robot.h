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
#include <tinyxml2.h>
#include <random>

#include <map_simulator/srv/spawn.hpp>
#include <map_simulator/srv/add_anchor.hpp>

#ifdef WITH_BUILTIN_RANGE
#include <map_simulator/msg/range.hpp>
using map_simulator::msg::Range;
#else
#include <sensor_msgs/msg/range.hpp>
using sensor_msgs::msg::Range;
#endif

namespace map_simulator
{

using Anchor = srv::AddAnchor::Request;

struct Pose2D
{
  double x=0, y=0, theta=0;
  inline Pose2D(double x=0, double y=0, double theta=0) : x{x}, y{y}, theta{theta} {}
  inline Pose2D(const geometry_msgs::msg::Pose &pose)
    : x{pose.position.x}, y{pose.position.y}, theta{2*atan2(pose.orientation.z, pose.orientation.w)}
  {}
  inline void updateFrom(double vx, double vy, double wz, double dt)
  {
    theta += .5*wz*dt;
    x += (vx*cos(theta) - vy*sin(theta))*dt;
    y += (vx*sin(theta) + vy*cos(theta))*dt;
    theta += .5*wz*dt;
  }

  inline void writeTo(geometry_msgs::msg::TransformStamped &tr) const
  {
    tr.transform.translation.x = x;
    tr.transform.translation.y = y;
    tr.transform.rotation.w = cos(theta/2);
    tr.transform.rotation.z = sin(theta/2);
  }
  inline void writeTo(geometry_msgs::msg::Pose &pose) const
  {
    pose.position.x = x;
    pose.position.y = y;
    pose.orientation.w = cos(theta/2);
    pose.orientation.z = sin(theta/2);
  }
};

class Robot
{        
  static builtin_interfaces::msg::Time stamp;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_sub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;

  inline static constexpr auto CIRCLE = srv::Spawn::Request::SHAPE_CIRCLE;
  inline static constexpr auto SQUARE = srv::Spawn::Request::SHAPE_SQUARE;
  inline static constexpr auto RECTANGLE = srv::Spawn::Request::SHAPE_RECTANGLE;

  nav_msgs::msg::Odometry odom;
  geometry_msgs::msg::TransformStamped transform, initial_pose;

  // robot specs
  std::string robot_namespace;
  Pose2D pose;
  double linear_noise = 0, angular_noise = 0;
  // 2D laser offset / base_link
  Pose2D laser_pose;

  // optional publishers
  bool zero_joints = false;
  sensor_msgs::msg::JointState::SharedPtr joint_states;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub;

  // anchors stuff
  rclcpp::Publisher<Range>::SharedPtr range_pub;
  Range rangeFrom(const Anchor &anchor);

  void loadModel(const std::string &urdf_xml, bool force_scanner, bool zero_joints, bool static_tf);

  template <typename T>
  static void readFrom(tinyxml2::XMLElement* root,
                       std::vector<std::string> tag_sequence,
                       T & val)
  {
    if(!root)
      return;

    if(tag_sequence.empty())
    {
      std::stringstream ss(root->GetText());
      ss >> val;
      return;
    }
    readFrom(root->FirstChildElement(tag_sequence.front().c_str()),
             {tag_sequence.begin()+1, tag_sequence.end()},
             val);
  }

  std::tuple<bool, uint, std::string> parseLaser(const std::string &urdf_xml, const std::string &link_prefix);

public:

  Robot(const std::string &robot_namespace, const Pose2D _pose,
        const std::array<double, 3> &size,
        cv::Scalar _color, cv::Scalar _laser_color,
        double _linear_noise, double _angular_noise);

  inline void resetTo(const Pose2D _pose)
  {    
    pose = _pose;
    if(!initial_pose.child_frame_id.empty())
      pose.writeTo(initial_pose);
    // also reset odom
    odom.pose.pose = geometry_msgs::msg::Pose();
    odom.twist.twist = geometry_msgs::msg::Twist();
  }

  void initFromURDF(bool force_scanner, bool zero_joints, bool static_tf);

  bool operator==(const std::string &robot_namespace) const
  {
    return this->robot_namespace == robot_namespace;
  }
  bool operator!=(const Robot &other) const
  {
    return robot_namespace != other.robot_namespace;
  }

  // grid access
  sensor_msgs::msg::LaserScan scan;
  cv::Point2f pos_pix;
  cv::Scalar laser_color;
  cv::Scalar color;

  std::array<double, 3> size;
  std::vector<cv::Point> contour;

  double x() const {return pose.x;}
  double y() const {return pose.y;}
  void write(cv::Mat &img) const;

  inline double radius() const {return size[0];}

  void updateContour()
  {
    const int W(size[0]/2);
    const int L(size[1]/2);
    const int O(size[2]);
    contour = {{-L+O, -W},
               {L+O, -W},
               {L+O, W},
               {-L+O, W}};
    const auto c{cos(pose.theta)};
    const auto s{sin(pose.theta)};
    for(auto &p: contour)
      p = {int(pos_pix.x + c*p.x + s*p.y), int(pos_pix.y - s*p.x + c*p.y)};
  }

  inline bool isCircle() const {return size[1] == 0.;}

  bool collidesWith(int u, int v) const;

  double x_l() const {return x() + laser_pose.x*cos(laser_pose.theta) - laser_pose.y*sin(laser_pose.theta);}
  double y_l() const {return y() + laser_pose.x*sin(laser_pose.theta) + laser_pose.y*cos(laser_pose.theta);}
  float theta_l() const {return pose.theta + laser_pose.theta;}

  // shared among robots
  static rclcpp::Node* sim_node;
  inline static void refreshStamp() {stamp = sim_node->get_clock()->now();}
  inline static void publishStaticTF(const geometry_msgs::msg::TransformStamped &tr)
  {
    static tf2_ros::StaticTransformBroadcaster static_tf_br(sim_node);
    static_tf_br.sendTransform(tr);
  }

  void move(double dt);

  // anchors stuff
  void publishRanges(const std::vector<Anchor> &anchors);

  inline bool connected() const
  {
    return odom_pub.get() != nullptr;
  }

  inline bool hasLaser() const
  {
    return scan_pub.get() != nullptr;
  }

  void publish(tf2_ros::TransformBroadcaster &br);
};

}

#endif // ROBOT_H
