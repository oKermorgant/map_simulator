#include <map_simulator/robot.h>
#include <tf2_ros/transform_listener.h>
#include <urdf/model.h>
#include <opencv2/imgproc.hpp>
#include <thread>

using namespace std::chrono_literals;

namespace map_simulator
{

// robot-agnostic helpers
urdf::Model getModel(const std::string &urdf_xml)
{
  urdf::Model model;
  model.initString(urdf_xml);
  return model;
}

template <class Msg>
std::unique_ptr<ros::Publisher> createPublisher(ros::NodeHandle* node, std::string topic)
{
  return std::make_unique<ros::Publisher>(node->advertise<Msg>(topic, 10));
}

std::tuple<std::string, std::string, bool> decomposeBaseLink(const urdf::Model &model)
{
  std::string base_link(model.getRoot()->name);
  auto prefix_length(base_link.find("base_"));

  if(prefix_length != base_link.npos)
    return {base_link, base_link.substr(0, prefix_length), true};
  else
    return {base_link, "", false};
}

std::vector<std::string> getVaryingJoints(const urdf::Model &model)
{
  std::vector<std::string> names;
  for(const auto &[name,joint]: model.joints_)
  {
    if(joint->type != urdf::Joint::FIXED)
      names.push_back(name);
  }
  return names;
}

ros::NodeHandle* Robot::Robot::sim_node;
char Robot::n_robots = 0;
std::default_random_engine Robot::random_engine;
std::normal_distribution<double> Robot::unit_noise(0,1);

Robot::Robot(const std::string &robot_namespace, const Pose2D _pose, bool is_circle, double _radius, cv::Scalar _color, cv::Scalar _laser_color, double _linear_noise, double _angular_noise)
  : id(n_robots++), robot_namespace(robot_namespace), shape(is_circle ? Shape::Cirle : Shape::Square),
    pose{_pose}, linear_noise(_linear_noise), angular_noise(_angular_noise),
    laser_color(_laser_color), color(_color), radius(_radius)
{
  odom.twist.covariance.front() = linear_noise*linear_noise;
  odom.twist.covariance.back() = angular_noise*angular_noise;

  if(robot_namespace.back() != '/')
    this->robot_namespace += '/';
}

void Robot::publish(const ros::Time &stamp,
                    tf2_ros::TransformBroadcaster *br)
{
  odom.header.stamp = transform.header.stamp = stamp;

  // build odom angle & publish as msg + tf
  odom_pub->publish(odom);

  // build transform odom -> base link
  transform.transform.translation.x = odom.pose.pose.position.x;
  transform.transform.translation.y = odom.pose.pose.position.y;
  transform.transform.rotation = odom.pose.pose.orientation;
  br->sendTransform(transform);

  if(hasLaser())
  {
    scan.header.stamp = stamp;
    scan_pub->publish(scan);
  }

  if(js_pub.get() && stamp.sec > joint_states.header.stamp.sec+1)
  {
    joint_states.header.stamp = stamp;
    js_pub->publish(joint_states);
  }
}

std::pair<std::string,char> Robot::initFromURDF(bool force_scanner, bool zero_joints, bool static_tf)
{
  std::thread listen_description_5sec([=]()
  {
    auto t0 = ros::Time::now().sec;
    auto t = t0;
    // check for description for 5 sec, otherwise assume no description
    while(t - t0 < 5)
    {
      if(sim_node->hasParam(robot_namespace + "robot_description"))
      {
        std::string description;
        sim_node->getParam(robot_namespace + "robot_description", description);
        loadModel(description, force_scanner,zero_joints,static_tf);
        break;
      }
      t = ros::Time::now().sec;
      std::this_thread::sleep_for(1s);
    }
  });

  listen_description_5sec.detach();

  return {robot_namespace,id};
}

std::tuple<bool, uint, std::string> Robot::parseLaser(const std::string &urdf_xml)
{
  uint samples(100);
  std::string scan_topic("scan");

  TiXmlDocument doc;
  doc.Parse(urdf_xml.c_str());
  auto root = doc.RootElement();

  for(auto gazebo_elem = root->FirstChildElement("gazebo");
      gazebo_elem != nullptr;
      gazebo_elem = gazebo_elem->NextSiblingElement("gazebo"))
  {
    for(auto sensor_elem = gazebo_elem->FirstChildElement("sensor");
        sensor_elem != nullptr;
        sensor_elem =sensor_elem->NextSiblingElement("sensor"))
    {
      if(std::string(sensor_elem->Attribute("type")) == "ray")
      {
        readFrom(sensor_elem, {"ray", "scan", "horizontal", "samples"}, samples);
        readFrom(sensor_elem, {"ray", "scan", "horizontal", "min_angle"}, scan.angle_min);
        readFrom(sensor_elem, {"ray", "scan", "horizontal", "max_angle"}, scan.angle_max);
        readFrom(sensor_elem, {"ray", "range", "min"}, scan.range_min);
        readFrom(sensor_elem, {"ray", "range", "max"}, scan.range_max);
        readFrom(sensor_elem, {"plugin", "frameName"}, scan.header.frame_id);
        readFrom(sensor_elem, {"plugin", "topicName"}, scan_topic);
        return {true, samples, scan_topic};
      }
    }
  }
  return {false, samples, scan_topic};
}

void Robot::loadModel(const std::string &urdf_xml,
                      bool force_scanner,
                      bool zero_joints,
                      bool static_tf)
{
  const auto model(getModel(urdf_xml));

  // create odom publisher + tf broadcaster
  const auto [base_link, tf_prefix, base_link_standard] = decomposeBaseLink(model); {}
  if(!base_link_standard)
  {
    ROS_WARN(sim_node->getNamespace().c_str(), "Description in namespace ", robot_namespace.c_str(),
             " has root link named '", base_link.c_str(), "' -> cannot detect TF prefix, should start with 'base_'");
  }

  // try to find laser scanner as Gazebo plugin
  auto [scan_init, samples, scan_topic] = parseLaser(urdf_xml); {}

  // finalize scan message + publisher
  if(scan_init || force_scanner)
  {
    if(!scan_init)
    {
      scan.angle_max = 2*M_PI/3;
      scan.angle_min = -scan.angle_max;
      scan.range_min = 0.1;
      scan.range_max = 5;
      scan.header.frame_id = base_link;
    }
    scan.angle_increment = (scan.angle_max-scan.angle_min)/samples;
    scan.angle_max -= scan.angle_increment;
    scan.ranges.resize(samples, 0.);
    if(scan_topic[0] == '/')
      scan_topic = scan_topic.substr(1, scan_topic.npos);
    scan_pub = createPublisher<sensor_msgs::LaserScan>(sim_node, robot_namespace + scan_topic);
  }

  odom_pub = createPublisher<nav_msgs::Odometry>(sim_node, robot_namespace + "odom");
  odom.header.frame_id = transform.header.frame_id = tf_prefix + "odom";
  odom.child_frame_id = transform.child_frame_id = base_link;

  // cmd vel subscriber
  cmd_sub = std::make_unique<ros::Subscriber>(sim_node->subscribe<geometry_msgs::Twist>
                                              (robot_namespace + "cmd_vel",10,
                                               [this](geometry_msgs::TwistConstPtr msg)  {
                                                // save perfect command velocities here
                                                odom.twist.twist.linear.x = msg->linear.x;
                                                odom.twist.twist.angular.z = msg->angular.z;
                                              }));
  // get offset between base link and scanner
  if(scan.header.frame_id != base_link)
  {
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    auto t = ros::Time::now();
    double t0 = t.sec;
    ros::Rate rate(1);
    while(ros::ok() && t.sec-t0 < 10)
    {
      t = ros::Time::now();
      if(tfBuffer.canTransform(odom.child_frame_id, scan.header.frame_id, ros::Time(0)))
      {
        auto base_to_scan =
            tfBuffer.lookupTransform(odom.child_frame_id, scan.header.frame_id, ros::Time(0));

        laser_pose.x = base_to_scan.transform.translation.x;
        laser_pose.y = base_to_scan.transform.translation.y;
        laser_pose.theta = 2*atan2(base_to_scan.transform.rotation.z,
                                   base_to_scan.transform.rotation.w);
        break;
      }
      ros::spinOnce();
      rate.sleep();
    }
  }

  // optional setups
  if(zero_joints)
  {
    // check if there are actually non-fixed joints
    joint_states.name = getVaryingJoints(model);
    if(joint_states.name.size())
    {
      js_pub = createPublisher<sensor_msgs::JointState>(sim_node, robot_namespace + "joint_states");
      joint_states.position.resize(joint_states.name.size(), 0);
    }
  }

  if(static_tf)
  {
    static_tf_br = std::make_unique<tf2_ros::StaticTransformBroadcaster>();
    geometry_msgs::TransformStamped odom2map;
    odom2map.header.stamp = ros::Time::now();
    odom2map.header.frame_id = "map";
    odom2map.child_frame_id = odom.header.frame_id;
    odom2map.transform.translation.x = pose.x;
    odom2map.transform.translation.y = pose.y;
    odom2map.transform.rotation.z = sin(pose.theta/2);
    odom2map.transform.rotation.w = cos(pose.theta/2);
    static_tf_br->sendTransform(odom2map);
  }

  // stop listening to robot_description, we got what we wanted
  description_sub.reset();
}

void Robot::move(double dt)
{
  // update real pose with perfect velocity command
  pose.x += odom.twist.twist.linear.x*cos(pose.theta)*dt;
  pose.y += odom.twist.twist.linear.x*sin(pose.theta)*dt;
  pose.theta += odom.twist.twist.angular.z*dt;

  // add noise: command velocity to measured one
  odom.twist.twist.linear.x *= (1+linear_noise*unit_noise(random_engine));
  odom.twist.twist.angular.z *= (1+angular_noise*unit_noise(random_engine));

  // update noised odometry
  double theta = 2*atan2(odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
  theta += odom.twist.twist.angular.z*dt;
  odom.pose.pose.position.x += odom.twist.twist.linear.x*cos(theta)*dt;
  odom.pose.pose.position.y += odom.twist.twist.linear.x*sin(theta)*dt;

  odom.pose.pose.orientation.z = sin(theta/2);
  odom.pose.pose.orientation.w = cos(theta/2);

  // TODO compute pose covariance for info (not used by EKFs anyway)
}

bool Robot::collidesWith(int u, int v) const
{
  if(shape == Shape::Cirle)
  {
    return (u-pos_pix.x)*(u-pos_pix.x) + (v-pos_pix.y)*(v-pos_pix.y)
        < radius*radius;
  }
  const auto poly(contour());
  return cv::pointPolygonTest(poly, cv::Point{u,v}, false) != -1;
}

void Robot::display(cv::Mat &img) const
{
  if(shape == Shape::Cirle)
  {
    cv::circle(img, pos_pix, radius, color, -1);
    return;
  }
  cv::fillConvexPoly(img, contour(), color);
}
}
