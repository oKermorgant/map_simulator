#include <simulation_2d/robot.h>
#include <tf2_ros/transform_listener.h>
#include <urdf/model.h>
#include <opencv2/imgproc.hpp>

namespace simulation_2d
{

rclcpp::Node* Robot::Robot::sim_node;
char Robot::n_robots = 0;

Robot::Robot(std::string robot_namespace, double _x, double _y, double _theta, bool is_circle, double _radius, cv::Scalar _color, cv::Scalar _laser_color)
  : id(n_robots++), robot_namespace(robot_namespace), shape(is_circle ? Shape::Cirle : Shape::Square),
    theta(_theta), laser_color(_laser_color), color(_color), radius(_radius)
{
  odom.pose.pose.position.x = _x;
  odom.pose.pose.position.y = _y;

  if(robot_namespace.back() != '/')
    this->robot_namespace += '/';
}

void Robot::initFromURDF(bool force_scanner)
{
  // wait for this description
  rclcpp::QoS latching_qos(1);
  latching_qos.transient_local();
  description_sub = sim_node->create_subscription<std_msgs::msg::String>(
        robot_namespace + "robot_description", latching_qos,
        [&,force_scanner](std_msgs::msg::String::SharedPtr msg) {loadModel(msg->data, force_scanner);});
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

std::tuple<std::string, std::string> Robot::decomposeBaseLink(const std::string &urdf_xml) const
{
  urdf::Model model;
  model.initString(urdf_xml);
  std::string base_link(model.getRoot()->name);
  auto prefix_length(base_link.find("base_"));

  if(prefix_length != base_link.npos)
    return {base_link, base_link.substr(0, prefix_length)};
  else
  {
    RCLCPP_WARN(sim_node->get_logger(), "Description in namespace ", robot_namespace.c_str(),
                " has root link named '", base_link.c_str(), "' -> cannot detect TF prefix, should start with 'base_'");
    return {base_link, ""};
  }
}

void Robot::loadModel(const std::string &urdf_xml, bool force_scanner)
{
  // create odom publisher + tf broadcaster
  const auto [base_link, tf_prefix] = decomposeBaseLink(urdf_xml); {}

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
    scan_pub = sim_node->create_publisher<sensor_msgs::msg::LaserScan>(robot_namespace + scan_topic,10);
  }

  odom_pub = sim_node->create_publisher<nav_msgs::msg::Odometry>(robot_namespace + "odom", 10);
  odom.header.frame_id = transform.header.frame_id = tf_prefix + "odom";
  odom.child_frame_id = transform.child_frame_id = base_link;

  // cmd vel subscriber
  cmd_sub = sim_node->create_subscription<geometry_msgs::msg::Twist>
      (robot_namespace + "cmd_vel", 10, [this](geometry_msgs::msg::Twist::UniquePtr msg)
  {
      odom.twist.twist.linear.x = msg->linear.x;
      odom.twist.twist.angular.z = msg->angular.z;
});

  // get offset between base link and scanner
  if(scan.header.frame_id != base_link)
  {
    tf2_ros::Buffer tfBuffer(sim_node->get_clock());
    tf2_ros::TransformListener tfListener(tfBuffer);
    auto t = sim_node->get_clock()->now();
    double t0 = t.seconds();
    while(rclcpp::ok() && t.seconds()-t0 < 10)
    {
      t = sim_node->get_clock()->now();
      if(tfBuffer.canTransform(odom.child_frame_id, scan.header.frame_id, tf2::TimePointZero))
      {
        auto base_to_scan =
            tfBuffer.lookupTransform(odom.child_frame_id, scan.header.frame_id, tf2::TimePointZero);

        scanner_x = base_to_scan.transform.translation.x;
        scanner_y = base_to_scan.transform.translation.y;
        scanner_theta = 2*atan2(base_to_scan.transform.rotation.z,
                                base_to_scan.transform.rotation.w);
        break;
      }
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
  }
  // stop listening to robot_description
  description_sub.reset();
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

