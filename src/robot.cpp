#include <map_simulator/robot.h>
#include <tf2_ros/transform_listener.h>
#include <urdf/model.h>
#include <opencv2/imgproc.hpp>

namespace map_simulator
{



void adaptNamespace(std::string &topic, std::string ns)
{
  if(ns[0] == '/')
    ns = ns.substr(1);

  if(topic.rfind(ns, 0) != 0)
    topic = ns + topic;
}

// robot-agnostic helpers
std::unique_ptr<const urdf::Model> getModel(const std::string &urdf_xml)
{
  auto model{std::make_unique<urdf::Model>()};
  model->initString(urdf_xml);
  return model;
}

std::tuple<std::string, std::string, bool> decomposeBaseLink(const urdf::Model &model)
{
  std::string base_link(model.getRoot()->name);
  auto prefix_length(base_link.find("base_"));

  if(prefix_length != base_link.npos)
    return {base_link.substr(prefix_length, base_link.npos), base_link.substr(0, prefix_length), true};
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

double withNoise(double val, double std)
{
  static std::default_random_engine random_engine;
  static std::normal_distribution<double> unit_noise(0,1);
  return val * (1+std*unit_noise(random_engine));
}


rclcpp::Node* Robot::Robot::sim_node;
builtin_interfaces::msg::Time Robot::stamp;

Robot::Robot(const std::string &robot_namespace, const Pose2D _pose, const std::array<double, 3> &size, cv::Scalar _color, cv::Scalar _laser_color, double _linear_noise, double _angular_noise)
  : robot_namespace(robot_namespace), pose{_pose},
    linear_noise(_linear_noise), angular_noise(_angular_noise),
    laser_color(_laser_color), color(_color), size(size)
{
}

void Robot::publish(tf2_ros::TransformBroadcaster &br)
{
  odom.header.stamp = transform.header.stamp = stamp;

  // build odom angle & publish as msg + tf
  odom_pub->publish(odom);

  // build transform odom -> base link
  transform.transform.translation.x = odom.pose.pose.position.x;
  transform.transform.translation.y = odom.pose.pose.position.y;
  transform.transform.rotation = odom.pose.pose.orientation;
  br.sendTransform(transform);

  if(hasLaser())
  {
    scan.header.stamp = stamp;
    scan_pub->publish(scan);
  }

  if(js_pub.get())
  {
    joint_states->header.stamp = stamp;
    js_pub->publish(*joint_states);
  }

  if(!static_tf)
  {
    geometry_msgs::msg::TransformStamped pose_gt;
    pose_gt.header.frame_id = "map";
    pose_gt.child_frame_id = odom.child_frame_id + "_gt";
    pose_gt.header.stamp = stamp;
    pose_gt.transform.translation.x = pose.x;
    pose_gt.transform.translation.y = pose.y;
    pose_gt.transform.rotation.w = cos(pose.theta/2.);
    pose_gt.transform.rotation.z = sin(pose.theta/2.);
    br.sendTransform(pose_gt);
  }
}

void Robot::initFromURDF(bool force_scanner, bool zero_joints, bool static_tf)
{
  // wait for this description
  rclcpp::QoS latching_qos(1);
  latching_qos.transient_local();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  RCLCPP_INFO(sim_node->get_logger(), "waiting for %srobot_description...", robot_namespace.c_str());
  description_sub = sim_node->create_subscription<std_msgs::msg::String>(
        robot_namespace + "robot_description", latching_qos,
        [&,force_scanner,zero_joints,static_tf](std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(sim_node->get_logger(), "Got %srobot_description", robot_namespace.c_str());
    loadModel(msg->data, force_scanner,zero_joints,static_tf);
  });
}

std::tuple<bool, uint, std::string> Robot::parseLaser(const std::string &urdf_xml, const std::string &link_prefix)
{
  uint samples(100);
  std::string scan_topic("scan");

  tinyxml2::XMLDocument xml;
  xml.Parse(urdf_xml.c_str());
  const auto root{xml.RootElement()};

  for(auto gazebo_elem = root->FirstChildElement("gazebo");
      gazebo_elem != nullptr;
      gazebo_elem = gazebo_elem->NextSiblingElement("gazebo"))
  {
    for(auto sensor_elem = gazebo_elem->FirstChildElement("sensor");
        sensor_elem != nullptr;
        sensor_elem =sensor_elem->NextSiblingElement("sensor"))
    {
      if(strcmp(sensor_elem->Attribute("type"), "ray") == 0)
      {
        readFrom(sensor_elem, {"ray", "scan", "horizontal", "samples"}, samples);
        readFrom(sensor_elem, {"ray", "scan", "horizontal", "min_angle"}, scan.angle_min);
        readFrom(sensor_elem, {"ray", "scan", "horizontal", "max_angle"}, scan.angle_max);
        readFrom(sensor_elem, {"ray", "range", "min"}, scan.range_min);
        readFrom(sensor_elem, {"ray", "range", "max"}, scan.range_max);
        // ROS 2 syntax
        readFrom(sensor_elem, {"plugin", "frame_name"}, scan.header.frame_id);
        readFrom(sensor_elem, {"plugin", "ros", "remapping"}, scan_topic);
        // extract actual topic name if any remapping
        const auto remap(scan_topic.find(":="));
        if(remap != scan_topic.npos)
          scan_topic = scan_topic.substr(remap+2);

        // add prefix to scan if not here, due to frame_prefix in robot_state_publisher
        adaptNamespace(scan.header.frame_id, link_prefix);
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
  const auto [base_link, tf_prefix, base_link_standard] = decomposeBaseLink(*model); {}
  if(!base_link_standard)
  {
    RCLCPP_WARN(sim_node->get_logger(),
                "Description in namespace %s has root link named '%s' -> cannot detect TF prefix, should start with 'base_'",
                robot_namespace.c_str(),
                (tf_prefix+base_link).c_str());
  }

  // link prefix is either tf_prefix if any, or robot namespace
  const auto link_prefix = tf_prefix.size() ? tf_prefix : robot_namespace.substr(1, robot_namespace.npos);

  // try to find laser scanner as Gazebo plugin
  auto [scan_init, samples, scan_topic] = parseLaser(urdf_xml, link_prefix); {}

  // finalize scan message + publisher
  if(scan_init || force_scanner)
  {
    if(!scan_init)
    {
      scan.angle_max = 2*M_PI/3;
      scan.angle_min = -scan.angle_max;
      scan.range_min = 0.1;
      scan.range_max = 5;
      scan.header.frame_id = link_prefix + base_link;
    }
    scan.angle_increment = (scan.angle_max-scan.angle_min)/samples;
    scan.angle_max -= scan.angle_increment;
    scan.ranges.resize(samples, 0.);
    if(scan_topic[0] == '/')
      scan_topic = scan_topic.substr(1, scan_topic.npos);
    adaptNamespace(scan_topic, robot_namespace);
    scan_pub = sim_node->create_publisher<sensor_msgs::msg::LaserScan>
        (scan_topic,rclcpp::SensorDataQoS());
  }

  odom_pub = sim_node->create_publisher<nav_msgs::msg::Odometry>(robot_namespace + "odom", 10);
  odom.header.frame_id = transform.header.frame_id = link_prefix + "odom";
  odom.child_frame_id = transform.child_frame_id = link_prefix + base_link;

  // cmd vel subscriber
  cmd_sub = sim_node->create_subscription<geometry_msgs::msg::Twist>
      (robot_namespace + "cmd_vel", 10, [this](geometry_msgs::msg::Twist::UniquePtr msg)
  {
      // save perfect command velocities in odom anyway
      odom.twist.twist.linear.x = msg->linear.x;
      odom.twist.twist.linear.y = msg->linear.y;
      odom.twist.twist.angular.z = msg->angular.z;
});

  // get offset between base link and scanner
  if(scan_pub.get() && scan.header.frame_id != link_prefix + base_link)
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

        laser_pose.x = base_to_scan.transform.translation.x;
        laser_pose.y = base_to_scan.transform.translation.y;
        laser_pose.theta = 2*atan2(base_to_scan.transform.rotation.z,
                                   base_to_scan.transform.rotation.w);
        break;
      }
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
  }

  // optional setups
  if(zero_joints)
  {
    // check if there are actually non-fixed joints
    joint_states = std::make_shared<sensor_msgs::msg::JointState>();
    joint_states->name = getVaryingJoints(*model);
    if(joint_states->name.size() == 0)
    {
      // pointless
      joint_states.reset();
    }
    else
    {
      js_pub = sim_node->create_publisher<sensor_msgs::msg::JointState>(robot_namespace + "joint_states", 10);
      joint_states->position.resize(joint_states->name.size(), 0);
    }
  }

  if(static_tf)
  {
    geometry_msgs::msg::TransformStamped odom2map;
    odom2map.header.stamp = sim_node->now();
    odom2map.header.frame_id = "map";
    odom2map.child_frame_id = odom.header.frame_id;
    odom2map.transform.translation.x = pose.x;
    odom2map.transform.translation.y = pose.y;
    odom2map.transform.rotation.z = sin(pose.theta/2);
    odom2map.transform.rotation.w = cos(pose.theta/2);
    publishStaticTF(odom2map);
  }
  this->static_tf = static_tf;

  // stop listening to robot_description, we got what we wanted
  description_sub.reset();
}

void Robot::move(double dt)
{
  // update real pose with noised velocity command
  auto &vx(odom.twist.twist.linear.x);
  auto &vy(odom.twist.twist.linear.y);
  auto &wz(odom.twist.twist.angular.z);

  auto vxn{withNoise(vx, linear_noise)};
  auto vyn{withNoise(vy, linear_noise)};
  auto wzn{withNoise(wz, angular_noise)};

  pose.updateFrom(vxn, vyn, wzn, dt);

  // write actual covariance, proportional to velocity
  odom.twist.covariance[0] = std::max(0.0001, std::abs(vx)*linear_noise*linear_noise);
  odom.twist.covariance[7] = std::max(0.0001, std::abs(vy)*linear_noise*linear_noise);
  odom.twist.covariance[35] = std::max(0.0001, std::abs(wz)*angular_noise*angular_noise);

  // use another noise for odometry unless perfect odom expected
  if(!static_tf)
  {
    vx = vxn = withNoise(vx, linear_noise);
    vy = vyn = withNoise(vy, linear_noise);
    wz = wzn = withNoise(wz, linear_noise);
  }

  // update noised odometry
  Pose2D rel_pose{odom.pose.pose.position.x, odom.pose.pose.position.y,
        2*atan2(odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)};
  rel_pose.updateFrom(vxn, vyn, wzn, dt);

  odom.pose.pose.position.x = rel_pose.x;
  odom.pose.pose.position.y = rel_pose.y;
  odom.pose.pose.orientation.z = sin(rel_pose.theta/2);
  odom.pose.pose.orientation.w = cos(rel_pose.theta/2);

  // TODO compute pose covariance for info (not used by EKFs anyway)
}

bool Robot::collidesWith(int u, int v) const
{
  if(isCircle())
  {
    return (u-pos_pix.x)*(u-pos_pix.x) + (v-pos_pix.y)*(v-pos_pix.y)
        < radius()*radius();
  }
  const auto poly(contour());
  return cv::pointPolygonTest(poly, cv::Point{u,v}, false) != -1;
}

void Robot::write(cv::Mat &img) const
{
  if(isCircle())
  {
    cv::circle(img, pos_pix, radius(), color, -1);
    return;
  }
  cv::fillConvexPoly(img, contour(), color);
}

Range Robot::rangeFrom(const Anchor &anchor)
{
  Range range;
  range.header.stamp = stamp;
  range.header.frame_id = anchor.frame_id;
  range.field_of_view = 2*M_PI;
  range.max_range = anchor.max_range;
  range.min_range = anchor.min_range;
  const auto dx{anchor.x-pose.x};
  const auto dy{anchor.y-pose.y};
  const auto real_range{sqrt(dx*dx + dy*dy)};
  range.range = withNoise(real_range, anchor.variance_factor_real);
  range.variance = anchor.variance_factor * real_range;
  return range;
}


void Robot::publishRanges(const std::vector<Anchor> &anchors)
{
  if(anchors.empty())
    return;
  if(!range_pub)
    range_pub = sim_node->create_publisher<Range>
        (robot_namespace + "ranges", 10);

  for(const auto &anchor: anchors)
    range_pub->publish(rangeFrom(anchor));
}

}
