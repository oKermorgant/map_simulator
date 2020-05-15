#include <simulation_2d/robot2d.h>
#include <tf2_ros/transform_listener.h>
#include <urdf/model.h>

Robot2D::Robot2D(ros::NodeHandle &nh, const ros::Rate & loop) : dt(loop.expectedCycleTime().toSec())
{
  ros::NodeHandle priv("~");

  // init odom
  priv.param("x", odom.pose.pose.position.x, 0.);
  priv.param("y", odom.pose.pose.position.y, 0.);
  priv.param("theta", theta, 0.);

  odom.header.frame_id = transform.header.frame_id = "odom";

  odom_pub = nh.advertise<nav_msgs::Odometry>("odom",50);
  cmd_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &Robot2D::cmd_callback, this);

  std::string map_file;
  priv.getParam("map", map_file);
  float max_height(0), max_width(0);
  if(priv.hasParam("max_height"))
    priv.getParam("max_height", max_height);
  if(priv.hasParam("max_width"))
    priv.getParam("max_width", max_width);

  occupancy_grid.initMap(map_file, max_height, max_width);

  loadModel(nh);
  waitForTransform(priv);
}


void Robot2D::waitForTransform(ros::NodeHandle &priv)
{
  float radius;
  priv.getParam("radius", radius);

  // wait for transforms
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  auto t = ros::Time::now();
  double t0 = t.toSec();

  while(ros::ok() && t.toSec()-t0 < 10)
  {
    t = ros::Time::now();
    if(tfBuffer.canTransform(odom.child_frame_id, scan.header.frame_id, t))
    {
      geometry_msgs::TransformStamped base_to_scan =
          tfBuffer.lookupTransform(odom.child_frame_id, scan.header.frame_id, t);

      occupancy_grid.initScanOffset(base_to_scan.transform.translation.x,
                                    base_to_scan.transform.translation.y,
                                    2*atan2(base_to_scan.transform.rotation.z,
                                            base_to_scan.transform.rotation.w),
                                    radius);
      return;
    }
  }

  ROS_INFO("Could not find transform between %s and %s",
           scan.header.frame_id.c_str(),
           odom.child_frame_id.c_str());
}


void Robot2D::loadModel(ros::NodeHandle &nh)
{
  std::string description;
  nh.getParam("robot_description", description);

  // URDF to get root link
  urdf::Model model;
  model.initString(description);
  odom.child_frame_id = transform.child_frame_id = model.getRoot()->name;

  // Extract gazebo laser sensor data
  tinyxml2::XMLDocument doc;
  doc.Parse(description.c_str());
  auto root = doc.RootElement();
  // look for gazebo / sensor with type ray
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
        scan_pub = nh.advertise<sensor_msgs::LaserScan>(parseSensorURDF(sensor_elem),50);
        return;
      }
    }
  }
}


std::string Robot2D::parseSensorURDF(tinyxml2::XMLElement *sensor_elem)
{
  uint samples;
  readFrom(sensor_elem, {"ray", "scan", "horizontal", "samples"}, samples);
  readFrom(sensor_elem, {"ray", "scan", "horizontal", "min_angle"}, scan.angle_min);
  readFrom(sensor_elem, {"ray", "scan", "horizontal", "max_angle"}, scan.angle_max);
  scan.angle_increment = (scan.angle_max-scan.angle_min)/samples;
  scan.angle_max -= scan.angle_increment;
  scan.ranges.resize(samples, 0.);
  readFrom(sensor_elem, {"ray", "range", "min"}, scan.range_min);
  readFrom(sensor_elem, {"ray", "range", "max"}, scan.range_max);
  readFrom(sensor_elem, {"plugin", "frameName"}, scan.header.frame_id);

  std::string scan_topic;
  readFrom(sensor_elem, {"plugin", "topicName"}, scan_topic);
  return scan_topic;
}

