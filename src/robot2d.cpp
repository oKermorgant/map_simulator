#include <simulation_2d/robot2d.h>
#include <tf2_ros/transform_listener.h>
#include <urdf/model.h>

Robot2D::Robot2D(uint rate) : Node("simulation_2d"), br(this), dt(1./rate)
{
  // init odom
  get_parameter_or("x", odom.pose.pose.position.x, 0.);
  get_parameter_or("y", odom.pose.pose.position.y, 0.);
  get_parameter_or("theta", theta, 0.);

  odom.header.frame_id = transform.header.frame_id = "odom";

  odom_pub = create_publisher<nav_msgs::msg::Odometry>("odom",50);

  cmd_sub = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        [this](geometry_msgs::msg::Twist::UniquePtr msg) {
      odom.twist.twist = *msg;
});

  std::string map_file;
  get_parameter_or<std::string>("map", map_file, "/home/olivier/code/ros/src/ecn/mobro/ecn_navigation/maps/batS.yaml");
  float max_height(0), max_width(0);
  get_parameter_or("max_height", max_height, 0.f);
  get_parameter_or("max_width", max_width, 0.f);


  occupancy_grid.initMap(map_file, max_height, max_width);

  loadModel();
  waitForTransform();

  timer = create_wall_timer(std::chrono::seconds(rate),
                            [this]()
  {
    move();
    publish();
  });
}


void Robot2D::waitForTransform()
{
  float radius;
  get_parameter("radius", radius);

  // wait for transforms
  tf2_ros::Buffer tfBuffer(get_clock());
  tf2_ros::TransformListener tfListener(tfBuffer);

  auto t = get_clock()->now();
  double t0 = t.seconds();

  while(rclcpp::ok() && t.seconds()-t0 < 10)
  {
    std::cout << "Asking for tf " << odom.child_frame_id << " / " <<  scan.header.frame_id << std::endl;
    t = get_clock()->now();
    if(tfBuffer.canTransform(odom.child_frame_id, scan.header.frame_id, tf2::TimePointZero))
    {
      auto base_to_scan =
          tfBuffer.lookupTransform(odom.child_frame_id, scan.header.frame_id, tf2::TimePointZero);

      occupancy_grid.initScanOffset(base_to_scan.transform.translation.x,
                                    base_to_scan.transform.translation.y,
                                    2*atan2(base_to_scan.transform.rotation.z,
                                            base_to_scan.transform.rotation.w),
                                    radius);
      return;
    }
    rclcpp::sleep_for(std::chrono::seconds(1));
  }
}


void Robot2D::loadModel()
{
  std::string description;
  //declare_parameter("robot_description", description);
  //get_parameter("/robot_state_publisher/robot_description", description);
  std::cout << description << std::endl;

  // URDF to get root link
  urdf::Model model;
  //model.initString(description);

  model.initFile("/home/olivier/code/libs/ros2/src/turtlebot3/turtlebot3/turtlebot3/turtlebot3_description/urdf/turtlebot3_waffle_pi.urdf");
  odom.child_frame_id = transform.child_frame_id = model.getRoot()->name;

  // Extract gazebo laser sensor data
  TiXmlDocument doc;
  doc.LoadFile("/home/olivier/code/libs/ros2/src/turtlebot3/turtlebot3/turtlebot3/turtlebot3_description/urdf/turtlebot3_waffle_pi.urdf");
  //doc.Parse(description.c_str());
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
        std::cout << "Found ray sensor" << std::endl;
        scan_pub = create_publisher<sensor_msgs::msg::LaserScan>(parseSensorURDF(sensor_elem),10);
        return;
      }
    }
  }
}


std::string Robot2D::parseSensorURDF(TiXmlElement *sensor_elem)
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

  occupancy_grid.initScanSensor(scan.angle_min, scan.angle_increment, scan.range_max);

  return scan_topic;
}

