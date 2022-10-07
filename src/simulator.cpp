#include <map_simulator/simulator.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace map_simulator
{

using namespace  std::chrono;
SimulatorNode::SimulatorNode(rclcpp::NodeOptions options)
  : rclcpp::Node("simulator", options), br(*this)
{
  Robot::sim_node = this;
  dt = 1./declare_parameter("rate", 20);

  const auto share_folder{ament_index_cpp::get_package_share_directory("map_simulator")};

  const auto map          {declare_parameter<std::string>("map", share_folder + "/example/house.yaml")};
  const auto max_height   {declare_parameter<int>("max_height", 800)};
  const auto max_width    {declare_parameter<int>("max_width", 1200)};
  const auto use_display  {declare_parameter<bool>("display", true)};

  grid.initMap(map, max_height, max_width, use_display);
  if(use_display)
  {
    cv::setMouseCallback("Simulator 2D", [](int event, int x, int y, int , void* node_ptr)
    {
      if(event != cv::EVENT_LBUTTONDBLCLK)
        return;
      ((SimulatorNode*)node_ptr)->removeRobotAt(x, y);
    },
    this);
  }

  refresh_timer = create_wall_timer(milliseconds(static_cast<long>(1000*dt)), [&]()
  {refresh(now());});

  spawn_srv = create_service<Spawn>
      ("/simulator/spawn",
       [&](const Spawn::Request::SharedPtr request, Spawn::Response::SharedPtr )
  {addRobot(*request);});

#ifdef WITH_ANCHORS
  anchor_srv = create_service<srv::AddAnchor>
      ("/simulator/add_anchor",
       [&](const srv::AddAnchor::Request::SharedPtr anchor, srv::AddAnchor::Response::SharedPtr)
  {addAnchor(*anchor);});
#endif
}

void SimulatorNode::addRobot(const Spawn::Request &spec)
{
  RCLCPP_INFO(get_logger(), "Adding new robot %s", spec.robot_namespace.c_str());
  cv::Scalar color(spec.robot_color[2],spec.robot_color[1],spec.robot_color[0]);
  cv::Scalar laser_color{(double)spec.laser_color[2],(double)spec.laser_color[1],(double)spec.laser_color[0]};
  auto robot_namespace = spec.robot_namespace;
  if(robot_namespace.back() != '/')
    robot_namespace += '/';

  if(const auto twin = std::find(robots.begin(), robots.end(), robot_namespace);
     twin != robots.end())
  {
    // remove duplicate robot before spawning the new one
    robots.erase(twin);
    RCLCPP_WARN(get_logger(), "overwriting prev robot %s", spec.robot_namespace.c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  robots.emplace_back(robot_namespace, Pose2D{spec.x, spec.y, spec.theta},
                      spec.shape == spec.SHAPE_CIRCLE, spec.radius/grid.resolution(),
                      color, laser_color,
                      spec.linear_noise, spec.angular_noise);
  robots.back().initFromURDF(spec.force_scanner,
                             spec.zero_joints,
                             spec.static_tf_odom);
}

void SimulatorNode::removeRobotAt(int x, int y)
{
  robots.remove_if([&](const Robot &robot)
  {return robot.collidesWith(x,y);});
}

#ifdef WITH_ANCHORS
void SimulatorNode::addAnchor(const Anchor &anchor)
{
  auto twin = std::find_if(anchors.begin(), anchors.end(),
                           [anchor](const auto &other){return anchor.frame == other.frame;});
  if(twin != anchors.end())
  {
    RCLCPP_WARN(get_logger(), "Cannot add %s: already exists", anchor.frame.c_str());
    return;
  }

  anchors.push_back(anchor);
  anchors.back().covariance_factor_real = sqrt(anchors.back().covariance_factor_real);
  geometry_msgs::msg::TransformStamped anchor_tf;
  anchor_tf.header.stamp = now();
  anchor_tf.header.frame_id = "map";
  anchor_tf.child_frame_id = anchor.frame + "_gt";
  anchor_tf.transform.translation.x = anchor.x;
  anchor_tf.transform.translation.y = anchor.y;
  Robot::publishStaticTF(anchor_tf);
}
#endif

void SimulatorNode::refresh(const rclcpp::Time &now)
{
  Robot::refreshStamp();

  for(auto &robot: robots)
  {
    if(robot.connected())
    {
      robot.move(dt);
#ifdef WITH_ANCHORS
      robot.publishRanges(anchors);
#endif
    }
  }

  grid.computeLaserScans(robots);

  if(last_tf != now.nanoseconds())
  {
    for(auto &robot: robots)
    {
      if(robot.connected())
        robot.publish(br);
    }
    last_tf = now.nanoseconds();
  }
}


}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(map_simulator::SimulatorNode)
