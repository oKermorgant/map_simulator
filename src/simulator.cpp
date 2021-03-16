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

  auto share_folder = ament_index_cpp::get_package_share_directory("map_simulator");

  const auto map = declare_parameter<std::string>("map", share_folder + "/example/house.yaml");
  const auto max_height = declare_parameter<int>("max_height", 800);
  const auto max_width = declare_parameter<int>("max_width", 1200);
  const auto use_display = declare_parameter<bool>("display", true);

  grid.initMap(map, max_height, max_width, use_display);
  if(use_display)
  {
    cv::setMouseCallback("Simulator 2D", [](int event, int x, int y, int , void* node_ptr)
    {
      if(event != cv::EVENT_LBUTTONDBLCLK)
        return;
      ((SimulatorNode*)node_ptr)->removeRobotAt(x, y);
    }, this);
  }

  refresh_timer = create_wall_timer(milliseconds((long)(1000*dt)), [&]()
  {refresh(now());});

  spawn_srv = create_service<Spawn>
      ("/simulator/spawn", [&](const Spawn::Request::SharedPtr request, Spawn::Response::SharedPtr response)
  {
    addRobot(*request.get());
    (void) response;
  });
}


void SimulatorNode::addRobot(const Spawn::Request &spec)
{
  cv::Scalar color{(double)spec.robot_color[2],(double)spec.robot_color[1],(double)spec.robot_color[0]};
  cv::Scalar laser_color{(double)spec.laser_color[2],(double)spec.laser_color[1],(double)spec.laser_color[0]};

  robots.emplace_back(spec.robot_namespace, Pose2D{spec.x, spec.y, spec.theta},
                      spec.shape == spec.SHAPE_CIRCLE, spec.radius/grid.resolution(),
                      color, laser_color,
                      spec.linear_noise, spec.angular_noise);
  auto new_robot = robots.back().initFromURDF(spec.force_scanner,
                                              spec.zero_joints,
                                              spec.static_tf_odom);

  // remove any robot with same namespace (unless obstacle)
  robots.remove_if([=](const Robot &robot)
  {return robot.isTwin(new_robot);});

}

void SimulatorNode::removeRobotAt(int x, int y)
{
  robots.remove_if([&](const Robot &robot)
  {return robot.collidesWith(x,y);});
}

void SimulatorNode::refresh(const rclcpp::Time &now)
{
  for(auto &robot: robots)
  {
    if(robot.connected())
      robot.move(dt);
  }

  grid.computeLaserScans(robots);

  if(last_tf != now)
  {
    for(auto &robot: robots)
    {
      if(robot.connected())
        robot.publish(now, &br);
    }
    last_tf = now;
  }
}


}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(map_simulator::SimulatorNode)
