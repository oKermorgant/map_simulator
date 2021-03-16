#include <map_simulator/simulator.h>
#include <rospack/rospack.h>

namespace map_simulator
{

using namespace  std::chrono;

SimulatorNode::SimulatorNode()
{

  Robot::sim_node = this;

  ros::NodeHandle priv("~");

  dt = 1./priv.param("rate", 20);

  rospack::Rospack rospack;
  std::string share_folder;
  rospack.find("map_simulator", share_folder);

  const auto map = priv.param("map", share_folder + "/example/house.yaml");
  const auto max_height = priv.param("max_height", 800);
  const auto max_width = priv.param("max_width", 1200);

  const auto use_display = priv.param("display", true);

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

  refresh_timer = createTimer(ros::Duration(dt),[&](auto &){refresh(ros::Time::now());});

  spawn_srv = advertiseService("/simulator/spawn", &SimulatorNode::addRobot, this);
}

bool SimulatorNode::addRobot(SpawnRequest & spec, SpawnResponse &)
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

  return true;
}

void SimulatorNode::removeRobotAt(int x, int y)
{
  robots.remove_if([&](const Robot &robot)
  {return robot.collidesWith(x,y);});
}

void SimulatorNode::refresh(const ros::Time &now)
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
