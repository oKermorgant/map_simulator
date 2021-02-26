// an executable to call a service from a launch file


#include <ros/ros.h>
#include <map_simulator/Spawn.h>

std::vector<int> read_color(ros::NodeHandle &node, std::string param, std::vector<int> fallback)
{
  auto color = node.param<std::string>(param, "");

  std::vector<int> rgb;
  // check formatting
  auto begin = color.find("[");
  auto end = color.find("]");
  if(begin != color.npos && end != color.npos)
  {
    color = color.substr(begin+1, end-begin-1);

    auto comma = color.find(",");
    if(comma != color.npos)
    {
      rgb.push_back(std::stoi(color.substr(0, comma)));
      color = color.substr(comma+1);
      comma = color.find(",");
      if(comma != color.npos)
      {
        rgb.push_back(std::stoi(color.substr(0, comma)));
        rgb.push_back(std::stoi(color.substr(comma+1)));
        return rgb;
      }
    }
  }
  return fallback;
}


using map_simulator::Spawn;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "spawner");
  auto node{ros::NodeHandle("~")};

  // arguments reflect service call
  auto request = Spawn::Request();

  request.robot_namespace = ros::NodeHandle().getNamespace();

  request.x = node.param("x", 0.);
  request.y = node.param("y", 0.);
  request.theta = node.param("theta", 0.);
  request.radius = node.param("radius", 0.3);

  const auto shape = node.param<std::string>("shape", "circle");
  request.shape = shape == "square" ? request.SHAPE_SQUARE : request.SHAPE_CIRCLE;

  request.linear_noise = node.param("linear_noise", 0.);
  request.angular_noise = node.param("angular_noise", 0.);

  auto robot_color = read_color(node, "robot_color", std::vector<int>{0,0,0});
  auto laser_color = read_color(node, "laser_color", std::vector<int>{255,0,0});

  std::copy(robot_color.begin(), robot_color.end(), request.robot_color.begin());

  if(laser_color.size() != 3)
    laser_color = {255,0,0};
  std::copy(laser_color.begin(), laser_color.end(), request.laser_color.begin());

  request.force_scanner= node.param("force_scanner", true);
  request.static_tf_odom = node.param("static_tf_odom", false);
  request.zero_joints = node.param("zero_joints", false);

  auto client = node.serviceClient<Spawn>("/simulator/spawn");
  client.waitForExistence();

  auto response = Spawn::Response();
  if(client.call(request, response))
  {
    ROS_INFO("successfully spawned robot in namespace %s", request.robot_namespace.c_str());
    return 0;
  }

  ROS_ERROR("service call failed :(");
  return 1;
}
