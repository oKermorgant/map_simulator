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
    color = color.substr(begin+1, end-begin-1);

  auto sep = ',';
  auto comma = color.find(sep);
  if(comma == color.npos)
  {
    sep = ' ';
    comma = color.find(sep);
  }

  if(comma != color.npos)
  {
    rgb.push_back(std::stoi(color.substr(0, comma)));
    color = color.substr(comma+1);
    comma = color.find(sep);
    if(comma != color.npos)
    {
      rgb.push_back(std::stoi(color.substr(0, comma)));
      rgb.push_back(std::stoi(color.substr(comma+1)));
      return rgb;
    }
  }
  return fallback;
}

void clearParams(ros::NodeHandle &nh)
{
  const auto ns(nh.getNamespace());
  const auto length = ns.size();
  std::vector<std::string> params;
  nh.getParamNames(params);
  for(const auto &param: params)
  {
    if(param.size() > length && param.substr(0, length) == ns)
      nh.deleteParam(param);
  }
}


using map_simulator::Spawn;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "spawner");
  auto nh{ros::NodeHandle("~")};

  // arguments reflect service call
  auto request = Spawn::Request();

  request.robot_namespace = ros::NodeHandle().getNamespace();

  request.x = nh.param("x", 0.);
  request.y = nh.param("y", 0.);
  request.theta = nh.param("theta", 0.);
  request.radius = nh.param("radius", 0.3);

  const auto shape = nh.param<std::string>("shape", "circle");
  request.shape = shape == "square" ? request.SHAPE_SQUARE : request.SHAPE_CIRCLE;

  request.linear_noise = nh.param("linear_noise", 0.);
  request.angular_noise = nh.param("angular_noise", 0.);

  auto robot_color = read_color(nh, "robot_color", std::vector<int>{0,0,0});
  auto laser_color = read_color(nh, "laser_color", std::vector<int>{255,0,0});

  std::copy(robot_color.begin(), robot_color.end(), request.robot_color.begin());

  if(laser_color.size() != 3)
    laser_color = {255,0,0};
  std::copy(laser_color.begin(), laser_color.end(), request.laser_color.begin());

  request.force_scanner = nh.param("force_scanner", true);
  request.static_tf_odom = nh.param("static_tf_odom", false);
  request.zero_joints = nh.param("zero_joints", false);

  auto client = nh.serviceClient<Spawn>("/simulator/spawn");
  client.waitForExistence();

  auto response = Spawn::Response();
  if(client.call(request, response))
  {
    ROS_INFO("successfully spawned robot in namespace %s", request.robot_namespace.c_str());
    clearParams(nh);
    return 0;
  }

  ROS_ERROR("service call failed :(");
  clearParams(nh);
  return 1;
}
