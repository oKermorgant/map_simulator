// an executable to call a service from a launch file

#include <rclcpp/rclcpp.hpp>
#include <map_simulator/srv/spawn.hpp>

using map_simulator::srv::Spawn;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("spawner");

  // arguments reflect service call
  auto request = std::make_shared<Spawn::Request>();

  request->robot_namespace = node->get_namespace();
  request->x = node->declare_parameter("x", 0.);
  request->y = node->declare_parameter("y", 0.);
  request->theta = node->declare_parameter("theta", 0.);
  request->radius = node->declare_parameter("radius", 0.3);

  const auto shape = node->declare_parameter("shape", "circle");
  request->shape = shape == "square" ? request->SHAPE_SQUARE : request->SHAPE_CIRCLE;

  request->linear_noise = node->declare_parameter("linear_noise", 0.);
  request->angular_noise = node->declare_parameter("angular_noise", 0.);

  auto robot_color = node->declare_parameter("robot_color", std::vector<int64_t>{0,0,0});
  auto laser_color = node->declare_parameter("laser_color", std::vector<int64_t>{255,0,0});

  if(robot_color.size() != 3)
    robot_color = {0,0,0};
  std::copy(robot_color.begin(), robot_color.end(), request->robot_color.begin());

  if(laser_color.size() != 3)
    laser_color = {255,0,0};
  std::copy(laser_color.begin(), laser_color.end(), request->laser_color.begin());

  request->force_scanner= node->declare_parameter("force_scanner", true);
  request->static_tf_odom = node->declare_parameter("static_tf_odom", false);
  request->zero_joints = node->declare_parameter("zero_joints", false);

  auto client = node->create_client<Spawn>("/simulator/spawn");

  client->wait_for_service();

  auto result_future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "service call failed :(");
    return 1;
  }
  auto result = result_future.get();
  RCLCPP_INFO(node->get_logger(), "successfully spawned robot in namespace ", request->robot_namespace.c_str());
  rclcpp::shutdown();
  return 0;
}
