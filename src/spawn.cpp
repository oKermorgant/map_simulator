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
  if(shape == "square")
    request->shape = request->SHAPE_SQUARE;
  else if(shape == "rectangle")
    request->shape = request->SHAPE_RECTANGLE;
  else
    request->shape = request->SHAPE_CIRCLE;

  request->linear_noise = node->declare_parameter("linear_noise", 0.);
  request->angular_noise = node->declare_parameter("angular_noise", 0.);

  auto robot_color = node->declare_parameter("robot_color", std::vector<int64_t>{0,0,0});
  if(robot_color.size() != 3)
    robot_color = {0,0,0};
  std::copy(robot_color.begin(), robot_color.end(), request->robot_color.begin());

  auto laser_color = node->declare_parameter("laser_color", std::vector<int64_t>{255,0,0});
  if(laser_color.size() != 3)
    laser_color = {255,0,0};
  std::copy(laser_color.begin(), laser_color.end(), request->laser_color.begin());

  auto size = node->declare_parameter("size", std::vector<double>{0,0,0});
  if(size.size() != 3)
    size = {0,0,0};
  std::copy(size.begin(), size.end(), request->size.begin());

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
  [[maybe_unused]] auto result = result_future.get();
  RCLCPP_INFO(node->get_logger(), "successfully spawned robot in namespace %s", request->robot_namespace.c_str());
  rclcpp::shutdown();
  return 0;
}
