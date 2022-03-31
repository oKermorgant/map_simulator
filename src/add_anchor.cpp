// an executable to call a service from a launch file

#include <rclcpp/rclcpp.hpp>
#include <map_simulator/srv/add_anchor.hpp>

using map_simulator::srv::AddAnchor;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("add_anchor");

  // arguments reflect service call
  auto request = std::make_shared<AddAnchor::Request>();

  request->frame = node->declare_parameter("frame", "anchor");
  request->x = node->declare_parameter("x", 0.);
  request->y = node->declare_parameter("y", 0.);
  request->covariance_factor = node->declare_parameter("covariance", request->covariance_factor);
  request->covariance_factor_real = node->declare_parameter("covariance_real", request->covariance_factor_real);
  request->range_min = node->declare_parameter("range_min", request->range_min);
  request->range_max = node->declare_parameter("range_max", request->range_max);

  auto client = node->create_client<AddAnchor>("/simulator/add_anchor");

  client->wait_for_service();

  auto result_future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "service call failed :(");
    return 1;
  }
  [[maybe_unused]] auto result = result_future.get();
  RCLCPP_INFO(node->get_logger(), "successfully added anchor %s", request->frame.c_str());
  rclcpp::shutdown();
  return 0;
}
