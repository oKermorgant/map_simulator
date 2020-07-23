#include <simulation_2d/robot2d.h>


int main(int argc, char** argv)
{
  const uint rate(20);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Robot2D>(rate));
  rclcpp::shutdown();
  return 0;
}
