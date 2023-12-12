#include <map_simulator/occupancy_grid.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <experimental/filesystem>
#include <iostream>

namespace fs = std::experimental::filesystem;

namespace map_simulator
{

void OccupancyGrid::initMap(const std::string & map_file, float max_height, float max_width, bool use_display)
{
  auto map_data{YAML::LoadFile(map_file)};

  // map registration
  res = map_data["resolution"].as<float>();
  const auto origin(map_data["origin"].as<std::vector<float>>());
  x0 = origin[0];
  y0 = origin[1];

  // load and clean image
  auto image{map_data["image"].as<std::string>()};
  if(!fs::exists(image))
  {
    // relative to map file
    image = fs::path(map_file).replace_filename(image);
  }

  occ_map = cv::imread(image, cv::IMREAD_GRAYSCALE);

  const auto free_thr{map_data["free_thresh"].as<double>()};
  const auto occ_thr{map_data["occupied_thresh"].as<double>()};
  const bool negate(map_data["negate"].as<int>());

  for(int i=0; i<occ_map.rows; i++)
  {
    for(int j=0; j<occ_map.cols; j++)
    {
      auto & pix(occ_map.at<uchar>(i,j));
      const auto p = negate ? pix/255. : (255.-pix)/255.;

      if(p > occ_thr)
        pix = 0;
      else if(p < free_thr)
        pix = 255;
      else
        pix = 255 - uchar(255*p);
    }
  }

  this->use_display = use_display;
  if(use_display)
  {
    cv::namedWindow("Simulator 2D", cv::WINDOW_NORMAL);
    const float scale = std::min(max_height/occ_map.rows, max_width/occ_map.cols);
    if(scale != 0.f && scale < 1)
      cv::resizeWindow("Simulator 2D", int(scale*occ_map.cols), int(scale*occ_map.rows));
    else
      cv::resizeWindow("Simulator 2D", occ_map.cols, occ_map.rows);
  }

  cv::cvtColor(occ_map, map_img, cv::COLOR_GRAY2BGR);
}


void OccupancyGrid::computeLaserScans(std::list<Robot> &robots)
{
  if(use_display)
    out_img = map_img.clone();

  // update robot pixel coord
  for(auto &robot: robots)
    robot.pos_pix = pointFrom(robot.x(), robot.y());

  // TODO try to use threadpool here, but they have to work on the same image
  for(auto &robot: robots)
  {
    if(robot.hasLaser())
      computeLaserScan(robot, robots);
  }

  if(use_display)
  {
    for(const auto &robot: robots)
      robot.write(out_img);

    cv::imshow("Simulator 2D", out_img);
    cv::waitKey(1);
  }
}


void OccupancyGrid::computeLaserScan(Robot &robot, const std::list<Robot> &robots)
{
  const auto origin(pointFrom(robot.x_l(), robot.y_l()));
  auto &scan(robot.scan);

  auto angle(robot.theta_l() + scan.angle_min);
  const auto max_pix_range = int(scan.range_max / res);

  for(auto &range: scan.ranges)
  {
    range = 0.;
    const auto c{cos(angle)};
    const auto s{sin(angle)};
    angle += scan.angle_increment;

    // ray tracing within image
    for(int k = 0; k < max_pix_range; ++k)
    {
      const auto u{int(origin.x + k*c)};
      const auto v{int(origin.y - k*s)};

      if(u < 0 || v < 0 || u >= occ_map.cols || v >= occ_map.rows)
        break;

      const auto hit{std::any_of(robots.begin(), robots.end(),
                                 [&](const auto &other)
        {return robot != other && other.collidesWith(u,v);})};

      if(hit || occ_map.at<uchar>(v, u) == 0)
      {
        range = (k-0.5f)*res;
        if(use_display)
          cv::line(out_img, origin, {u, v}, robot.laser_color);
        break;
      }
    }
  }
}
}
