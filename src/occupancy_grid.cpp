#include <simulation_2d/occupancy_grid.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <experimental/filesystem>

namespace fs = std::experimental::filesystem;

void OccupancyGrid::initMap(const std::string & map_file, float max_height, float max_width)
{
  YAML::Node map_data = YAML::LoadFile(map_file);

  // map registration
  resolution = map_data["resolution"].as<float>();
  const std::vector<float> origin(map_data["origin"].as<std::vector<float>>());
  x0 = origin[0];
  y0 = origin[1];

  // load and clean image
  std::string image(map_data["image"].as<std::string>());
  if(!fs::exists(image))
  {
    // relative to map file
    image = fs::path(map_file).remove_filename().string() + "/" + image;
  }

  occ_map = cv::imread(image, cv::IMREAD_GRAYSCALE);

  const double free_thr(map_data["free_thresh"].as<double>());
  const double occ_thr(map_data["occupied_thresh"].as<double>());
  const bool negate(map_data["negate"].as<int>());

  for(int i=0; i<occ_map.rows; i++)
  {
      for(int j=0; j<occ_map.cols; j++)
      {
       auto & pix(occ_map.at<uchar>(i,j));
       const double p = negate ? pix/255. : (255.-pix)/255.;

       if(p > occ_thr)
         pix = 0;
       else if(p < free_thr)
         pix = 255;
       else
       {
         pix = 255 - uchar(255*p);
       }
      }
  }

  // deal with max dimensions
  float scale = std::min(max_height/occ_map.rows, max_width/occ_map.cols);
  if(scale < 1)
  {
    cv::resizeWindow("Simulator 2D", int(scale*occ_map.rows), int(scale*occ_map.cols));
    std::cout << "Resizing display from " <<occ_map.cols << " x " << occ_map.rows
              << " to " << int(scale*occ_map.cols) << " x " << int(scale*occ_map.rows) << std::endl;

  }

  cv::cvtColor(occ_map, base_map, cv::COLOR_GRAY2BGR);




}

void OccupancyGrid::computeLaserScan(const float xr, const float yr, const float thetar,
                                     sensor_msgs::LaserScan &scan)
{
  cv::Mat scan_img = base_map.clone();

  // get laser origin in image coordinates
  const auto origin = pointFrom(xr + xs*cos(thetar) - ys*sin(thetar),
                                yr + xs*sin(thetar) + ys*cos(thetar));

  float angle(static_cast<float>(thetar) + thetas + scan.angle_min);
  const auto max_pix_range = int(scan.range_max / resolution);

  for(auto &range: scan.ranges)
  {
    range = 0.;
    const float c(cos(angle));
    const float s(sin(angle));

    // ray tracing within image
    for(int k = 0; k < max_pix_range; ++k)
    {
      const auto u = int(origin.x + k*c);
      const auto v = int(origin.y - k*s);

      if(u < 0 || v < 0 || u >= occ_map.cols || v >= occ_map.rows)
        break;

      if(occ_map.at<uchar>(v, u) == 0)
      {
        range = k*resolution;
        cv::line(scan_img, origin, {u, v}, laser_color);
        break;
      }
    }
    angle += scan.angle_increment;
  }
  
    // display robot
  cv::circle(scan_img, pointFrom(xr, yr), radius, robot_color, -1);

  cv::imshow("Simulator 2D", scan_img);
  cv::waitKey(1);
}
