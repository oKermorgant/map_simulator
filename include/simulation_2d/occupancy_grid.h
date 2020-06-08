#ifndef OCCUPANCYGRID_H
#define OCCUPANCYGRID_H

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

class OccupancyGrid
{
  cv::Mat base_map, occ_map;
  float resolution;
  float x0, y0;
  float xs, ys, thetas;
  int radius;
  float angle_min, angle_increment, range_max;

  const cv::Scalar robot_color{0,0,0};
  const cv::Scalar laser_color{0,0,255};

  template <typename Numeric>
  cv::Point2f pointFrom(Numeric x, Numeric y)
  {
    return {(static_cast<float>(x)-x0)/resolution,
          occ_map.rows-(static_cast<float>(y)-y0)/resolution};
  }

public:
  OccupancyGrid()
  {
    cv::namedWindow("Simulator 2D", cv::WINDOW_NORMAL);
  }
  void initMap(const std::string &map_file, float max_height, float max_width);
  void initScanOffset(double _xs, double _ys, double _thetas, float _radius)
  {
    xs = float(_xs);
    ys = float(_ys);
    thetas = float(_thetas);
    radius = int(_radius/resolution);  // only for display
  }
  void initScanSensor(float _angle_min, float _angle_inc, float _range_max)
  {
      angle_min = _angle_min;
      angle_increment = _angle_inc;
      range_max = _range_max;
  }

  void computeLaserScan(const float xr, const float yr, const float thetar,
                        std::vector<float> &scan);
};

#endif // OCCUPANCYGRID_H
