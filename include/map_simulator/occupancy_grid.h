#ifndef OCCUPANCYGRID_H
#define OCCUPANCYGRID_H

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <map_simulator/robot.h>

namespace map_simulator
{

class OccupancyGrid
{
  cv::Mat base_map, occ_map, scan_img;
  float res;
  float x0, y0;

  template <typename Numeric>
  cv::Point2f pointFrom(Numeric x, Numeric y)
  {
    return {(static_cast<float>(x)-x0)/res,
          occ_map.rows-(static_cast<float>(y)-y0)/res};
  }

  void computeLaserScan(Robot &robot, const std::list<Robot> &robots);

public:
  OccupancyGrid()
  {
    cv::namedWindow("Simulator 2D", cv::WINDOW_NORMAL);
  }
  double resolution() const
  {
    return res;
  }
  void initMap(const std::string &map_file, float max_height, float max_width);

  void computeLaserScans(std::list<Robot> &robots);
};

}

#endif // OCCUPANCYGRID_H
