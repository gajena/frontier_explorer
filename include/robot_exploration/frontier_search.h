#ifndef FRONTIER_SEARCH_H
#define FRONTIER_SEARCH_H

#include <nav_msgs/OccupancyGrid.h>
#include <robot_exploration/pose_handler.h>
#include <math.h>

namespace FrontierSearches{

class FrontierSearch
{
public:
  FrontierSearch(nav_msgs::OccupancyGrid& map, PoseHandlers::PoseHandler& pose_handler);
  std::vector<std::vector<std::pair<int, int> > > buildBidimensionalMap();
  std::vector<std::pair<int, int> > getCentroids(std::vector<std::vector<std::pair<int, int> > > frontiers);
private:
  nav_msgs::OccupancyGrid& map_;
  PoseHandlers::PoseHandler& pose_handler_;
  bool isFrontierCell(std::vector<std::vector<int> > grid, int w_index, int h_index, int radius, int value_to_check = -1);
  bool isOnSameFrontier(std::vector<std::pair<int, int> > frontier, std::pair<int, int> candidate, float radius = sqrt(2.0));
  void MapCoord(int x, int y, float x_coord, float y_coord);
};

}

#endif // FRONTIER_SEARCH_H
