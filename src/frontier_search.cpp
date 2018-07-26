#include <robot_exploration/frontier_search.h>
#include <robot_exploration/pose_handler.h>
#include <tf/LinearMath/Transform.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>

int i_fov_mid, j_fov_mid, i_fov_min, j_fov_min, i_fov_max, j_fov_max;

namespace FrontierSearches
{
FrontierSearch::FrontierSearch(nav_msgs::OccupancyGrid &map, PoseHandlers::PoseHandler &pose_handler) : map_(map), pose_handler_(pose_handler) {}

std::vector<std::vector<std::pair<int, int>>> FrontierSearch::buildBidimensionalMap()
{
  int height = map_.info.width;
  int width = map_.info.height;
  std::vector<std::vector<int>> vectorial_map(width, std::vector<int>(height));
  int data_index = 0;

  // std::cout<<"Organizing map over a bidimensional grid-like array"<<std::endl;
  for (int i = 0; i < width; i++)
  {
    for (int j = 0; j < height; j++)
    {
      vectorial_map[i][j] = map_.data[data_index];
      data_index++;
    }
  }

  // std::cout<<"Selecting frontier cells"<<std::endl;
  std::vector<std::pair<int, int>> frontier_cells;
  for (int i = 0; i < width; i++)
  {
    for (int j = 0; j < height; j++)
    {

      tf::Transform transform = pose_handler_.lookupPose("/map", "/imu");
      float pose_x = (transform.getOrigin().getX()) - 0.14;
      float pose_y = (transform.getOrigin().getY()) - 0.05;
      tf::Quaternion q1(
          transform.getRotation().getX(),
          transform.getRotation().getY(),
          transform.getRotation().getZ(),
          transform.getRotation().getW());

      tf::Matrix3x3 m(q1);
      double r, p, yaw;
      m.getRPY(r, p, yaw);
      
      float x_coord = j * map_.info.resolution + map_.info.origin.position.x;
      float y_coord = i * map_.info.resolution + map_.info.origin.position.y;

      if (vectorial_map[i][j] == 0 && isFrontierCell(vectorial_map, i, j, 1))
      {
        if (((x_coord - 0.4) < pose_x && (x_coord + 0.4) > pose_x && (y_coord - 0.4) < pose_y && (y_coord + 0.4) > pose_y) != 1)
        {

          float y_mid = pose_y + (x_coord - pose_x) * tan(yaw);
          float y_min = pose_y + (x_coord - pose_x) * tan(yaw - 0.5);
          float y_max = pose_y + (x_coord - pose_x) * tan(yaw + 0.5);
          if ((y_mid - y_coord) < 0.1 && (y_mid - y_coord) > -0.1)
          {
            i_fov_mid = (y_coord - map_.info.origin.position.y) / map_.info.resolution;
            j_fov_mid = (x_coord - map_.info.origin.position.x) / map_.info.resolution;
          }
          frontier_cells.push_back(std::pair<int, int>(i, j));
        }
      }
    }
  }

  // std::cout<<"assignement of frontier cells to groups-frontiers"<<std::endl;
  std::vector<std::vector<std::pair<int, int>>> frontiers;
  for (std::pair<int, int> coords : frontier_cells)
  {

    bool isOnAnyFrontier = false;
    /* Let's check if the cell belongs to one of the already classified frontiers and add to it */
    for (int i = 0; i < frontiers.size(); i++)
    {
      if (isOnSameFrontier(frontiers[i], coords, 4.0f))
      {
        isOnAnyFrontier = true;
        frontiers[i].push_back(coords);
        break;
      }
    }
    /*If the cell does not belong to any frontier we create a new one starting from that cell*/
    if (!isOnAnyFrontier)
    {
      std::vector<std::pair<int, int>> v;
      v.push_back(coords);
      frontiers.push_back(v);
    }
  }

  // std::cout<<"Frontiers at step 1 (detection): "<<frontiers.size()<<std::endl;

  for (std::vector<std::vector<std::pair<int, int>>>::iterator it_frontiers = frontiers.begin(); it_frontiers < (frontiers.end() - 1);)
  {
    for (std::vector<std::vector<std::pair<int, int>>>::iterator it_ahead = it_frontiers + 1; it_ahead < frontiers.end();)
    {
      bool merged = false;
      //std::cout<<"First it: "<<it_frontiers-frontiers.begin()<<"\t Second it: "<<it_ahead-frontiers.begin()<<std::endl;
      //std::cout<<"First it: "<< &it_frontiers <<"\t Second it: "<< &it_ahead<<"\t End it: "<< &frontiers.end()<<"\t End it: "<< &frontiers.end()<<"\t End it-1: "<< &frontiers.end()-1<<std::endl;
      for (int i = 0; i < it_ahead->size(); i++)
      {
        if (isOnSameFrontier(*it_frontiers, (*it_ahead)[i], 4.0f))
        {
          it_frontiers->insert(it_frontiers->end(), it_ahead->begin(), it_ahead->end());
          it_ahead = frontiers.erase(it_ahead);
          //it_frontiers--;
          merged = true;
          break;
        }
      }
      if (!merged)
      {
        it_ahead++;
      }
    }
    it_frontiers++;
  }

  // std::cout<<"Frontiers at step 2 (merging): "<<frontiers.size()<<std::endl;

  //Remove small frontiers (single cell like). To improve: remove coherently with the robot size, based on the cell resolution of the map.
  for (int i = 0; i < frontiers.size(); i++)
  {
    if (frontiers[i].size() < 15)
    {
      frontiers.erase(frontiers.begin() + i);
      i--;
    }
  }

  return frontiers;
}

/**
   * @brief FrontierSearch::isFrontierCell
   * @param grid
   * @param w_index
   * @param h_index
   * @param radius in which we want to search for a certain value imu
  imu(e.g. -1 for unknown cells in occ. grids)
  * @param value_to_check, default = -1 (unknown)
  * @return
  */
bool FrontierSearch::isFrontierCell(std::vector<std::vector<int>> grid, int w_index, int h_index, int radius, int value_to_check)
{

  for (int i = -radius; i < radius + 1; i++)
  {
    for (int j = -radius; j < radius + 1; j++)
    {
      if (w_index + i < 0 || h_index + j < 0 || w_index + i >= grid.size() || h_index + j >= grid[0].size())
        return true;
      if (grid[w_index + i][h_index + j] == value_to_check)
        return true;
    }
  }
  return false;
}

/**
   * @brief FrontierSearch::isOnSameFrontier
   * @param frontier to examine
   * @param candidate
   * @param radius in which we still consider the candidate a neighbour, default = SQRT(2)
   * @return
   */
bool FrontierSearch::isOnSameFrontier(std::vector<std::pair<int, int>> frontier, std::pair<int, int> candidate, float radius)
{
  std::cout << "ymid" << i_fov_mid << "," << j_fov_mid << std::endl;
  if (frontier.size() > 50)
    return false;
  for (std::pair<int, int> frontier_cell : frontier)
  {
    float d = sqrt(pow(candidate.first - frontier_cell.first, 2) + pow(candidate.second - frontier_cell.second, 2));
    if (d <= radius && (sqrt(pow(candidate.first - i_fov_mid, 2) + pow(candidate.second - j_fov_mid, 2))) < 25)
      return true;
    if (d <= radius && (sqrt(pow(candidate.first - i_fov_mid, 2) + pow(candidate.second - j_fov_mid, 2))) > 24)
      return true;
  }
  return false;
}

// void FrontierSearch::MapCoord(int x, int y, float x_coord, float y_coord)
// {
//   x = (y_coord - map_.info.origin.position.y) / map_.info.resolution;
//   y = (x_coord - map_.info.origin.position.x) / map_.info.resolution;
// }

/**
   * @brief FrontierSearch::getCentroids
   * @param frontiers
   * @return
   */
std::vector<std::pair<int, int>> FrontierSearch::getCentroids(std::vector<std::vector<std::pair<int, int>>> frontiers)
{

  tf::Transform transform = pose_handler_.lookupPose("/map", "/imu");
  float pose_x = (transform.getOrigin().getX()) - 0.14;
  float pose_y = (transform.getOrigin().getY()) - 0.05;
  //  map_pose.first  = (pose_x/map_.info.resolution) + (map_.info.height/2);
  //  map_pose.second = (pose_y/map_.info.resolution) + (map_.info.width/2);

  float min_distance = std::numeric_limits<float>::max();
  float min_index = -1;

  std::vector<std::pair<int, int>> centroids;
  for (std::vector<std::pair<int, int>> frontier : frontiers)
  {
    int x_centroid = 0;
    int y_centroid = 0;
    for (std::pair<int, int> cell : frontier)
    {
      x_centroid += cell.first;
      y_centroid += cell.second;
    }
    x_centroid /= frontier.size();
    y_centroid /= frontier.size();
    float dist = 1000000;
    float x_temp = x_centroid;
    float y_temp = y_centroid;
    for (std::pair<int, int> cell : frontier)
    {
      float min_dist = sqrt(pow((x_temp - cell.first), 2) + pow((y_temp - cell.second), 2));
      if (dist > min_dist)
      {
        dist = min_dist;
        x_centroid = cell.first;
        y_centroid = cell.second;
      }
    }
    centroids.push_back(std::pair<int, int>(x_centroid, y_centroid));

    float rwX = y_centroid * map_.info.resolution + map_.info.origin.position.x; //swapped because of a mess with the row major ordering......
    float rwY = x_centroid * map_.info.resolution + map_.info.origin.position.y;

    float distance = sqrt(pow(rwX - pose_x, 2) + pow(rwY - pose_y, 2));
    if (distance < min_distance)
    {
      min_distance = distance;
      min_index = centroids.size() - 1;
    }
  }

  std::swap(centroids[0], centroids[min_index]);
  // ROS_INFO("Closest centroid (x,y) map: %d, %d \t (x,y) rw pose: %f, %f  ", centroids[0].first, centroids[0].second, pose_x, pose_y);

  return centroids;
}

} // namespace FrontierSearches
