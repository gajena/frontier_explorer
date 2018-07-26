#include <robot_exploration/frontier_search.h>
#include <robot_exploration/pose_handler.h>
#include <tf/LinearMath/Transform.h>

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
namespace FrontierSearches{

FrontierSearch::FrontierSearch(nav_msgs::OccupancyGrid &map, PoseHandlers::PoseHandler &pose_handler) : map_(map), pose_handler_(pose_handler){}

  std::vector<std::vector<std::pair<int, int> > > FrontierSearch::buildBidimensionalMap(){
  int height = map_.info.width;
  int width = map_.info.height;
  std::vector< std::vector<int> > vectorial_map (width, std::vector<int>(height));
  int data_index = 0;

  std::cout<<"Organizing map over a bidimensional grid-like array"<<std::endl;
    for(int i=0; i<width; i++){
      for(int j=0; j<height; j++){
        vectorial_map[i][j] = map_.data[data_index];
        data_index++;

      }
    }

  std::cout<<"Selecting frontier cells"<<std::endl;
  std::vector< std::pair<int, int> > frontier_cells;
  for(int i=0; i<width; i++){
    for(int j=0; j<height; j++){

      if(vectorial_map[i][j] == 0 && isFrontierCell(vectorial_map, i, j, 1)){
        frontier_cells.push_back(std::pair<int, int>(i,j));
      }

    }
  }

  std::cout<<"assignement of frontier cells to groups-frontiers"<<std::endl;
  std::vector<std::vector<std::pair<int, int> > > frontiers;
  for(std::pair<int, int> coords : frontier_cells){

      bool isOnAnyFrontier = false;
      /* Let's check if the cell belongs to one of the already classified frontiers and add to it */
      for(int i=0; i<frontiers.size(); i++){
        if(isOnSameFrontier(frontiers[i], coords, 4.0f)){
          isOnAnyFrontier = true;
          frontiers[i].push_back(coords);
          break;
        }
      }
      /*If the cell does not belong to any frontier we create a new one starting from that cell*/
      if(!isOnAnyFrontier){
        std::vector<std::pair<int, int> > v;
        v.push_back(coords);
        frontiers.push_back(v);
      }
  }

  std::cout<<"Frontiers at step 1 (detection): "<<frontiers.size()<<std::endl;


  for(std::vector<std::vector<std::pair<int, int> > >::iterator it_frontiers = frontiers.begin(); it_frontiers<(frontiers.end()-1);){
    for(std::vector<std::vector<std::pair<int, int> > >::iterator it_ahead = it_frontiers+1; it_ahead<frontiers.end();){
      bool merged = false;
      //std::cout<<"First it: "<<it_frontiers-frontiers.begin()<<"\t Second it: "<<it_ahead-frontiers.begin()<<std::endl;
      //std::cout<<"First it: "<< &it_frontiers <<"\t Second it: "<< &it_ahead<<"\t End it: "<< &frontiers.end()<<"\t End it: "<< &frontiers.end()<<"\t End it-1: "<< &frontiers.end()-1<<std::endl;
      for(int i=0; i<it_ahead->size();i++){
        if(isOnSameFrontier(*it_frontiers,(*it_ahead)[i], 4.0f)){
          it_frontiers->insert(it_frontiers->end(), it_ahead->begin(), it_ahead->end());
          it_ahead = frontiers.erase(it_ahead);
          //it_frontiers--;
          merged = true;
          break;
        }
      }
      if(!merged){
        it_ahead++;
      }
    }
    it_frontiers++;
   }

  std::cout<<"Frontiers at step 2 (merging): "<<frontiers.size()<<std::endl;



  //Remove small frontiers (single cell like). To improve: remove coherently with the robot size, based on the cell resolution of the map.
  for(int i=0; i<frontiers.size(); i++){
    if(frontiers[i].size()<25){
      frontiers.erase(frontiers.begin()+i);
      i--;
    }
  }
  std::cout<<"Frontiers at step 3 (filtering): "<<frontiers.size()<<std::endl;


  return frontiers;

}




/**
 * @brief FrontierSearch::isFrontierCell
 * @param grid
 * @param w_index
 * @param h_index
 * @param radius in which we want to search for a certain value (e.g. -1 for unknown cells in occ. grids)
 * @param value_to_check, default = -1 (unknown)
 * @return
 */
bool FrontierSearch::isFrontierCell(std::vector<std::vector<int> > grid, int w_index, int h_index, int radius, int value_to_check){

  for(int i=-radius; i<radius+1; i++){
    for(int j=-radius; j<radius+1; j++){
      if(w_index+i<0 || h_index+j<0 || w_index+i>=grid.size() || h_index+j>=grid[0].size())
        return true;
      if(grid[w_index+i][h_index+j] == value_to_check)
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
bool FrontierSearch::isOnSameFrontier(std::vector<std::pair<int, int> > frontier, std::pair<int, int> candidate, float radius){

   for(std::pair<int, int> frontier_cell : frontier){
     float d = sqrt(pow(candidate.first-frontier_cell.first, 2)+pow(candidate.second-frontier_cell.second,2));
     if(d<=radius)
       return true;
   }
  return false;

}

/**
 * @brief FrontierSearch::getCentroids
 * @param frontiers
 * @return
 */
std::vector<std::pair<int, int> > FrontierSearch::getCentroids(std::vector<std::vector<std::pair<int, int> > > frontiers){

  tf::Transform transform = pose_handler_.lookupPose("/map", "/base_link");
  float pose_x = transform.getOrigin().getX();
  float pose_y = transform.getOrigin().getY();
//  map_pose.first  = (pose_x/map_.info.resolution) + (map_.info.height/2);
//  map_pose.second = (pose_y/map_.info.resolution) + (map_.info.width/2);

  float min_distance = std::numeric_limits<float>::max();
  float min_index = -1;

  std::vector<std::pair<int, int> > centroids;
  for(std::vector<std::pair<int, int> > frontier: frontiers){
    int x_centroid = 0;
    int y_centroid = 0;
    for(std::pair<int, int> cell: frontier){
      x_centroid+=cell.first;
      y_centroid+=cell.second;
    }
    x_centroid/=frontier.size();
    y_centroid/=frontier.size();
    centroids.push_back(std::pair<int, int>(x_centroid, y_centroid));

    float rwX = y_centroid*map_.info.resolution + map_.info.origin.position.x; //swapped because of a mess with the row major ordering......
    float rwY = x_centroid*map_.info.resolution + map_.info.origin.position.y;

    float distance = sqrt(pow(rwX-pose_x, 2)+pow(rwY-pose_y,2));
    if(distance < min_distance){
      min_distance = distance;
      min_index = centroids.size()-1;
    }
  }

  std::swap(centroids[0], centroids[min_index]);
  ROS_INFO("Closest centroid (x,y) map: %d, %d \t (x,y) rw pose: %f, %f  ", centroids[0].first, centroids[0].second, pose_x, pose_y);


  return centroids;
}

}//Closes namespace
