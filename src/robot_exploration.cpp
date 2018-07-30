#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <geometry_msgs/PoseArray.h>
#include <robot_exploration/pose_handler.h>
#include <robot_exploration/frontier_search.h>

class RobotExploration{

public:

  RobotExploration(){}

  void map_callback(const nav_msgs::OccupancyGridConstPtr& msg);

  nav_msgs::OccupancyGrid map;
  nav_msgs::OccupancyGrid frontiers_map;
  std::vector<std::pair<int, int> > centroids;

  std::vector<std::vector<std::pair<int, int> > > frontiers;

  ros::NodeHandle nh;

  PoseHandlers::PoseHandler pose_handler;

private:

};

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "robot_exploration");

  RobotExploration robot_exploration;

  ROS_INFO("Hello world!");

  ros::Subscriber map_sub = robot_exploration.nh.subscribe("/costmap_node/costmap/costmap", 1, &RobotExploration::map_callback, &robot_exploration);
  ros::Publisher frontier_pub = robot_exploration.nh.advertise<nav_msgs::OccupancyGrid>("frontiers", 10);
  ros::Publisher goal_pub = robot_exploration.nh.advertise<geometry_msgs::PoseArray>("/frontier/goal", 1);

  ros::Rate loop_rate(3.0);

  while(ros::ok()){

    for(int i = 0; i<robot_exploration.map.data.size(); i++ ){
      robot_exploration.frontiers_map.data[i] = -1;
    }
    int data_index = 0;
    int darkness = 120;
    for(std::vector<std::pair<int, int> > frontier : robot_exploration.frontiers){
      for(std::pair<int, int> coords: frontier){
        for(int i=0; i<robot_exploration.map.info.height; i++){
          for(int j=0; j<robot_exploration.map.info.width; j++){
            if(coords.first == i && coords.second == j){
              robot_exploration.frontiers_map.data[data_index] = darkness;
            }
            if(i == 288 && j == 288)robot_exploration.frontiers_map.data[data_index] = 99;
            data_index++;
          }
        }
        data_index=0;
      }
      darkness=(darkness+100);
    }
    data_index = 0;
    for(int i=0; i<robot_exploration.map.info.height; i++){
      for(int j=0; j<robot_exploration.map.info.width; j++){
        for(std::pair<int, int> coords: robot_exploration.centroids){
          if(coords.first == i && coords.second == j){
            robot_exploration.frontiers_map.data[data_index] = 0;
          }
        }
        data_index++;
      }
    }

//    frontier_pub.publish(robot_exploration.frontiers_map);
    ros::spinOnce();

    if(!robot_exploration.centroids.empty()){
    geometry_msgs::PoseArray goal;
    geometry_msgs::Pose pose_;
    goal.header.frame_id = "/map";
    goal.header.stamp = ros::Time::now();
    for (int i = 0; i < robot_exploration.centroids.size() ; i++)
    {
      pose_.position.x = (robot_exploration.centroids[i].second * robot_exploration.map.info.resolution) + robot_exploration.map.info.origin.position.x;
      pose_.position.y = (robot_exploration.centroids[i].first*robot_exploration.map.info.resolution)+robot_exploration.map.info.origin.position.y;
      pose_.position.z = 0;
      goal.poses.push_back(pose_);
    }
    //ROS_INFO("%d, %f, %f", robot_exploration.centroids[0].first, robot_exploration.map.info.resolution, robot_exploration.map.info.origin.position.x);
    // ROS_INFO("Centroid (x,y) map: %d, %d \t (x,y) rw: %f, %f  ", robot_exploration.centroids[0].first, robot_exploration.centroids[0].second, goal.pose.position.x, goal.pose.position.y);

    goal_pub.publish(goal);
    goal.poses.clear();
    }


    loop_rate.sleep();

  }

  ros::shutdown();

}

void RobotExploration::map_callback(const nav_msgs::OccupancyGridConstPtr &msg){

  map = *msg;
  FrontierSearches::FrontierSearch frontier_search(map, pose_handler);
  frontiers = frontier_search.buildBidimensionalMap();
  centroids = frontier_search.getCentroids(frontiers);
  frontiers_map = map;

}

