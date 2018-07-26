#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
MoveBaseClient* ac;

void centroid_callback(const geometry_msgs::PoseStampedConstPtr &msg){

  while(!ac->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = msg->header.stamp;

  goal.target_pose.pose.position.x = msg->pose.position.x;
  goal.target_pose.pose.position.y = msg->pose.position.y;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac->sendGoal(goal);

  ac->waitForResult();

  if(ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved to the goal");
  else
    ROS_INFO("The base failed to move for some reason");

}

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  ros::NodeHandle nh;

  ac = new MoveBaseClient("move_base", true);

  ros::Subscriber centroid_sub = nh.subscribe("/frontier_goal", 5, centroid_callback);

  ros::spin();

  return 0;
}
