#include <robot_exploration/pose_handler.h>
#include <ros/ros.h>

namespace PoseHandlers{

PoseHandler::PoseHandler()
{
}

tf::StampedTransform PoseHandler::lookupPose(std::string parent, std::string child){


  tf::StampedTransform transform;
  try{
    listener.lookupTransform(parent, child, ros::Time(0), transform);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  return transform;

}

}//Closes namespace
