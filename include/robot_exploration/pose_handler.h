#ifndef POSE_HANDLER_H
#define POSE_HANDLER_H

#include <tf/transform_listener.h>

namespace PoseHandlers{

class PoseHandler
{
public:
  PoseHandler();
  tf::StampedTransform lookupPose(std::string parent="/map", std::string child="/base_link");

private:
    tf::TransformListener listener;

};

}
#endif // POSE_HANDLER_H
