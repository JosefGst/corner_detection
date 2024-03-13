#include "ros/ros.h"
#include "std_msgs/String.h"
#include "laser_line_extraction/LineSegmentList.h"


void chatterCallback(const laser_line_extraction::LineSegmentList::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->header.frame_id.c_str());
  ROS_INFO("I heard: [%f]", msg->line_segments[0].radius);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "corner_detection");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("line_segments", 1000, chatterCallback);
  ros::spin();
  return 0;
}