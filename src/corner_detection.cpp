#include "ros/ros.h"
#include "corner_detection.h"

CornerDetection::CornerDetection() : nh_("~")
{
  sub_line_segment = nh_.subscribe("/line_segments", 1000, &CornerDetection::chatterCallback, this);
}

void CornerDetection::chatterCallback(const laser_line_extraction::LineSegmentList::ConstPtr& msg)
{
  ROS_INFO("Frame ID: [%s]", msg->header.frame_id.c_str());
  ROS_INFO("I heard: [%f]", msg->line_segments[2].radius);

}
