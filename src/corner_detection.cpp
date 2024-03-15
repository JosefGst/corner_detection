#include "ros/ros.h"
#include "corner_detection.h"

CornerDetection::CornerDetection() : nh_("~")
{
  sub_line_segment = nh_.subscribe("/line_segments", 1000, &CornerDetection::corner_detection_cb, this);
}

void CornerDetection::corner_detection_cb(const laser_line_extraction::LineSegmentList::ConstPtr &msg)
{
  ROS_INFO("Frame ID: [%s]", msg->header.frame_id.c_str());

  for (int line_segment = 0; line_segment < msg->line_segments.size(); line_segment++){
    float start1[line_segment][2], end1[line_segment][2];
    for (int coordinate = 0; coordinate<2; coordinate++){
      ROS_INFO("I heard: [%f]", msg->line_segments[line_segment].start[coordinate]);
      // start1[coordinate] = msg->line_segments[line_segment].start[coordinate];
    }
    // ROS_INFO("Start1: [%f, %f]", start1[0], start1[1]);
  }
}
