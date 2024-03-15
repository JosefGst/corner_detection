#ifndef CORNER_DETECTION_H
#define CORNER_DETECTION_H

#include "ros/ros.h"
#include "laser_line_extraction/LineSegmentList.h"

class CornerDetection
{
public:
    CornerDetection();
    void chatterCallback(const laser_line_extraction::LineSegmentList::ConstPtr& msg);
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_line_segment;
    
};

#endif // CORNER_DETECTION_H