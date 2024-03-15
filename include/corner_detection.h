#ifndef CORNER_DETECTION_H
#define CORNER_DETECTION_H

#include "ros/ros.h"
#include "laser_line_extraction/LineSegmentList.h"

class CornerDetection
{
public:
    CornerDetection();

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_line_segment;

    void corner_detection_cb(const laser_line_extraction::LineSegmentList::ConstPtr &msg);

    /**
     * @param Start1 First Start Point
     * @param End1 First End Point
     * @param Start2 Second Start Point
     * @param End2 Second End Point
     * @return 0 when OK. 1 if crc error
     */
    void get_intersection(float *Start1, float *End1,float *Start2,float *End2);
};

#endif // CORNER_DETECTION_H