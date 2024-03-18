#ifndef CORNER_DETECTION_H
#define CORNER_DETECTION_H

#include "ros/ros.h"
#include "laser_line_extraction/LineSegmentList.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

class CornerDetection
{
public:
    CornerDetection();
    float max_conrner_distance_to_line = 1.0;

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_line_segment;

    tf2_ros::TransformBroadcaster corner_broadcaster;
    geometry_msgs::TransformStamped transformStamped;

    /**
     * @brief callback whick is called after a "line_segment" topic is received
     */
    void corner_detection_cb(const laser_line_extraction::LineSegmentList::ConstPtr &msg);

    /** Publish the tf of the detected corner as "corner<id>" the id will increment strarting from 0:
    @param x x-coordinate
    @param y y-coordinate
    @index index number which will be appended to the "corner" string
    */
    void publish_corner_tf(float x, float y, int index);

    /** Calculate determinant of matrix:
    [a b]
    [c d]
    */
    float Det(float a, float b, float c, float d);

    /// Calculate intersection of two lines.
    ///\return true if found, false if not found or error
    bool LineLineIntersect(float x1, float y1,          // Line 1 start
                           float x2, float y2,          // Line 1 end
                           float x3, float y3,          // Line 2 start
                           float x4, float y4,          // Line 2 end
                           float &ixOut, float &iyOut); // Output

    float distance_square(float x1, float y1, float x2, float y2);
};

#endif // CORNER_DETECTION_H