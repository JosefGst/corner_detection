#include "ros/ros.h"
#include "corner_detection.h"
#include "reconfigure.h"

CornerDetection::CornerDetection() : nh_("~")
{
  sub_line_segment = nh_.subscribe("/line_segments", 1000, &CornerDetection::corner_detection_cb, this);
}

void CornerDetection::corner_detection_cb(const laser_line_extraction::LineSegmentList::ConstPtr &msg)
{
  // ROS_INFO("____Frame ID: [%s]", msg->header.frame_id.c_str());
  float ix = -1.0, iy = -1.0;
  int corner_id = 0;

  for (int line_segment = 0; line_segment < msg->line_segments.size(); line_segment++)
  {
    for (int line_segment_n = line_segment + 1; line_segment_n < msg->line_segments.size(); line_segment_n++)
    {
      // ROS_INFO("Start1: [%f, %f]", msg->line_segments[line_segment].start[0], msg->line_segments[line_segment].start[1]);
      // ROS_INFO("End1: [%f, %f]", msg->line_segments[line_segment].end[0], msg->line_segments[line_segment].end[1]);

      // ROS_INFO("Start2: [%f, %f]", msg->line_segments[line_segment_n].start[0], msg->line_segments[line_segment_n].start[1]);
      // ROS_INFO("End2: [%f, %f]", msg->line_segments[line_segment_n].end[0], msg->line_segments[line_segment_n].end[1]);

      float x1 = msg->line_segments[line_segment].start[0];
      float y1 = msg->line_segments[line_segment].start[1];

      float x2 = msg->line_segments[line_segment].end[0];
      float y2 = msg->line_segments[line_segment].end[1];

      float x3 = msg->line_segments[line_segment_n].start[0];
      float y3 = msg->line_segments[line_segment_n].start[1];

      float x4 = msg->line_segments[line_segment_n].end[0];
      float y4 = msg->line_segments[line_segment_n].end[1];

      bool result = LineLineIntersect(x1, y1, x2, y2, x3, y3, x4, y4, ix, iy);
      // ROS_INFO("Intersection: [%f, %f]", ix, iy);
      

      float min_distance_line1_to_corner = std::min(distance_square(ix, iy, x1, y1), distance_square(ix, iy, x2, y2));
      float min_distance_line2_to_corner = std::min(distance_square(ix, iy, x3, y3), distance_square(ix, iy, x4, y4));
      // ROS_INFO("distance to corner: [%f, %f, %f]", distance_square(ix, iy, x1, y1), distance_square(ix, iy, x2, y2), min_distance_line1_to_corner);

      if (min_distance_line1_to_corner < pow(global_config.max_corner_distance_to_line,2) || min_distance_line2_to_corner < pow(global_config.max_corner_distance_to_line,2))
      {
        publish_corner_tf(ix, iy, corner_id);
        corner_id++;
      }
    }
  }
}

void CornerDetection::publish_corner_tf(float x, float y, int index)
{
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "base_link";
  transformStamped.child_frame_id = "corner";
  transformStamped.child_frame_id += std::to_string(index);
  transformStamped.transform.translation.x = x;
  transformStamped.transform.translation.y = y;
  transformStamped.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  corner_broadcaster.sendTransform(transformStamped);
}

float CornerDetection::Det(float a, float b, float c, float d)
{
  return a * d - b * c;
}

/// Calculate intersection of two lines.
///\return true if found, false if not found or error
bool CornerDetection::LineLineIntersect(float x1, float y1,         // Line 1 start
                                        float x2, float y2,         // Line 1 end
                                        float x3, float y3,         // Line 2 start
                                        float x4, float y4,         // Line 2 end
                                        float &ixOut, float &iyOut) // Output
{
  // http://mathworld.wolfram.com/Line-LineIntersection.html

  float detL1 = Det(x1, y1, x2, y2);
  float detL2 = Det(x3, y3, x4, y4);
  float x1mx2 = x1 - x2;
  float x3mx4 = x3 - x4;
  float y1my2 = y1 - y2;
  float y3my4 = y3 - y4;

  float xnom = Det(detL1, x1mx2, detL2, x3mx4);
  float ynom = Det(detL1, y1my2, detL2, y3my4);
  float denom = Det(x1mx2, y1my2, x3mx4, y3my4);
  if (denom == 0.0) // Lines don't seem to cross
  {
    ixOut = NAN;
    iyOut = NAN;
    return false;
  }

  ixOut = xnom / denom;
  iyOut = ynom / denom;
  if (!isfinite(ixOut) || !isfinite(iyOut)) // Probably a numerical issue
    return false;

  return true; // All OK
}

float CornerDetection::distance_square(float x1, float y1, float x2, float y2){
  return pow(x1-x2,2)+pow(y1-y2,2);
}