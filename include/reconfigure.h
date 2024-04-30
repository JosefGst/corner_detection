#ifndef RECONFIGURE_H
#define RECONFIGURE_H

#include <dynamic_reconfigure/server.h>
#include <corner_detection/ReconfigureConfig.h>

corner_detection::ReconfigureConfig global_config;

void reconfigure_cb(corner_detection::ReconfigureConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure Request: %f %s",
           config.max_corner_distance_to_line,
           config.frame_id.c_str());
  global_config = config;
}

#endif