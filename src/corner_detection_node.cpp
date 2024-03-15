#include "corner_detection.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "corner_detection");
    CornerDetection corner_detection;

    ros::spin();

    return 0;
}