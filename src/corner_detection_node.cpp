#include "corner_detection.h"
#include "reconfigure.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "corner_detection");

    // DYNAMIC RECONFIGURE
    dynamic_reconfigure::Server<corner_detection::ReconfigureConfig> server;
    dynamic_reconfigure::Server<corner_detection::ReconfigureConfig>::CallbackType f;
    f = boost::bind(&reconfigure_cb, _1, _2);
    server.setCallback(f);

    CornerDetection corner_detection;

    ros::spin();
    return 0;
}