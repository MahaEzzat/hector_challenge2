#include <ros/ros.h>
#include "hector_challenge2/HectorChallenge2.h"

int  main(int argc, char **argv)
{
    ros::init(argc, argv, "hector_challenge2");
    ros::NodeHandle n("~");

 hector_challenge2::HectorChallenge2 hectorchallenge2(n);

    ros::spin();
    return 0;
}