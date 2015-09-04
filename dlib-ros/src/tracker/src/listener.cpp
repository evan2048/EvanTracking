/*
*2015.9.4 write by evan2048
*
*/
#include "ros/ros.h"
#include "std_msgs/String.h"

void msgCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("got you:%s", msg->data.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle nodeHandle;
    ros::Subscriber sub = nodeHandle.subscribe("tracker", 1000, msgCallback);
    ros::spin();
    return 0;
}
