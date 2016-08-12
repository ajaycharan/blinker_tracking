#include <ros/ros.h>

int main (int argc, char* argv[])
{
    ros::init(argc, argv, "blinker_tracker");
    ros::NodeHandle nh("~");

    ros::Subscriber imu_sub;
    sub = nh.subscribe("imu", 10, &imu_callback);



}
