#include <ros/ros.h>
#include <blinker_tracking/KeyPoint.h>
#include <blinker_tracking/KeyPointArray.h>
#include <opencv/cv.hpp>

void callback(const blinker_tracking::KeyPointArray::ConstPtr &msg)
{
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "event_filter");
    ros::NodeHandle nh("~");
    
    ros::Subscriber sub;
    sub = nh.subscribe("candidates", 10, &callback);

    ros::spin();
    return 0;

}
