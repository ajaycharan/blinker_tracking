#include <ros/ros.h>

#include <blinker_tracking/BlinkersWithImage.h>

void callback(const blinker_tracking::BlinkersWithImage::ConstPtr &msg)
{
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "blinker_decoder");
    ros::NodeHandle nh("~");

    ros::Subscriber sub;
    sub = nh.subscribe("blinkers", 10, &callback);

    ros::spin();
    return 0;

}
