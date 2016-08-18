#include <ros/ros.h>
#include <opencv/cv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <sensor_msgs/Image.h>
#include <blinker_tracking/BlinkerArray.h>

#include <queue>

// param
int k_max_queue_size;

// image publisher
image_transport::Publisher image_pub;

// queue of images
std::queue< sensor_msgs::Image::ConstPtr > image_queue;

void blinker_callback(const blinker_tracking::BlinkerArray::ConstPtr &msg)
{

    // int k = image_queue.size();
    // while(k > 0)
    // {
    
    if (image_queue.size() == 0)
        return;

    // get oldest image
    sensor_msgs::Image::ConstPtr image_ros = image_queue.front();
    image_queue.pop();
    // k--;

    // std::cout << 
    //     "-" << std::endl <<
    //     "blinker id: " << msg->header.seq << std::endl <<
    //     "image id: " << image_ros->header.seq << std::endl;

    // if(msg->header.seq != image_ros->header.seq)
    // {
    //     continue;
    // }

    // convert from ros type to cv_bridge type
    cv_bridge::CvImagePtr image_ptr;
    image_ptr = cv_bridge::toCvCopy(image_ros);

    // extract opencv mat
    cv::Mat I = image_ptr->image;

    // list of keypoints for plotting
    std::vector< cv::KeyPoint > keypoints;
    for (int i = 0; i < msg->blinkers.size(); i++)
    {
        cv::Point2d p(msg->blinkers[i].u, msg->blinkers[i].v);
        cv::KeyPoint kp(p, 20);
        keypoints.push_back(kp);
    }

    // draw
    cv::Mat res;
    cv::drawKeypoints(I, keypoints, res, cv::Scalar(0, 255, 0), 
            cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    // publish
    cv_bridge::CvImage out;
    out.encoding = std::string("bgr8");
    out.image = res;
    image_pub.publish(out.toImageMsg());

    // }

}

void image_callback(const sensor_msgs::Image::ConstPtr &msg)
{
    // push the image into the queue
    image_queue.push(msg);

}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "blinker_tracker_viz");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);

    nh.param(std::string("k_max_queue_size"), k_max_queue_size, 10);

    ros::Subscriber blinker_sub;
    blinker_sub = nh.subscribe("blinkers", 10, &blinker_callback);

    ros::Subscriber image_sub;
    image_sub = nh.subscribe("image_in", 5, &image_callback);

    image_pub = it.advertise("image_out", 1);

    ros::spin();
    return 0;
}
