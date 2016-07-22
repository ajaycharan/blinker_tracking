#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.hpp>
#include <vector>

image_transport::Publisher pub;

// initializers
cv_bridge::CvImagePtr I_0;
bool is_init = 0;

// parameters
cv::SimpleBlobDetector::Params params;

void callback(const sensor_msgs::Image::ConstPtr &msg)
{

    if (!is_init)
    {
        I_0 = cv_bridge::toCvCopy(msg);
        is_init = 1;
        return;
    }

    cv_bridge::CvImagePtr I;
    I = cv_bridge::toCvCopy(msg);

    // abs difference of the last image with the incoming
    cv::Mat diff;
    cv::absdiff(I->image, I_0->image, diff);

    ////////////////////////// detect blobs //////////////////////////

    // keypoints
    std::vector<cv::KeyPoint> keypoints;

    // create detector
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

    // detect
    detector->detect(diff, keypoints);

    // results
    cv::Mat res;
    cv::drawKeypoints(diff, 
            keypoints, 
            res, 
            cv::Scalar(0, 0, 255), 
            cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    
    ///////////////////////////////////////////////////////////////////

    // publish result
    cv_bridge::CvImage out;
    out.encoding = std::string("bgr8");
    out.image = res;
    pub.publish(out.toImageMsg());

    // publish keypoints

    // save last
    I_0 = I;

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "blinker_detection_node");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);

    int blobColor;

    // parameters
    nh.param(std::string("minThreshold"),        params.minThreshold,        (float) 128);
    nh.param(std::string("maxThreshold"),        params.maxThreshold,        (float) 255);
    nh.param(std::string("filterByColor"),       params.filterByColor,       true);
    nh.param(std::string("blobColor"),           blobColor,                  255);
    nh.param(std::string("filterByArea"),        params.filterByArea,        false);
    nh.param(std::string("minArea"),             params.minArea,             (float) 500);
    nh.param(std::string("maxArea"),             params.maxArea,             (float) 1000);
    nh.param(std::string("filterByCircularity"), params.filterByCircularity, false);
    nh.param(std::string("minCircularity"),      params.minCircularity,      (float) 0.5);
    nh.param(std::string("filterByConvexity"),   params.filterByConvexity,   false);
    nh.param(std::string("minConvexity"),        params.minConvexity,        (float) 0.4);
    nh.param(std::string("filterByInertia"),     params.filterByInertia,     false);
    nh.param(std::string("minInertiaRatio"),     params.minInertiaRatio,     (float) 0.2);
    nh.param(std::string("minDistBetweenBlobs"), params.minDistBetweenBlobs, (float) 100);

    params.blobColor = blobColor;

    ros::Subscriber sub;
    sub = nh.subscribe("image_raw", 10, &callback);

    pub = it.advertise("image_out", 1);

    ros::spin();
    return 0;

}
