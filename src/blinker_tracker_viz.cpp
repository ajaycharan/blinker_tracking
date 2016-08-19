#include <ros/ros.h>
#include <opencv/cv.hpp>
#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <sensor_msgs/Image.h>
#include <blinker_tracking/BlinkersWithImage.h>

#include <queue>
#include <cmath>

// image publisher
image_transport::Publisher image_pub;

void blinker_callback(const blinker_tracking::BlinkersWithImage::ConstPtr &msg)
{

    // convert from ros type to cv_bridge type
    cv_bridge::CvImagePtr image_ptr;
    image_ptr = cv_bridge::toCvCopy(msg->image);

    // extract opencv mat
    cv::Mat I = image_ptr->image;

    // list of keypoints for plotting
    std::vector< cv::KeyPoint > keypoints;
    for (int i = 0; i < msg->blinkers.size(); i++)
    {

        Eigen::Matrix2d P;
        P << msg->blinkers[i].covariance[0], msg->blinkers[i].covariance[1], 
          msg->blinkers[i].covariance[2], msg->blinkers[i].covariance[3];

        Eigen::Vector2cd eigs;
        eigs = P.eigenvalues();

        double radius = eigs.norm();
        std::cout << radius << std::endl;

        cv::Point2d p(msg->blinkers[i].u, msg->blinkers[i].v);
        cv::KeyPoint kp(p, radius);
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

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "blinker_tracker_viz");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);

    ros::Subscriber blinker_sub;
    blinker_sub = nh.subscribe("blinkers", 10, &blinker_callback);

    image_pub = it.advertise("image_out", 1);

    ros::spin();
    return 0;
}
