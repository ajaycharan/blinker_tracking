#include <ros/ros.h>
#include <opencv/cv.hpp>

#include <sensor_msgs/Imu.h>
#include <blinker_tracking/BlobFeature.h>

#include <vector>

// correspondence threshold
double alpha;

// camera intrinsics matrix
cv::Mat K;

// process noise matrix
cv::Mat Q;

// measurement noise matrix
cv::Mat R;

// array of tracked blinkers
std::vector< cv::KalmanFilter > particles;

void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{

    // propogate all predictions
    for (int i = 0; i < particles.size(); i++)
    {
        
    }

}

void blob_callback(const blinker_tracking::BlobFeature::ConstPtr &msg)
{

    // data association first

    // create new particle if necessary
    if (particles.size() == 0)
    {
    }

    // perform update
    
    // kill off particles with too high of a covariance or that have
    //      not been seen for a certain number of steps

}



int main (int argc, char* argv[])
{

    ros::init(argc, argv, "blinker_tracker");
    ros::NodeHandle nh("~");

    // parameters
    nh.param(std::string("alpha"), alpha, 100.0);

    double fx, fy, cx, cy, s;
    nh.param(std::string("fx"), fx, 602.815050);
    nh.param(std::string("fy"), fy, 494.179300);
    nh.param(std::string("cx"), cx, 601.697141);
    nh.param(std::string("cy"), cy, 403.922047);
    nh.param(std::string("s"), s, 0.0);

    double Qx, Qy, Qa, Qc, Qi;
    nh.param(std::string("Qx"), Qx, 0.0);
    nh.param(std::string("Qy"), Qy, 0.0);

    double Rx, Ry, Ra, Rc, Ri;
    nh.param(std::string("Rx"), Rx, 4.0);
    nh.param(std::string("Ry"), Ry, 4.0);
    
    K  = (cv::Mat_<double>(3, 3) << 
            fx,     s,      cx,
            0.0,    fy,     cy,
            0.0,    0.0,    1.0);

    Q  = (cv::Mat_<double>(2, 2) << 
            Qx,     0.0,
            0.0,    Qy);

    R  = (cv::Mat_<double>(2, 2) << 
            Rx,     0.0,
            0.0,    Ry);

    ros::Subscriber imu_sub;
    imu_sub = nh.subscribe("imu", 10, &imu_callback);

    ros::Subscriber blob_sub;
    blob_sub = nh.subscribe("blob", 10, &blob_callback);

    ros::spin();
    return 0;

}
