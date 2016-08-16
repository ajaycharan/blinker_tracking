#include <ros/ros.h>
#include <opencv/cv.hpp>

#include <sensor_msgs/Imu.h>
#include <blinker_tracking/BlobFeatureArray.h>

#include <vector>
#include <algorithm>

#define DYNAM_PARAMS   2
#define MEASURE_PARAMS   2

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

void blob_callback(const blinker_tracking::BlobFeatureArray::ConstPtr &msg)
{
    // check input
    if (msg->features.size() == 0)
    {
        return;
    }

    // for each feature seen
    for (int i = 0; i < msg->features.size(); i++)
    {

        // extract observation data
        cv::Mat zi = (cv::Mat_<double>(DYNAM_PARAMS, 1) <<
                msg->features[i].x,
                msg->features[i].y);

        // data association loop -- compare zi to all other tracked blobs
        std::vector<double> pi;
        for (int k = 0; k < particles.size(); k++)
        {
            cv::Mat zk = particles[k].statePre;
            cv::Mat S = particles[k].errorCovPre;

            // compute mahalanobis distance
            cv::Mat d = ( (zi - zk).t() * S * (zi - zk) );
            pi.push_back(d.at<double>(0));
        }

        // check for sufficient newness before adding new element
        pi.push_back(alpha);
        int j = std::min_element(pi.begin(), pi.end()) - pi.begin();

        // add element
        if (j > particles.size())
        {
            cv::KalmanFilter KF(DYNAM_PARAMS, MEASURE_PARAMS, 0);

            cv::setIdentity(KF.transitionMatrix);
            // TODO: [PROBLEM] this should be a 3x3 for the homography...
            // is this nonlinear...?

            cv::setIdentity(KF.measurementMatrix);
            KF.processNoiseCov = Q.clone();
            KF.measurementNoiseCov = R.clone();

        }

        // update

        // kill off particles with too high of a covariance or that have
        //      not been seen for a certain number of steps
    }

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

    Q  = (cv::Mat_<double>(DYNAM_PARAMS, DYNAM_PARAMS) << 
            Qx,     0.0,
            0.0,    Qy);

    R  = (cv::Mat_<double>(DYNAM_PARAMS, DYNAM_PARAMS) << 
            Rx,     0.0,
            0.0,    Ry);

    ros::Subscriber imu_sub;
    imu_sub = nh.subscribe("imu", 10, &imu_callback);

    ros::Subscriber blob_sub;
    blob_sub = nh.subscribe("blob", 10, &blob_callback);

    // publish posterior measuremnts

    ros::spin();
    return 0;

}
