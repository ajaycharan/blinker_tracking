#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <blinker_tracking/BlinkerArray.h>
#include <blinker_tracking/BlobFeatureArray.h>

#include <vector>
#include <algorithm>

#include <blinker_tracking/ekf.hpp>

#define DYNAM_PARAMS   2
#define MEASURE_PARAMS   2

// publisher
ros::Publisher pub;

// correspondence threshold
double alpha;

// camera intrinsics matrix
Eigen::Matrix3d K;

// process noise matrix
Eigen::Matrix2d Q;

// measurement noise matrix
Eigen::Matrix2d R;

// last rotation
Eigen::Matrix3d Rot_0;

// array of tracked blinkers
// TODO: Manage this resource with a mutex lock
std::vector< blinker_tracking::EKF > particles;

void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{

    std::cout << "Prediction: " << std::endl;

    // extract rotation
    Eigen::Quaternion<double> q = Eigen::Quaternion<double>(
            msg->orientation.w,
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z);
    Eigen::Matrix3d Rot = q.toRotationMatrix();

    // delta R
    Eigen::Matrix3d dRot_imu = Rot * Rot_0.transpose();

    // save rotation
    Rot_0 = Rot;

    // transform between IMU and camera frames
    Eigen::Matrix3d T;
    T <<
        1.0,    0.0,    0.0,
        0.0,    -1.0,   0.0,
        0.0,    0.0,    -1.0;

    // compute rotation change in the camera frame I like
    Eigen::Matrix3d dRot_cam = T * dRot_imu * T.transpose();

    // create homography
    Eigen::Matrix3d H = K * dRot_cam.transpose() * K.inverse();

    std::cout << dRot_cam << std::endl;

    // propogate all predictions
    for (int i = 0; i < particles.size(); i++)
    {

        // predict
        particles[i].predict(H);

        // kill off particles with too high of a covariance or that have
        //      not been seen for a certain number of steps

        std::cout << particles[i] << std::endl;
    }

}

void blob_callback(const blinker_tracking::BlobFeatureArray::ConstPtr &msg)
{
    std::cout << "Update: " << std::endl;

    // for each feature seen
    for (int i = 0; i < msg->features.size(); i++)
    {

        // extract observation data
        Eigen::Vector2d z_i; 
        z_i <<
            msg->features[i].u,
            msg->features[i].v;

        // data association loop -- compare z_i to all other tracked blobs
        std::vector<double> pi;
        for (int k = 0; k < particles.size(); k++)
        {
            Eigen::Vector2d z_k = particles[k].getState();
            Eigen::Matrix2d S = particles[k].getCovariance();

            // compute mahalanobis distance
            double d = ( (z_i - z_k).transpose() * S * (z_i - z_k) );
            pi.push_back(d);

            std::cout << d << std::endl;
        }

        // check for sufficient newness before adding new element
        pi.push_back(alpha);
        int j = std::min_element(pi.begin(), pi.end()) - pi.begin();

        // add element
        if (j >= particles.size())
        {
            // initial position
            Eigen::Vector2d x_0;
            x_0 << 
                msg->features[i].u,
                msg->features[i].v;
            
            // create particle
            blinker_tracking::EKF ekf(x_0);
            ekf.Q = Q;
            ekf.R = R;

            // add new particle
            particles.push_back(ekf);

            std::cout << ekf << std::endl;

            return;
        }

        // update
        particles[j].correct(z_i);

    }

    // publish blinkers
    blinker_tracking::BlinkerArray ba;
    ba.header.seq = msg->header.seq;
    ba.header.stamp = msg->header.stamp;
    for (int i = 0; i < particles.size(); i++)
    {
        blinker_tracking::Blinker b;
        b.u = particles[i].getState()(0);
        b.v = particles[i].getState()(1);
        b.covariance[0] = particles[i].getCovariance()(0, 0);
        b.covariance[1] = particles[i].getCovariance()(0, 1);
        b.covariance[2] = particles[i].getCovariance()(1, 0);
        b.covariance[3] = particles[i].getCovariance()(1, 1);
        ba.blinkers.push_back(b);

        std::cout << particles[i] << std::endl;
    }
    std::cout << std::endl;

    pub.publish(ba);

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
    
    K << 
        fx,     s,      cx,
        0.0,    fy,     cy,
        0.0,    0.0,    1.0;

    Q << 
        Qx,     0.0,
        0.0,    Qy;

    R << 
        Rx,     0.0,
        0.0,    Ry;

    Rot_0 = Eigen::Matrix3d::Identity();

    ros::Subscriber imu_sub;
    imu_sub = nh.subscribe("imu", 10, &imu_callback);

    ros::Subscriber blob_sub;
    blob_sub = nh.subscribe("blob", 10, &blob_callback);

    pub = nh.advertise<blinker_tracking::BlinkerArray>("blinkers", 5);

    ros::spin();
    return 0;

}
