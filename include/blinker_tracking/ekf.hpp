#ifndef EKF_HPP
#define EKF_HPP

#include <iostream>
#include <cmath>
#include <Eigen/Dense>


namespace blinker_tracking
{

    class EKF
    {
        private:
            
            Eigen::Vector2d x;
            Eigen::Matrix2d P;

        public:

            Eigen::Matrix2d Q;
            Eigen::Matrix2d R;

            EKF(Eigen::Vector2d x_0);
            void predict(Eigen::Matrix3d Hom);
            void correct(Eigen::Vector2d z_i);

            Eigen::Vector2d getState();
            Eigen::Matrix2d getCovariance();

    };

    EKF::EKF(Eigen::Vector2d x_0)
    {
        this->x = x_0;
        this->P = Eigen::Matrix2d::Identity();
        this->Q = Eigen::Matrix2d::Zero();
        this->R = Eigen::Matrix2d::Zero();
    }

    Eigen::Vector2d EKF::getState()
    {
        return this->x;
    }

    Eigen::Matrix2d EKF::getCovariance()
    {
        return this->P;
    }

    void EKF::predict(Eigen::Matrix3d Hom)
    {
        // homogeneous coordinates
        Eigen::Vector3d x_tilde;
        x_tilde  << this->x(0), this->x(1), 1.0;

        // intermediary values
        double gx   = Hom.row(0) * x_tilde;
        double gy   = Hom.row(1) * x_tilde;
        double h    = Hom.row(2) * x_tilde;

        // update transition matrix
        Eigen::Matrix2d F;
        F << 
            (Hom(0, 0) * h - Hom(2, 0) * gx) / (h*h),   (Hom(0, 1) * h - Hom(2, 1) * gx) / (h*h),
            (Hom(1, 0) * h - Hom(2, 0) * gy) / (h*h),   (Hom(1, 1) * h - Hom(2, 1) * gy) / (h*h);

        // propogate
        this->x = Eigen::Vector2d(gx / h, gy / h);
        this->P = F * this->P * F.transpose() + this->Q;
        
    }

    void EKF::correct(Eigen::Vector2d z_i)
    {
        // measurement matrix
        Eigen::Matrix2d I = Eigen::Matrix2d::Identity();
        Eigen::Matrix2d H = Eigen::Matrix2d::Identity();

        // innovation residual
        Eigen::Vector2d y = z_i - H * this->x;

        // innovation covariance
        Eigen::Matrix2d S = H * this->P * H.transpose() + this->R;

        // Kalman gain
        Eigen::Matrix2d K = this->P * H.transpose() * S.inverse();

        // updated state
        this->x = this->x + K * y;

        // updated covariance
        this->P = (I - K * H) * this->P;


    }

}

std::ostream& operator<<(std::ostream& os, blinker_tracking::EKF& ekf)
{
    os << "-" << std::endl <<
        "state: " << std::endl << 
        ekf.getState() << std::endl <<
        "covariance: " << std::endl << 
        ekf.getCovariance() << std::endl;

    return os;
}



#endif
