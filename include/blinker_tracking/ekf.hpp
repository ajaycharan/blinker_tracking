#ifndef EKF_HPP
#define EKF_HPP

#include <Eigen/Core>

namespace blinker_tracking
{

    typedef Eigen::Matrix<double, 2, 1> Vector2d;

    typedef Eigen::Matrix<double, 2, 2> Matrix2d;
    typedef Eigen::Matrix<double, 3, 3> Matrix3d;


    class EKF
    {
        private:
            
            Vector2d x;
            Matrix2d P;

            Matrix2d F;
            Matrix2d H;

        public:

            Matrix2d Q;
            Matrix2d R;

            EKF();
            void predict(Matrix3d Hom);
            void correct(Vector2d zi);

            Vector2d getState();
            Matrix2d getCovariance();

    };

    EKF::EKF()
    {
        P = Matrix2d::Identity();
        F = Matrix2d::Zero();
        H = Matrix2d::Identity();

        Q = Matrix2d::Zero();
        R = Matrix2d::Zero();
    }

    Vector2d EKF::getState()
    {
        return this->x;
    }

    Matrix2d EKF::getCovariance()
    {
        return this->P;
    }

    void EKF::predict(Matrix3d Hom)
    {

        
    }

    void EKF::correct(Vector2d zi)
    {

    }

}


#endif
