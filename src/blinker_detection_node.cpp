#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <blinker_tracking/KeyPoint.h>
#include <blinker_tracking/KeyPointArray.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.hpp>
#include <vector>

image_transport::Publisher event_image_pub;
ros::Publisher candidate_pub;

// initializers
cv_bridge::CvImagePtr I_0;
bool is_init = 0;

// parameters
cv::SimpleBlobDetector::Params params;

using namespace cv;

struct Center
{
    Point2d location;
    double radius;
    double confidence;
};

void findBlobs(InputArray _image, InputArray _binaryImage, std::vector<Center> &centers)
{
    Mat image = _image.getMat(), binaryImage = _binaryImage.getMat();
    (void)image;
    centers.clear();

    std::vector < std::vector<Point> > contours;
    Mat tmpBinaryImage = binaryImage.clone();
    findContours(tmpBinaryImage, contours, RETR_LIST, CHAIN_APPROX_NONE);

    for (size_t contourIdx = 0; contourIdx < contours.size(); contourIdx++)
    {
        Center center;
        center.confidence = 1;
        Moments moms = moments(Mat(contours[contourIdx]));
        if (params.filterByArea)
        {
            double area = moms.m00;
            if (area < params.minArea || area >= params.maxArea)
                continue;
        }

        if (params.filterByCircularity)
        {
            double area = moms.m00;
            double perimeter = arcLength(Mat(contours[contourIdx]), true);
            double ratio = 4 * CV_PI * area / (perimeter * perimeter);
            if (ratio < params.minCircularity || ratio >= params.maxCircularity)
                continue;
        }

        if (params.filterByInertia)
        {
            double denominator = std::sqrt(std::pow(2 * moms.mu11, 2) + std::pow(moms.mu20 - moms.mu02, 2));
            const double eps = 1e-2;
            double ratio;
            if (denominator > eps)
            {
                double cosmin = (moms.mu20 - moms.mu02) / denominator;
                double sinmin = 2 * moms.mu11 / denominator;
                double cosmax = -cosmin;
                double sinmax = -sinmin;

                double imin = 0.5 * (moms.mu20 + moms.mu02) - 0.5 * (moms.mu20 - moms.mu02) * cosmin - moms.mu11 * sinmin;
                double imax = 0.5 * (moms.mu20 + moms.mu02) - 0.5 * (moms.mu20 - moms.mu02) * cosmax - moms.mu11 * sinmax;
                ratio = imin / imax;
            }
            else
            {
                ratio = 1;
            }

            if (ratio < params.minInertiaRatio || ratio >= params.maxInertiaRatio)
                continue;

            center.confidence = ratio * ratio;
        }

        if (params.filterByConvexity)
        {
            std::vector < Point > hull;
            convexHull(Mat(contours[contourIdx]), hull);
            double area = contourArea(Mat(contours[contourIdx]));
            double hullArea = contourArea(Mat(hull));
            double ratio = area / hullArea;
            if (ratio < params.minConvexity || ratio >= params.maxConvexity)
                continue;
        }

        if(moms.m00 == 0.0)
            continue;
        center.location = Point2d(moms.m10 / moms.m00, moms.m01 / moms.m00);

        if (params.filterByColor)
        {
            if (binaryImage.at<uchar> (cvRound(center.location.y), cvRound(center.location.x)) != params.blobColor)
                continue;
        }

        //compute blob radius
        {
            std::vector<double> dists;
            for (size_t pointIdx = 0; pointIdx < contours[contourIdx].size(); pointIdx++)
            {
                Point2d pt = contours[contourIdx][pointIdx];
                dists.push_back(norm(center.location - pt));
            }
            std::sort(dists.begin(), dists.end());
            center.radius = (dists[(dists.size() - 1) / 2] + dists[dists.size() / 2]) / 2.;
        }

        centers.push_back(center);
    }
}

void detect(cv::InputArray image, 
        std::vector<cv::KeyPoint>& keypoints, 
        const cv::SimpleBlobDetector::Params& params)
{
    //TODO: support mask
    keypoints.clear();
    Mat grayscaleImage;
    if (image.channels() == 3)
        cvtColor(image, grayscaleImage, COLOR_BGR2GRAY);
    else
        grayscaleImage = image.getMat();

    if (grayscaleImage.type() != CV_8UC1) {
        CV_Error(Error::StsUnsupportedFormat, "Blob detector only supports 8-bit images!");
    }

    std::vector < std::vector<Center> > centers;
    for (double thresh = params.minThreshold; thresh < params.maxThreshold; thresh += params.thresholdStep)
    {
        Mat binarizedImage;
        threshold(grayscaleImage, binarizedImage, thresh, 255, THRESH_BINARY);

        std::vector < Center > curCenters;
        findBlobs(grayscaleImage, binarizedImage, curCenters);
        std::vector < std::vector<Center> > newCenters;
        for (size_t i = 0; i < curCenters.size(); i++)
        {
            bool isNew = true;
            for (size_t j = 0; j < centers.size(); j++)
            {
                double dist = norm(centers[j][ centers[j].size() / 2 ].location - curCenters[i].location);
                isNew = dist >= params.minDistBetweenBlobs && dist >= centers[j][ centers[j].size() / 2 ].radius && dist >= curCenters[i].radius;
                if (!isNew)
                {
                    centers[j].push_back(curCenters[i]);

                    size_t k = centers[j].size() - 1;
                    while( k > 0 && centers[j][k].radius < centers[j][k-1].radius )
                    {
                        centers[j][k] = centers[j][k-1];
                        k--;
                    }
                    centers[j][k] = curCenters[i];

                    break;
                }
            }
            if (isNew)
                newCenters.push_back(std::vector<Center> (1, curCenters[i]));
        }
        std::copy(newCenters.begin(), newCenters.end(), std::back_inserter(centers));
    }

    for (size_t i = 0; i < centers.size(); i++)
    {
        if (centers[i].size() < params.minRepeatability)
            continue;
        Point2d sumPoint(0, 0);
        double normalizer = 0;
        for (size_t j = 0; j < centers[i].size(); j++)
        {
            sumPoint += centers[i][j].confidence * centers[i][j].location;
            normalizer += centers[i][j].confidence;
        }
        sumPoint *= (1. / normalizer);
        KeyPoint kpt(sumPoint, (float)(centers[i][centers[i].size() / 2].radius) * 2.0f);
        keypoints.push_back(kpt);
    }
}

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

    ////////////////////////// detect blobs //////////////////////////

    // keypoints
    std::vector<cv::KeyPoint> keypoints;

    // detect
    detect(I->image, keypoints, params);

    // results
    cv::Mat res;
    cv::drawKeypoints(I->image, 
            keypoints, 
            res, 
            cv::Scalar(0, 0, 255), 
            cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    ///////////////////////////////////////////////////////////////////

    // publish result
    cv_bridge::CvImage out;
    out.encoding = std::string("bgr8");
    out.image = res;
    event_image_pub.publish(out.toImageMsg());

    // publish keypoints
    blinker_tracking::KeyPointArray kps_msg;
    for (int i = 0; i < keypoints.size(); i++)
    {
        blinker_tracking::KeyPoint kp_msg;
        kp_msg.x = keypoints[i].pt.x;
        kp_msg.y = keypoints[i].pt.y;
        kp_msg.theta = keypoints[i].angle;
        kp_msg.size = keypoints[i].size;
        kps_msg.keypoints.push_back(kp_msg);
    }
    candidate_pub.publish(kps_msg);

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

    event_image_pub = it.advertise("image_out", 1);
    candidate_pub = nh.advertise<blinker_tracking::KeyPointArray>("candidates", 5);

    ros::spin();
    return 0;

}
