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

struct Center
{
    cv::Point2d location;
    double radius;
    double confidence;
};

void findBlobs(cv::InputArray _binaryImage, std::vector<Center> &centers)
{
    cv::Mat binaryImage = _binaryImage.getMat();
    centers.clear();

    std::vector < std::vector<cv::Point> > contours;
    cv::Mat tmpBinaryImage = binaryImage.clone();
    cv::findContours(tmpBinaryImage, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

    for (size_t contourIdx = 0; contourIdx < contours.size(); contourIdx++)
    {
        Center center;
        center.confidence = 1;
        cv::Moments moms = cv::moments(cv::Mat(contours[contourIdx]));
        if (params.filterByArea)
        {
            double area = moms.m00;
            if (area < params.minArea || area >= params.maxArea)
                continue;
        }

        if (params.filterByCircularity)
        {
            double area = moms.m00;
            double perimeter = cv::arcLength(cv::Mat(contours[contourIdx]), true);
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
            std::vector < cv::Point > hull;
            cv::convexHull(cv::Mat(contours[contourIdx]), hull);
            double area = cv::contourArea(cv::Mat(contours[contourIdx]));
            double hullArea = cv::contourArea(cv::Mat(hull));
            double ratio = area / hullArea;
            if (ratio < params.minConvexity || ratio >= params.maxConvexity)
                continue;
        }

        if(moms.m00 == 0.0)
            continue;
        center.location = cv::Point2d(moms.m10 / moms.m00, moms.m01 / moms.m00);

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
                cv::Point2d pt = contours[contourIdx][pointIdx];
                dists.push_back(cv::norm(center.location - pt));
            }
            std::sort(dists.begin(), dists.end());
            center.radius = (dists[(dists.size() - 1) / 2] + dists[dists.size() / 2]) / 2.;
        }

        centers.push_back(center);
    }
}

void detect(cv::InputArray image, std::vector<cv::KeyPoint>& keypoints)
{
    // clean up input
    keypoints.clear();
    cv::Mat grayscaleImage;
    if (image.channels() == 3)
        cv::cvtColor(image, grayscaleImage, cv::COLOR_BGR2GRAY);
    else
        grayscaleImage = image.getMat();

    if (grayscaleImage.type() != CV_8UC1) {
        CV_Error(cv::Error::StsUnsupportedFormat, "Blob detector only supports 8-bit images!");
    }

    // initialize centers - contains blob centers at each threshold level
    std::vector < std::vector<Center> > centers;

    // for each level of thresholding
    for (double thresh = params.minThreshold; thresh < params.maxThreshold; thresh += params.thresholdStep)
    {

        // threshold
        cv::Mat binarizedImage;
        cv::threshold(grayscaleImage, binarizedImage, thresh, 255, cv::THRESH_BINARY);

        // find blobs at current threshold level
        std::vector < Center > curCenters;
        findBlobs(binarizedImage, curCenters);

        // for each blob found at the current level
        std::vector < std::vector<Center> > newCenters;
        for (size_t i = 0; i < curCenters.size(); i++)
        {

            // assume that it is newly seen
            bool isNew = true;

            // for the blobs at each level of thresholding
            for (size_t j = 0; j < centers.size(); j++)
            {

                // compute euclidean distance between current blob i and
                //      every other blob within the last half of all levels
                //      NOTE: it would be unlikely for a blob to exist in the
                //      first few levels, disappear, then reappear. Searching
                //      the last half speeds up computation
                double dist = norm(centers[j][ centers[j].size() / 2 ].location - curCenters[i].location);

                // check for correspondence between blob i and blob j
                isNew = dist >= params.minDistBetweenBlobs && \
                        dist >= centers[j][ centers[j].size() / 2 ].radius && \
                        dist >= curCenters[i].radius;

                // if correspondence is detected between blob i and blob j
                if (!isNew)
                {

                    // append the new instance of blob j found at the current
                    //      level to its parent list (row of centers) which
                    //      tracks the blob accross levels
                    centers[j].push_back(curCenters[i]);

                    // let k be the index of the last observation of blob j
                    //      within its row
                    size_t k = centers[j].size() - 1;

                    // shift previous observations up such that the newest
                    //      observation is placed in the position which preserves
                    //      nondecreasing order of radii within a row
                    while( k > 0 && centers[j][k].radius < centers[j][k-1].radius )
                    {
                        centers[j][k] = centers[j][k-1];
                        k--;
                    }

                    // place newest observation of blob j in its approprate
                    //      position within its row which preserves
                    //      nondecreasing order of radii
                    centers[j][k] = curCenters[i];

                    // after the correspondence is detected and processed
                    //      continue with the next blob found in the current
                    //      level of thresholding
                    break;
                }
            }

            // if the blob found in the current level is new
            if (isNew)

                // append a new list of centers initialized with blob j
                //      seen at the current level
                newCenters.push_back(std::vector<Center> (1, curCenters[i]));
        }

        // vertically stack newCenters and centers (append new unique blobs)
        std::copy(newCenters.begin(), newCenters.end(), std::back_inserter(centers));
    }

    // for each blob
    for (size_t i = 0; i < centers.size(); i++)
    {

        // if the blob has not persisted across enough levels of thresholding
        //      then process next
        if (centers[i].size() < params.minRepeatability)
            continue;

        // compute the "average" center of blob across the observed levels
        cv::Point2d sumPoint(0, 0);
        double normalizer = 0;
        for (size_t j = 0; j < centers[i].size(); j++)
        {
            sumPoint += centers[i][j].confidence * centers[i][j].location;
            normalizer += centers[i][j].confidence;
        }
        sumPoint *= (1. / normalizer);

        // convert the center to a keypoint and append it to the list
        cv::KeyPoint kpt(sumPoint, (float)(centers[i][centers[i].size() / 2].radius) * 2.0f);
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
    detect(I->image, keypoints);

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

void loadOpenCVDefaults()
{
    params.thresholdStep = 10;
    params.minThreshold = 50;
    params.maxThreshold = 220;
    params.minRepeatability = 2;
    params.minDistBetweenBlobs = 10;

    params.filterByColor = true;
    params.blobColor = 0;

    params.filterByArea = true;
    params.minArea = 25;
    params.maxArea = 5000;

    params.filterByCircularity = false;
    params.minCircularity = 0.8f;
    params.maxCircularity = std::numeric_limits<float>::max();

    params.filterByInertia = true;
    // params.minInertiaRatio = 0.6;
    params.minInertiaRatio = 0.1f;
    params.maxInertiaRatio = std::numeric_limits<float>::max();

    params.filterByConvexity = true;
    // params.minConvexity = 0.8;
    params.minConvexity = 0.95f;
    params.maxConvexity = std::numeric_limits<float>::max();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "blinker_detection_node");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);

    loadOpenCVDefaults();

    // parameters
    nh.param(std::string("thresholdStep"),          params.thresholdStep,       (float) 10);
    nh.param(std::string("minThreshold"),           params.minThreshold,        (float) 128);
    nh.param(std::string("maxThreshold"),           params.maxThreshold,        (float) 255);

    int blobColor;
    nh.param(std::string("filterByColor"),          params.filterByColor,       false);
    nh.param(std::string("blobColor"),              blobColor,                  255);
    params.blobColor = (unsigned char) blobColor;

    nh.param(std::string("filterByArea"),           params.filterByArea,        false);
    nh.param(std::string("minArea"),                params.minArea,             (float) 500);
    nh.param(std::string("maxArea"),                params.maxArea,             (float) 1000);

    nh.param(std::string("filterByCircularity"),    params.filterByCircularity, false);
    nh.param(std::string("minCircularity"),         params.minCircularity,      (float) 0.5);

    nh.param(std::string("filterByConvexity"),      params.filterByConvexity,   false);
    nh.param(std::string("minConvexity"),           params.minConvexity,        (float) 0.4);

    nh.param(std::string("filterByInertia"),        params.filterByInertia,     false);
    nh.param(std::string("minInertiaRatio"),        params.minInertiaRatio,     (float) 0.2);
    nh.param(std::string("minDistBetweenBlobs"),    params.minDistBetweenBlobs, (float) 100);

    ros::Subscriber sub;
    sub = nh.subscribe("image_raw", 10, &callback);

    event_image_pub = it.advertise("image_out", 1);
    candidate_pub = nh.advertise<blinker_tracking::KeyPointArray>("candidates", 5);

    ros::spin();
    return 0;

}
