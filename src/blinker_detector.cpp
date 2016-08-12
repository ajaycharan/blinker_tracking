#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <blinker_tracking/BlobFeature.h>
#include <blinker_tracking/BlobFeatureArray.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.hpp>

#include <vector>
#include <boost/algorithm/cxx11/is_sorted.hpp>

image_transport::Publisher event_image_pub;
ros::Publisher candidate_pub;

// globals
bool is_init = 0;
float areaWeight;
float circularityWeight;
float confidenceWeight;
float peakRiseThreshold;
cv::SimpleBlobDetector::Params params;

cv::Mat prev_descriptors;

struct Blob
{
    cv::Point2d location;
    double radius;
    double confidence;

    double area;
    double circularity;
};

void detectPeakRise(
        cv::Mat descriptors,
        cv::Mat prev_descriptors,
        std::vector<int>& candidateIds)
{
    // check that something new has been detected in the frame
    if (descriptors.rows == 0)
        return;

    // if there are no existing peaks in the last frame add all the new
    //      peaks as candidates
    if (prev_descriptors.rows == 0)
    {
        for(int i = 0; i < descriptors.rows; i++)
        {
            candidateIds.push_back(i);
        }
        return;
    }

    // match similar peaks across frames
    cv::FlannBasedMatcher matcher;
    std::vector< std::vector<cv::DMatch> > matches;
    matcher.knnMatch(descriptors, prev_descriptors, matches, 1);

    // for each peak
    for (int i = 0; i < matches.size(); i++)
    {
        if (matches[i][0].distance > peakRiseThreshold)
        {
            candidateIds.push_back((int) matches[i][0].queryIdx);
        }
    }

}


void findBlobs(cv::InputArray _binaryImage, std::vector<Blob> &blobs)
{
    cv::Mat binaryImage = _binaryImage.getMat();
    blobs.clear();

    std::vector < std::vector<cv::Point> > contours;
    cv::Mat tmpBinaryImage = binaryImage.clone();
    cv::findContours(tmpBinaryImage, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

    for (size_t contourIdx = 0; contourIdx < contours.size(); contourIdx++)
    {
        Blob blob;
        blob.area = 1;
        blob.circularity = 1;
        blob.confidence = 1;
        cv::Moments moms = cv::moments(cv::Mat(contours[contourIdx]));
        if (params.filterByArea)
        {
            double area = moms.m00;
            if (area < params.minArea || area >= params.maxArea)
                continue;
            blob.area = area;
        }

        if (params.filterByCircularity)
        {
            double area = moms.m00;
            double perimeter = cv::arcLength(cv::Mat(contours[contourIdx]), true);
            double ratio = 4 * CV_PI * area / (perimeter * perimeter);
            if (ratio < params.minCircularity || ratio >= params.maxCircularity)
                continue;
            blob.circularity = ratio;
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

            blob.confidence = ratio * ratio;
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
        blob.location = cv::Point2d(moms.m10 / moms.m00, moms.m01 / moms.m00);

        if (params.filterByColor)
        {
            if (binaryImage.at<uchar> (cvRound(blob.location.y), cvRound(blob.location.x)) != params.blobColor)
                continue;
        }

        //compute blob radius
        {
            std::vector<double> dists;
            for (size_t pointIdx = 0; pointIdx < contours[contourIdx].size(); pointIdx++)
            {
                cv::Point2d pt = contours[contourIdx][pointIdx];
                dists.push_back(cv::norm(blob.location - pt));
            }
            std::sort(dists.begin(), dists.end());
            blob.radius = (dists[(dists.size() - 1) / 2] + dists[dists.size() / 2]) / 2.;
        }

        blobs.push_back(blob);
    }
}

void findPeaks(cv::InputArray image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors)
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

    // initialize blobs - i : blob; j : levels
    std::vector < std::vector<Blob> > blobs;

    // for each level of thresholding from maxThreshold down to minThreshold
    for (double thresh = params.maxThreshold; thresh > params.minThreshold; thresh -= params.thresholdStep)
    {

        // threshold
        cv::Mat binarizedImage;
        cv::threshold(grayscaleImage, binarizedImage, thresh, 255, cv::THRESH_BINARY);

        // find blobs at current threshold level
        std::vector < Blob > curBlobs;
        findBlobs(binarizedImage, curBlobs);

        // for each blob found at the current level
        std::vector < std::vector<Blob> > newBlobs;
        for (size_t i = 0; i < curBlobs.size(); i++)
        {

            // assume that it is newly seen
            bool isNew = true;

            // for the blobs at each level of thresholding
            for (size_t j = 0; j < blobs.size(); j++)
            {

                // compute euclidean distance between current blob i and
                //      every other blob -- choosing the blob observation 
                //      with the median radius
                double dist = norm(blobs[j][ blobs[j].size() / 2 ].location - curBlobs[i].location);

                // check for correspondence between blob i and blob j
                isNew = dist >= params.minDistBetweenBlobs && \
                        dist >= blobs[j][ blobs[j].size() / 2 ].radius && \
                        dist >= curBlobs[i].radius;

                // if correspondence is detected between blob i and blob j
                if (!isNew)
                {

                    // append the new instance of blob j found at the current
                    //      level to its parent list which
                    //      tracks the blob accross levels
                    blobs[j].push_back(curBlobs[i]);

                    // let k be the index of the last observation of blob j
                    //      within its row
                    size_t k = blobs[j].size() - 1;

                    // shift previous observations up such that the newest
                    //      observation is placed in the position which preserves
                    //      nondecreasing order of radii within a row
                    while( k > 0 && blobs[j][k].radius < blobs[j][k-1].radius )
                    {
                        blobs[j][k] = blobs[j][k-1];
                        k--;
                    }

                    // place newest observation of blob j in its approprate
                    //      position within its row which preserves
                    //      nondecreasing order of radii
                    blobs[j][k] = curBlobs[i];

                    // after the correspondence is detected and processed
                    //      continue with the next blob found in the current
                    //      level of thresholding
                    break;
                }
            }

            // if the blob found in the current level is new
            if (isNew)

                // append a new list of blobs initialized with blob j
                //      seen at the current level
                newBlobs.push_back(std::vector<Blob> (1, curBlobs[i]));
        }

        // vertically stack newBlobs and blobs (append new unique blobs)
        std::copy(newBlobs.begin(), newBlobs.end(), std::back_inserter(blobs));
    }

    // reset descriptor array
    descriptors = cv::Mat();

    // for each blob
    for (size_t i = 0; i < blobs.size(); i++)
    {

        // if the blob has not persisted across enough levels of thresholding
        //      then skip
        if (blobs[i].size() < params.minRepeatability)
            continue;

        // compute the "average" center of blob across the observed levels
        cv::Point2d sumPoint(0, 0);
        double normalizer = 0;
        for (size_t j = 0; j < blobs[i].size(); j++)
        {
            sumPoint += blobs[i][j].confidence * blobs[i][j].location;
            normalizer += blobs[i][j].confidence;
        }
        sumPoint *= (1. / normalizer);

        // convert the blob to a keypoint and append it to the list
        cv::KeyPoint kpt(sumPoint, (float)(blobs[i][blobs[i].size() / 2].radius) * 2.0f);
        keypoints.push_back(kpt);

        // blob feature vector
        cv::Mat row = cv::Mat::zeros(1, 5, CV_32F);
        row.at<float>(0) = sumPoint.x;
        row.at<float>(1) = sumPoint.y;
        row.at<float>(2) =
            (float) blobs[i][blobs[i].size() / 2].area * areaWeight;
        row.at<float>(3) =
            (float) blobs[i][blobs[i].size() / 2].circularity * circularityWeight;
        row.at<float>(4) =
            (float) blobs[i][blobs[i].size() / 2].confidence * confidenceWeight;
        descriptors.push_back(row);


    }
}

void callback(const sensor_msgs::Image::ConstPtr &msg)
{

    // extract input
    cv_bridge::CvImagePtr I;
    I = cv_bridge::toCvCopy(msg);

    // find peaks
    std::vector<cv::KeyPoint> peaks;
    cv::Mat descriptors;
    findPeaks(I->image, peaks, descriptors);

    // save data and return if this is the first call
    if (!is_init)
    {
        prev_descriptors = descriptors;
        is_init = 1;
        return;
    }

    // find peaks not seen in the last frame
    std::vector<int> candidateIds;
    detectPeakRise(descriptors, prev_descriptors, candidateIds);

    // convert to list of keypoints for plotting
    std::vector<cv::KeyPoint> candidates;
    for (int i = 0; i < candidateIds.size(); i++)
    {
        candidates.push_back(peaks[candidateIds[i]]);
    }

    // results from spatial and temporal detection
    cv::Mat res;
    cv::drawKeypoints(I->image, 
            candidates, 
            res, 
            cv::Scalar(0, 0, 255), 
            cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    // publish image with overlay
    cv_bridge::CvImage out;
    out.encoding = std::string("bgr8");
    out.image = res;
    event_image_pub.publish(out.toImageMsg());

    // publish candidates
    blinker_tracking::BlobFeatureArray bfa;
    bfa.header.seq = msg->header.seq;
    bfa.header.stamp = msg->header.stamp;
    for (int i = 0; i < candidateIds.size(); i++)
    {
        blinker_tracking::BlobFeature bf;
        bf.x = descriptors.at<float>(candidateIds[i], 0);
        bf.y = descriptors.at<float>(candidateIds[i], 1);
        bf.area = descriptors.at<float>(candidateIds[i], 2);
        bf.circularity = descriptors.at<float>(candidateIds[i], 3);
        bf.confidence = descriptors.at<float>(candidateIds[i], 4);
        bfa.features.push_back(bf);
    }
    candidate_pub.publish(bfa);

    // save last set of keypoints
    prev_descriptors = descriptors;

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
    ros::init(argc, argv, "blinker_detector");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);

    loadOpenCVDefaults();

    // parameters
    int minRepeatability;
    nh.param(std::string("thresholdStep"),          params.thresholdStep,       (float) 10);
    nh.param(std::string("minThreshold"),           params.minThreshold,        (float) 128);
    nh.param(std::string("maxThreshold"),           params.maxThreshold,        (float) 255);
    nh.param(std::string("minRepeatability"),       minRepeatability,           (int) 2);
    params.minRepeatability = (unsigned long int) minRepeatability;

    int blobColor;
    nh.param(std::string("filterByColor"),          params.filterByColor,       false);
    nh.param(std::string("blobColor"),              blobColor,                  255);
    params.blobColor = (unsigned char) blobColor;

    nh.param(std::string("filterByArea"),           params.filterByArea,        false);
    nh.param(std::string("minArea"),                params.minArea,             (float) 25);
    nh.param(std::string("maxArea"),                params.maxArea,             (float) 5000);

    nh.param(std::string("filterByCircularity"),    params.filterByCircularity, false);
    nh.param(std::string("minCircularity"),         params.minCircularity,      (float) 0.5);

    nh.param(std::string("filterByConvexity"),      params.filterByConvexity,   false);
    nh.param(std::string("minConvexity"),           params.minConvexity,        (float) 0.4);

    nh.param(std::string("filterByInertia"),        params.filterByInertia,     false);
    nh.param(std::string("minInertiaRatio"),        params.minInertiaRatio,     (float) 0.2);
    nh.param(std::string("minDistBetweenBlobs"),    params.minDistBetweenBlobs, (float) 100);

    nh.param(std::string("areaWeight"),             areaWeight,                 (float) 1.0);
    nh.param(std::string("circularityWeight"),      circularityWeight,          (float) 0.0);
    nh.param(std::string("confidenceWeight"),       confidenceWeight,           (float) 0.0);
    nh.param(std::string("peakRiseThreshold"),      peakRiseThreshold,          (float) 100.0);

    ros::Subscriber sub;
    sub = nh.subscribe("image_raw", 30, &callback);

    event_image_pub = it.advertise("image_out", 1);
    candidate_pub = nh.advertise<blinker_tracking::BlobFeatureArray>("candidates", 5);

    ros::spin();
    return 0;

}
