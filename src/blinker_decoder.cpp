#include <ros/ros.h>
#include <Eigen/Dense>
#include <opencv/cv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <unordered_map>
#include <queue>
#include <algorithm>

#include <blinker_tracking/BlinkersWithImage.h>

// parameter
double width;
double height;

std::vector< unsigned int > keys_0;
std::unordered_map< unsigned int, std::queue<int> > blinker_patterns;
std::unordered_map< unsigned int, cv::Mat > subframes_0;

void blinker_callback(const blinker_tracking::BlinkersWithImage::ConstPtr &msg)
{
    // for each blinker being tracked
    std::vector< unsigned int > keys;
    for (int i = 0; i < msg->blinkers.size(); i++)
    {
        // extract data
        unsigned int id = msg->blinkers[i].id;
        keys.push_back(id);

        // extract image
        cv_bridge::CvImagePtr image_ptr;
        image_ptr = cv_bridge::toCvCopy(msg->image);

        // extract patch
        cv::Mat patch;
        cv::getRectSubPix(image_ptr->image,
                cv::Size(width, height),
                cv::Point2f(msg->blinkers[i].u, msg->blinkers[i].v),
                patch);

        // check if the blinker has been seen
        if( blinker_patterns.find(id) == blinker_patterns.end() )
        {

            // save patch
            subframes_0[id] = patch;

            // add id to keys_0
            keys_0.push_back(id);

        } else {

            // compute high-low
            cv::Mat intensity_change = patch - subframes_0[id];

            // treshold

            // append to patterns list

            // save patch

        }

    }

    // sort the two sets of keys
    std::sort(keys_0.begin(), keys_0.end());
    std::sort(keys.begin(), keys.end());

    // find the keys seen in the last few frames but not in the current frame
    std::vector< unsigned int >::iterator it;
    std::vector< unsigned int > diff(keys_0.size() + keys.size());
    it = std::set_difference(
            keys_0.begin(),
            keys_0.end(),
            keys.begin(),
            keys.end(),
            diff.begin());
    diff.resize(it - diff.begin());

    // remove each key corresponding to a blinker that has gone missing
    for (int i = 0; i < diff.size(); i++)
    {
        blinker_patterns.erase(diff[i]);
        subframes_0.erase(diff[i]);
    }

    // save keys
    keys_0.clear();
    keys_0 = keys;

}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "blinker_decoder");
    ros::NodeHandle nh("~");

    nh.param(std::string("width"), width, 50.0);
    nh.param(std::string("height"), height, 50.0);

    ros::Subscriber blinker_sub;
    blinker_sub = nh.subscribe("blinkers", 10, &blinker_callback);

    ros::Subscriber blinker_database_sub;
    // blinker_database_sub = nh.subscribe("blinkers", 10, &bdb_callback);

    ros::spin();
    return 0;

}
