#include <ros/ros.h>
#include <Eigen/Dense>
#include <opencv/cv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>

#include <fstream>
#include <vector>
#include <deque>
#include <unordered_map>
#include <algorithm>
#include <yaml-cpp/yaml.h>

#include <blinker_tracking/BlinkersWithImage.h>

/*
 * TODO : Autosize height and width of patches based on the covariance
 * matrices
 */

struct BlinkerID
{
    unsigned int id;
    unsigned int blink_code;
};

image_transport::Publisher debug_pub;

// parameter
int intensity_change_thresh;
int vote_thresh;
double width;
double height;

std::vector< unsigned int > keys_0;
std::unordered_map< unsigned int, std::deque< cv::Mat > > subframes;

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

        // save patch
        subframes[id].push_back(patch);

        // maintain size
        if (subframes[id].size() < 18)
        {
            continue;
        }

        unsigned int even = 0;
        unsigned int odd = 0;
        for (int j = 0; j < 7; j++)
        {
            // compute intensity change
            cv::Mat m_even;
            cv::Mat m_odd;
            cv::absdiff(subframes[id][2*j  ], subframes[id][2*j+2], m_even);
            cv::absdiff(subframes[id][2*j+1], subframes[id][2*j+3], m_odd);

            cv::Mat bm_even;
            cv::Mat bm_odd;
            cv::threshold(m_even, bm_even, intensity_change_thresh, 255, cv::THRESH_BINARY);
            cv::threshold(m_odd, bm_odd, intensity_change_thresh, 255, cv::THRESH_BINARY);

            // ratio of high to low pixels
            int p_even = cv::countNonZero(bm_even);
            int p_odd = cv::countNonZero(bm_odd);

            // signal
            unsigned int s_even = (p_even > vote_thresh) ? 1 : 0;
            unsigned int s_odd = (p_odd > vote_thresh) ? 1 : 0;

            // append to patterns list
            even = even + (s_even << j);
            odd = odd + (s_odd << j);

            // publish image to test 
            if (j == 0)
            {
                cv_bridge::CvImage out;
                out.encoding = std::string("mono8");
                out.image = bm_even;

                sensor_msgs::ImagePtr image_ros = out.toImageMsg();
                image_ros->header.stamp = ros::Time::now();
                debug_pub.publish(image_ros);
            }

        }

        std::cout << "Even Pattern : " << even << std::endl;
        std::cout << "Odd Pattern : " << odd << std::endl;

        // pop oldest to maintain size
        subframes[id].pop_front();

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
        subframes.erase(diff[i]);
    }

    // save keys
    keys_0.clear();
    keys_0 = keys;

}

void load_blinker_database(std::string f, std::vector< BlinkerID >& blinker_ids)
{
    // open file
    std::fstream fs;
    fs.open(f.c_str());
    if (!fs.is_open())
    {
        ROS_ERROR("Blinker database not found");
    }
    YAML::Node map = YAML::LoadFile(f.c_str());
    assert(map.IsSequence());

    for (int i = 0; i < map.size(); i++)
    {
        BlinkerID bid;
        bid.id = (unsigned int) map[i]["id"].as<int>();
        bid.blink_code = (unsigned int) map[i]["blink_code"].as<int>();
        blinker_ids.push_back(bid);
    }
    
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "blinker_decoder");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);

    // load parameters
    std::string blinker_database;
    nh.param(std::string("blinker_database"), blinker_database, std::string(""));
    nh.param(std::string("vote_thresh"), vote_thresh, 30);
    nh.param(std::string("intensity_change_thresh"), intensity_change_thresh, 200);
    nh.param(std::string("width"), width, 80.0);
    nh.param(std::string("height"), height, 80.0);

    // load blinker database
    std::vector< BlinkerID > blinker_ids;
    load_blinker_database(blinker_database, blinker_ids);
    
    for(int i = 0; i < blinker_ids.size(); i++)
    {
        std::cout << blinker_ids[i].id << std::endl;
        std::cout << blinker_ids[i].blink_code << std::endl;
    }
    std::cout << std::endl;

    ros::Subscriber blinker_sub;
    blinker_sub = nh.subscribe("blinkers", 10, &blinker_callback);

    ros::Subscriber blinker_database_sub;
    // blinker_database_sub = nh.subscribe("blinkers", 10, &bdb_callback);

    debug_pub = it.advertise("image_out", 10);

    ros::spin();
    return 0;

}
