#include "racecar_simulator/message_synchronizer.hpp"
// list of existing sensors
// [scan, odom, collision, top_down_img]
// list of proposed sensors
// [imu?, wheel_vel?]

MsgSync::~MsgSync() {
    ROS_INFO("Message Synchronizer shutting down.");
}

MsgSync::MsgSync(ros::NodeHandle &nh): nh_(nh), it(nh) {
    std::string img_topic, scan_topic, odom_topic, collision_topic;
    nh_.getParam("scan_topic", scan_topic);
    nh_.getParam("converted_img_toic", img_topic);
    nh_.getParam("odom_topic", odom_topic);
    nh_.getParam("collision_topic", collision_topic);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "message_synchronizer");
    ros::NodeHandle nh;
    MsgSync MsgSync(nh);
    ros::spin();
    return 0;
}