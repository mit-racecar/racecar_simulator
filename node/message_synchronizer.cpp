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

    scan_sub = nh_.subscribe(scan_topic, 10, &MsgSync::scan_callback, this);
    img_sub = it.subscribe(img_topic, 10, &MsgSync::img_callback, this);
    odom_sub = nh_.subscribe(odom_topic, 10, &MsgSync::odom_callback, this);
    collision_sub = nh_.subscribe(collision_topic, 10, &MsgSync::collision_callback, this);
}

void MsgSync::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
    current_scan = *scan_msg;
    scan_here = true;
    check_sync();
}

void MsgSync::odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
    current_odom = *odom_msg;
    odom_here = true;
    check_sync();
}

void MsgSync::collision_callback(const std_msgs::Bool::ConstPtr &collision_msg){
    current_collision.data = collision_msg->data;
    collision_here = true;
    check_sync();
}

void MsgSync::img_callback(const sensor_msgs::Image::ConstPtr &img_msg){
    current_image = *img_msg;
    image_here = true;
    check_sync();
}

void MsgSync::check_sync() {
    // called in every callback
    // if bool toggles all true, create a mega message and publish
    if (scan_here && odom_here && collision_here && image_here) {
        racecar_simulator::SynchronizedSensor synced_msg;
        synced_msg.laserscan = current_scan;
        synced_msg.odom = current_odom;
        synced_msg.collision = current_collision.data;
        synced_msg.img = current_image;
        mega_pub.publish(synced_msg);
        scan_here = false;
        odom_here = false;
        collision_here = false;
        image_here = false;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "message_synchronizer");
    ros::NodeHandle nh;
    MsgSync MsgSync(nh);
    ros::spin();
    return 0;
}