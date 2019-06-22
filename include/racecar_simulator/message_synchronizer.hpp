#include <ros/ros.h>
#include <racecar_simulator/SynchronizedSensor.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <vector>

// list of existing sensors
// [scan, odom, collision, top_down_img]
// list of proposed sensors
// [imu?, wheel_vel?]

class MsgSync {
public:
    MsgSync(ros::NodeHandle &nh);
    virtual ~MsgSync();
private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber collision_sub;
    image_transport::ImageTransport it;
    image_transport::Subscriber img_sub;
    
    ros::Publisher mega_pub;
};