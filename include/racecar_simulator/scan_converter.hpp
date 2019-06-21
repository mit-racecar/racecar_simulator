// ros
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/image_encodings.h>
// image
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
// standard
#include <vector>

class ScanConverter {
public:
    ScanConverter(ros::NodeHandle &nh);
    virtual ~ScanConverter();
    cv::Mat update_scan(std::vector<float> &ranges, std::vector<float> &angles_vector, int scan_count);
private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub;
    image_transport::ImageTransport it;
    image_transport::Publisher img_pub;
    // image params
    int img_width, img_height;
    // fixed car center position in image;
    int car_x, car_y;
    int car_width, car_length;
    double img_res;
    // colors
    int bg_r, bg_g, bg_b, road_r, road_g, road_b, car_r, car_g, car_b;

    std::vector<float> angles_vector;
    int scan_count;

    bool out_of_bounds(int img_x, int img_y);

};