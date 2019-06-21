#include "racecar_simulator/scan_converter.hpp"

ScanConverter::~ScanConverter() {
    ROS_INFO("Scan Converter shutting down.");
}

ScanConverter::ScanConverter(ros::NodeHandle &nh): nh_(nh), it(nh) {
    // ros
    std::string img_topic;
    nh_.getParam("converted_image_topic", img_topic);
    img_pub = it.advertise(img_topic, 1);
    std::string scan_topic;
    nh_.getParam("scan_topic", scan_topic);
    scan_sub = nh_.subscribe(scan_topic, 10, &ScanConverter::scan_callback, this);

    // image params
    nh_.getParam("image_size", img_width);
    nh_.getParam("image_size", img_height);
    nh_.getParam("image_res", img_res);
    nh_.getParam("car_width", car_width);
    nh_.getParam("car_legnth", car_length);
    car_x = img_width/2;
    car_y = img_heigth-15;

    // colors
    nh_.getParam("background_r", bg_r);
    nh_.getParam("background_g", bg_g);
    nh_.getParam("background_b", bg_b);
    nh_.getParam("road_r", road_r);
    nh_.getParam("road_g", road_g);
    nh_.getParam("road_b", road_b);
    nh_.getParam("car_r", car_r);
    nh_.getParam("car_g", car_g);
    nh_.getParam("car_b", car_b);

    // scan info
    boost::shared_ptr<sensor_msgs::LaserScan const> laser_ptr;
    laser_ptr = ros::topic::waitForMessage<sensor_msgs::LaserScan>(scan_topic);
    if (laser_ptr == NULL) ROS_ERROR("Laser message null.");
    scan_count = (laser_ptr->ranges).size();
    angles_vector.reserve(scan_count);
    for (int i=0; i<scan_count; i++) {
        angles_vector[i] = laser_ptr->angle_min + laser_ptr->angle_increment*i;
    }
    ROS_INFO("Laser params loaded.");
    ROS_INFO("Scan Converter created.");
}

void ScanConverter::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
    std::vector<float> ranges = scan_msg->ranges;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Point> road_contour;
    for (int i=0; i<scan_count; i++) {
        double range = ranges[i];
        if (std::isnan(range) || std::isinf(range)) continue;
        double x = -range*sin(angles_vector[i]), y = range*cos(angles_vector[i]);
        double img_x = car_x + x/img_res, img_y = car_y - y/img_res;
        if (out_of_bounds(img_x, img_y)) continue;
        cv::Point current_point(img_x, img_y);
        road_contour.push_back(current_point);
    }
    contours.push_back(road_contour);
    // background
    cv::Mat img(img_height, img_width, CV_8UC3, cv::Scalar(bg_b, bg_g, bg_r));
    // road
    cv::Scalar road_color(road_b, road_g, road_r);
    cv::drawContours(img, contours, 0, road_color, CV_FILLED);
    // car
    cv::Point car_top_left(car_x-car_width/2, car_y-car_length/2);
    cv::Point car_bot_right(car_x+car_width/2, car_y+car_length/2);
    cv::rectangle(img, car_top_left, car_bot_right, cv::Scalar(car_b, car_g, car_r));
    sensor_msgs::ImagePtr ros_img;
    if (!img.empty()) {
        ros_img = cv::bridge::CvImage(std::msgs::Header(), "bgr8", img).toImageMsg();
    }
    img_pub.publish(ros_img);
}

cv::Mat ScanConverter::update_scan(std::vector<float> &ranges, std::vector<float> &angles_vector, int scan_count) {
    // update scan and draw image
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Point> road_contour;
    for (int i=0; i<scan_count; i++) {
        double range = ranges[i];
        if (std::isnan(range) || std::isinf(range)) continue;
        double x = -range*sin(angles_vector[i]), y = range*cos(angles_vector[i]);
        double img_x = car_x + x/img_res, img_y = car_y - y/img_res;
        if (out_of_bounds(img_x, img_y)) continue;
        cv::Point current_point(img_x, img_y);
        road_contour.push_back(current_point);
    }
    contours.push_back(road_contour);
    // create blank green image
    cv::Mat img(img_height, img_width, CV_8UC3, cv::Scalar(0, 255, 0));
    // draw gray contour of road
    cv::Scalar road_color(100, 100, 100);
    cv::drawContours(img, contours, 0, road_color, CV_FILLED);
    // draw red car as box
    cv::Point car_top_left(car_x-car_width/2, car_y-car_length/2);
    cv::Point car_bot_right(car_x+car_width/2, car_y+car_length/2);
    cv::rectangle(img, car_top_left, car_bot_right, cv::Scalar(0, 0, 255), CV_FILLED);
    return img;
}

bool ScanConverter::out_of_bounds(int img_x, int img_y) {
    return (img_x < 0 || img_x >= img_width || img_y < 0 || img_y >= img_height);
}

int main(int argc, char const** argv)
{
    ros::init(argc, argv, "scan_converter");
    ros::NodeHandle nh;
    ScanConverter ScanConverter(nh);
    ros::spin();
    return 0;
}