#include <rclcpp/rclcpp.hpp>

#include <tf2/impl/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include "racecar_simulator/pose_2d.hpp"
#include "racecar_simulator/ackermann_kinematics.hpp"
#include "racecar_simulator/scan_simulator_2d.hpp"

namespace racecar_simulator {

class RacecarSimulator : public rclcpp::Node {
  private:
    // The transformation frames used
    std::string map_frame_, base_frame_, scan_frame_;

    // The car state and parameters
    Pose2D pose_;
    double wheelbase_;
    double speed_;
    double steering_angle_;
    rclcpp::Time previous_pose_update_;
    double scan_distance_to_base_link_;
    double max_speed_, max_steering_angle_;

    // A simulator of the laser
    ScanSimulator2D scan_simulator_;
    double map_free_threshold_;

    // Joystick parameters
    int joy_speed_axis_, joy_angle_axis_;
    double joy_max_speed_;

    // For publishing transformations
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf2_br_;

    // A timer to update the pose
    rclcpp::TimerBase::SharedPtr update_pose_timer_;

    // Listen for drive and joystick commands
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    // Listen for a map
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    bool map_exists = false;

    // Listen for updates to the pose
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_rviz_sub_;

    // Publish a scan, odometry, and imu data
    bool broadcast_transform_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  public:

    RacecarSimulator() : rclcpp::Node("racecar_simulator") {

      // Initialize the pose and driving commands
      pose_ = {0, 0, 0};
      speed_ = 0;
      steering_angle_ = 0;
      previous_pose_update_ = this->get_clock()->now();

      // Get the topic names
      std::string joy_topic, drive_topic, map_topic, 
        scan_topic, pose_topic, pose_rviz_topic, odom_topic;

      joy_topic       = declare_parameter<std::string>("joy_topic");
      drive_topic     = declare_parameter<std::string>("drive_topic");
      map_topic       = declare_parameter<std::string>("map_topic");
      scan_topic      = declare_parameter<std::string>("scan_topic");
      pose_topic      = declare_parameter<std::string>("pose_topic");
      pose_rviz_topic = declare_parameter<std::string>("odom_topic");
      odom_topic      = declare_parameter<std::string>("pose_rviz_topic");

      // Get the transformation frame names
      map_frame_  = declare_parameter<std::string>("map_frame");
      base_frame_ = declare_parameter<std::string>("base_frame");
      scan_frame_ = declare_parameter<std::string>("scan_frame");

      // Fetch the car parameters
      int scan_beams;
      double update_pose_period, scan_field_of_view, scan_std_dev;
      wheelbase_                 = declare_parameter<double>("wheelbase");
      update_pose_period         = declare_parameter<double>("update_pose_period");
      scan_beams                 = declare_parameter<int>   ("scan_beams");
      scan_field_of_view         = declare_parameter<double>("scan_field_of_view");
      scan_std_dev               = declare_parameter<double>("scan_std_dev");
      map_free_threshold_        = declare_parameter<double>("map_free_threshold");
      scan_distance_to_base_link_= declare_parameter<double>("scan_distance_to_base_link");
      max_speed_                 = declare_parameter<double>("max_speed");
      max_steering_angle_        = declare_parameter<double>("max_steering_angle");

      // Get joystick parameters
      bool joystick_enabled;
      joystick_enabled = declare_parameter<bool>  ("joy");
      joy_speed_axis_  = declare_parameter<int>   ("joy_speed_axis");
      joy_angle_axis_  = declare_parameter<int>   ("joy_angle_axis");
      joy_max_speed_   = declare_parameter<double>("joy_max_speed");

      // Determine if we should broadcast
      broadcast_transform_ = declare_parameter<bool>("broadcast_transform");

      if (broadcast_transform_) {
        tf2_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
      }

      scan_simulator_ = ScanSimulator2D(
          scan_beams,
          scan_field_of_view,
          scan_std_dev);

      scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic, rclcpp::QoS(1));

      odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 1);

      // Start a timer to output the pose
      update_pose_timer_ = this->create_wall_timer(std::chrono::duration<double>(update_pose_period),
        std::bind(&RacecarSimulator::update_pose, this));

      // If the joystick is enabled
      if (joystick_enabled) 
      {
        // Start a subscriber to listen to joystick commands
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(joy_topic, rclcpp::QoS(1), 
          std::bind(&RacecarSimulator::joy_callback, this, std::placeholders::_1));

      }

      // Start a subscriber to listen to drive commands
      drive_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, rclcpp::QoS(1), 
        std::bind(&RacecarSimulator::drive_callback, this, std::placeholders::_1));

      // Start a subscriber to listen to new maps
      map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(map_topic, rclcpp::QoS(1),
        std::bind(&RacecarSimulator::map_callback, this, std::placeholders::_1));


      // Start a subscriber to listen to pose messages
      pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(pose_topic, rclcpp::QoS(1), 
        std::bind(&RacecarSimulator::pose_callback, this, std::placeholders::_1));

      pose_rviz_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_rviz_topic, rclcpp::QoS(1), 
        std::bind(&RacecarSimulator::pose_rviz_callback, this, std::placeholders::_1));

    }

    void update_pose() {

      // Update the pose
      rclcpp::Time now = this->get_clock()->now();
      rclcpp::Time timestamp = now;
      pose_ = AckermannKinematics::update(
          pose_, 
          speed_,
          steering_angle_,
          wheelbase_,
          (now - previous_pose_update_).seconds());
      previous_pose_update_ = now;

      // Convert the pose into a transformation
      geometry_msgs::msg::Transform t;
      t.translation.x = pose_.x;
      t.translation.y = pose_.y;
      tf2::Quaternion quat;
      quat.setEuler(0., 0., pose_.theta);
      t.rotation.x = quat.x();
      t.rotation.y = quat.y();
      t.rotation.z = quat.z();
      t.rotation.w = quat.w();

      // Add a header to the transformation
      geometry_msgs::msg::TransformStamped ts;
      ts.transform = t;
      ts.header.stamp = timestamp;
      ts.header.frame_id = map_frame_;
      ts.child_frame_id = base_frame_;

      // Make an odom message as well
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = timestamp;
      odom.header.frame_id = map_frame_;
      odom.child_frame_id = base_frame_;
      odom.pose.pose.position.x = pose_.x;
      odom.pose.pose.position.y = pose_.y;
      odom.pose.pose.orientation.x = quat.x();
      odom.pose.pose.orientation.y = quat.y();
      odom.pose.pose.orientation.z = quat.z();
      odom.pose.pose.orientation.w = quat.w();
      odom.twist.twist.linear.x = speed_;
      odom.twist.twist.angular.z = 
        AckermannKinematics::angular_velocity(speed_, steering_angle_, wheelbase_);

      // Publish them
      if (broadcast_transform_) tf2_br_->sendTransform(ts);
      odom_pub_->publish(odom);
      // Set the steering angle to make the wheels move
      set_steering_angle(steering_angle_, timestamp);

      // If we have a map, perform a scan
      if (map_exists) {
        // Get the pose of the lidar, given the pose of base link
        // (base link is the center of the rear axle)
        Pose2D scan_pose;
        scan_pose.x = pose_.x + scan_distance_to_base_link_ * std::cos(pose_.theta);
        scan_pose.y = pose_.y + scan_distance_to_base_link_ * std::sin(pose_.theta);
        scan_pose.theta = pose_.theta;

        // Compute the scan from the lidar
        std::vector<double> scan = scan_simulator_.scan(scan_pose);

        // Convert to float
        std::vector<float> scan_(scan.size());
        for (size_t i = 0; i < scan.size(); i++)
          scan_[i] = scan[i];

        // Publish the laser message
        sensor_msgs::msg::LaserScan scan_msg;
        scan_msg.header.stamp = timestamp;
        scan_msg.header.frame_id = scan_frame_;
        scan_msg.angle_min       = -scan_simulator_.get_field_of_view()/2.;
        scan_msg.angle_max       =  scan_simulator_.get_field_of_view()/2.;
        scan_msg.angle_increment =  scan_simulator_.get_angle_increment();
        scan_msg.range_max = 100;
        scan_msg.ranges = scan_;
        scan_msg.intensities = scan_;

        scan_pub_->publish(scan_msg);

        // Publish a transformation between base link and laser
        geometry_msgs::msg::TransformStamped scan_ts;
        scan_ts.transform.translation.x = scan_distance_to_base_link_;
        scan_ts.transform.rotation.w = 1;
        scan_ts.header.stamp = timestamp;
        scan_ts.header.frame_id = base_frame_;
        scan_ts.child_frame_id = scan_frame_;
        tf2_br_->sendTransform(scan_ts);
      }
    }

    void pose_callback(const geometry_msgs::msg::Pose & msg) {
      pose_.x = msg.position.x;
      pose_.y = msg.position.y;
      geometry_msgs::msg::Quaternion q = msg.orientation;
      tf2::Quaternion quat(q.x, q.y, q.z, q.w);
      pose_.theta = tf2::impl::getYaw(quat);
    }

    void pose_rviz_callback(const geometry_msgs::msg::PoseWithCovarianceStamped & msg) {
      pose_callback(msg.pose.pose);
    }

    void drive_callback(const ackermann_msgs::msg::AckermannDriveStamped & msg) {
      set_speed(msg.drive.speed);
      set_steering_angle(msg.drive.steering_angle, this->get_clock()->now());
    }

    void joy_callback(const sensor_msgs::msg::Joy & msg) {
      set_speed(
          joy_max_speed_ * msg.axes[joy_speed_axis_]);
      set_steering_angle(
          max_steering_angle_ * msg.axes[joy_angle_axis_],
          this->get_clock()->now());
    }

    void set_speed(double speed) {
      speed_ = std::min(std::max(speed, -max_speed_), max_speed_);
    }

    void set_steering_angle(double steering_angle, builtin_interfaces::msg::Time timestamp) {
      steering_angle_ = std::min(std::max(steering_angle, -max_steering_angle_), max_steering_angle_);

      // Publish the steering angle
      tf2::Quaternion quat;
      quat.setEuler(0., 0., steering_angle);
      geometry_msgs::msg::TransformStamped ts;
      ts.transform.rotation.x = quat.x();
      ts.transform.rotation.y = quat.y();
      ts.transform.rotation.z = quat.z();
      ts.transform.rotation.w = quat.w();
      ts.header.stamp = timestamp;
      ts.header.frame_id = "front_left_hinge";
      ts.child_frame_id = "front_left_wheel";
      tf2_br_->sendTransform(ts);
      ts.header.frame_id = "front_right_hinge";
      ts.child_frame_id = "front_right_wheel";
      tf2_br_->sendTransform(ts);
    }

    void map_callback(const nav_msgs::msg::OccupancyGrid & msg) {
      // Fetch the map parameters
      size_t height = msg.info.height;
      size_t width = msg.info.width;
      double resolution = msg.info.resolution;
      // Convert the ROS origin to a pose
      Pose2D origin;
      origin.x = msg.info.origin.position.x;
      origin.y = msg.info.origin.position.y;
      geometry_msgs::msg::Quaternion q = msg.info.origin.orientation;
      tf2::Quaternion quat(q.x, q.y, q.z, q.w);
      origin.theta = tf2::impl::getYaw(quat);

      // Convert the map to probability values
      std::vector<double> map(msg.data.size());
      for (size_t i = 0; i < height * width; i++) {
        if (msg.data[i] > 100 or msg.data[i] < 0) {
          map[i] = 0.5; // Unknown
        } else {
          map[i] = msg.data[i]/100.;
        }
      }

      // Send the map to the scanner
      scan_simulator_.set_map(
          map,
          height,
          width,
          resolution,
          origin,
          map_free_threshold_);
      map_exists = true;
    }
};

} // namespace racecar_simulator

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<racecar_simulator::RacecarSimulator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
