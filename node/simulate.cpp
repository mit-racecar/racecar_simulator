#include <ros/ros.h>

#include <tf2/impl/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Joy.h>

#include "racecar_simulator/pose_2d.hpp"
#include "racecar_simulator/ackermann_kinematics.hpp"
#include "racecar_simulator/scan_simulator_2d.hpp"

using namespace racecar_simulator;

class RacecarSimulator {
  private:
    // The car state and parameters
    Pose2D pose;
    double wheelbase;
    double speed;
    double steering_angle;
    double previous_seconds;

    // A simulator of the laser
    ScanSimulator2D scan_simulator;
    double map_free_threshold;

    // Joystick parameters
    int joy_speed_axis, joy_angle_axis;
    double joy_max_speed, joy_max_angle;

    // A ROS node
    ros::NodeHandle n;

    // For publishing transformations
    tf2_ros::TransformBroadcaster br;

    // A timer to update the pose
    ros::Timer update_pose_timer;

    // Listen for drive and joystick commands
    ros::Subscriber drive_sub;
    ros::Subscriber joy_sub;

    // Listen for a map
    ros::Subscriber map_sub;
    bool map_exists = false;

    // Publish a scan
    ros::Publisher scan_pub;

  public:

    RacecarSimulator() {
      // Initialize the node handle
      n = ros::NodeHandle("~");

      // Initialize the pose and driving commands
      pose = {.x=0, .y=0, .theta=0};
      speed = 0;
      steering_angle = 0;
      previous_seconds = ros::Time::now().toSec();

      // Get the topic names
      std::string joy_topic, drive_topic, map_topic, scan_topic;
      n.getParam("joy_topic", joy_topic);
      n.getParam("drive_topic", drive_topic);
      n.getParam("map_topic", map_topic);
      n.getParam("scan_topic", scan_topic);

      // Fetch the car parameters
      int scan_beams;
      double update_pose_rate, scan_field_of_view, scan_std_dev;
      n.getParam("wheelbase", wheelbase);
      n.getParam("update_pose_rate", update_pose_rate);
      n.getParam("scan_beams", scan_beams);
      n.getParam("scan_field_of_view", scan_field_of_view);
      n.getParam("scan_std_dev", scan_std_dev);
      n.getParam("map_free_threshold", map_free_threshold);

      // Get joystick parameters
      bool joy;
      n.getParam("joy", joy);
      n.getParam("joy_speed_axis", joy_speed_axis);
      n.getParam("joy_angle_axis", joy_angle_axis);
      n.getParam("joy_max_speed", joy_max_speed);
      n.getParam("joy_max_angle", joy_max_angle);

      // Initialize a simulator of the laser scanner
      scan_simulator = ScanSimulator2D(
          scan_beams,
          scan_field_of_view,
          scan_std_dev);

      // Make a publisher for laser scan messages
      scan_pub = n.advertise<sensor_msgs::LaserScan>(scan_topic, 1);

      // Start a timer to output the pose
      update_pose_timer = n.createTimer(ros::Duration(update_pose_rate), &RacecarSimulator::update_pose, this);

      // If the joystick is enabled
      if (joy)
        // Start a subscriber to listen to joystick commands
        joy_sub = n.subscribe(joy_topic, 1, &RacecarSimulator::joy_callback, this);

      // Start a subscriber to listen to drive commands
      drive_sub = n.subscribe(drive_topic, 1, &RacecarSimulator::drive_callback, this);

      // Start a subscriber to listen to new maps
      map_sub = n.subscribe(map_topic, 1, &RacecarSimulator::map_callback, this);
    }

    void update_pose(const ros::TimerEvent&) {

      // Update the pose
      ros::Time timestamp = ros::Time::now();
      double current_seconds = timestamp.toSec();
      pose = AckermannKinematics::update(
          pose, 
          speed,
          steering_angle,
          wheelbase,
          current_seconds - previous_seconds);
      previous_seconds = current_seconds;

      // Convert the pose into a transformation
      geometry_msgs::Transform t;
      t.translation.x = pose.x;
      t.translation.y = pose.y;
      tf2::Quaternion quat;
      quat.setEuler(0., 0., pose.theta);
      t.rotation.x = quat.x();
      t.rotation.y = quat.y();
      t.rotation.z = quat.z();
      t.rotation.w = quat.w();

      // Add a header to the transformation
      geometry_msgs::TransformStamped ts;
      ts.transform = t;
      ts.header.stamp = timestamp;
      ts.header.frame_id = "/map";
      ts.child_frame_id = "/base_link";

      // Publish it
      br.sendTransform(ts);

      // If we have a map, perform a scan
      if (map_exists) {
        // TODO
        // Get the pose of the laser in the map frame
        // (Currently this is base link)
        std::vector<double> scan = scan_simulator.scan(pose);

        // Convert to float
        std::vector<float> scan_(scan.size());
        for (size_t i = 0; i < scan.size(); i++)
          scan_[i] = scan[i];

        // Publish the laser message
        sensor_msgs::LaserScan scan_msg;
        scan_msg.header.stamp = timestamp;
        scan_msg.header.frame_id = "/base_link";
        scan_msg.angle_min = -scan_simulator.get_field_of_view()/2.;
        scan_msg.angle_max =  scan_simulator.get_field_of_view()/2.;
        scan_msg.angle_increment = scan_simulator.get_angle_increment();
        scan_msg.range_max = 100;
        scan_msg.ranges = scan_;
        scan_msg.intensities = scan_;

        scan_pub.publish(scan_msg);
      }
    }

    void drive_callback(const ackermann_msgs::AckermannDriveStamped & msg) {
      speed = msg.drive.speed;
      steering_angle = msg.drive.steering_angle;
    }

    void joy_callback(const sensor_msgs::Joy & msg) {
      speed          = joy_max_speed * msg.axes[joy_speed_axis];
      steering_angle = joy_max_angle * msg.axes[joy_angle_axis];
    }

    void map_callback(const nav_msgs::OccupancyGrid & msg) {
      // Fetch the map parameters
      size_t height = msg.info.height;
      size_t width = msg.info.width;
      double resolution = msg.info.resolution;
      // Convert the ROS origin to a pose
      Pose2D origin;
      origin.x = msg.info.origin.position.x;
      origin.y = msg.info.origin.position.y;
      geometry_msgs::Quaternion q = msg.info.origin.orientation;
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
      scan_simulator.set_map(
          map,
          height,
          width,
          resolution,
          origin,
          map_free_threshold);
      map_exists = true;
    }
};

int main(int argc, char ** argv) {
  ros::init(argc, argv, "racecar_simulator");
  RacecarSimulator rs;
  ros::spin();
  return 0;
}
