#include <ros/ros.h>

// interactive marker
#include <interactive_markers/interactive_marker_server.h>

#include <tf2/impl/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "racecar_simulator/pose_2d.hpp"
#include "racecar_simulator/ackermann_kinematics.hpp"
#include "racecar_simulator/scan_simulator_2d.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <iostream>

using namespace racecar_simulator;

class RacecarSimulator {
  private:
    // The transformation frames used
    std::string map_frame, base_frame, scan_frame;

    // obstacle states (1D index) and parameters
    std::vector<int> added_obs;
    // listen for clicked point for adding obstacles
    ros::Subscriber obs_sub;
    int obstacle_size;

    // interactive markers' server
    interactive_markers::InteractiveMarkerServer im_server;

    // The car state and parameters
    Pose2D pose;
    double wheelbase;
    double speed;
    double steering_angle;
    double previous_seconds;
    double scan_distance_to_base_link;
    double max_speed, max_steering_angle;

    // A simulator of the laser
    ScanSimulator2D scan_simulator;
    double map_free_threshold;

    // Joystick parameters
    int joy_speed_axis, joy_angle_axis;
    double joy_max_speed;

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

    // Listen for updates to the pose
    ros::Subscriber pose_sub;
    ros::Subscriber pose_rviz_sub;

    // Publish a scan, odometry, and imu data
    bool broadcast_transform;
    ros::Publisher scan_pub;
    ros::Publisher odom_pub;
    // ros::Publisher img_pub;
    ros::Publisher collision_pub;
    ros::Publisher eroded_pub;
    // publisher for map with obstacles
    ros::Publisher map_pub;

    // keep an original map for obstacles
    nav_msgs::OccupancyGrid original_map;
    nav_msgs::OccupancyGrid current_map;

    // eroded map for collision
    std::vector<int> eroded_map;
    std::vector<int> original_eroded_map;
    int map_width, map_height, inflation_size;
    double map_resolution, origin_x, origin_y;
    nav_msgs::OccupancyGrid eroded_map_msg;

  public:

    RacecarSimulator(): im_server("racecar_sim") {
      // Initialize the node handle
      n = ros::NodeHandle("~");

      // Initialize the pose and driving commands
      pose = {.x=0, .y=0, .theta=0};
      speed = 0;
      steering_angle = 0;
      previous_seconds = ros::Time::now().toSec();

      // Get the topic names
      std::string joy_topic, drive_topic, map_topic, 
        scan_topic, pose_topic, pose_rviz_topic, odom_topic, collision_topic, eroded_topic;
      n.getParam("joy_topic", joy_topic);
      n.getParam("drive_topic", drive_topic);
      n.getParam("map_topic", map_topic);
      n.getParam("scan_topic", scan_topic);
      n.getParam("pose_topic", pose_topic);
      n.getParam("odom_topic", odom_topic);
      n.getParam("pose_rviz_topic", pose_rviz_topic);
      n.getParam("eroded_map_topic", eroded_topic);
      n.getParam("collision_topic", collision_topic);

      // Get the transformation frame names
      n.getParam("map_frame", map_frame);
      n.getParam("base_frame", base_frame);
      n.getParam("scan_frame", scan_frame);

      // Fetch the car parameters
      int scan_beams;
      double update_pose_rate, scan_field_of_view, scan_std_dev;
      n.getParam("wheelbase", wheelbase);
      n.getParam("update_pose_rate", update_pose_rate);
      n.getParam("scan_beams", scan_beams);
      n.getParam("scan_field_of_view", scan_field_of_view);
      n.getParam("scan_std_dev", scan_std_dev);
      n.getParam("map_free_threshold", map_free_threshold);
      n.getParam("scan_distance_to_base_link", scan_distance_to_base_link);
      n.getParam("max_speed", max_speed);
      n.getParam("max_steering_angle", max_steering_angle);

      // Get joystick parameters
      bool joy;
      n.getParam("joy", joy);
      n.getParam("joy_speed_axis", joy_speed_axis);
      n.getParam("joy_angle_axis", joy_angle_axis);
      n.getParam("joy_max_speed", joy_max_speed);

      // Determine if we should broadcast
      n.getParam("broadcast_transform", broadcast_transform);

      // Get map-based collision parameters
      n.getParam("inflation_size", inflation_size);

      // Get obstacle size parameter
      n.getParam("obstacle_size", obstacle_size);

      // Initialize a simulator of the laser scanner
      scan_simulator = ScanSimulator2D(
          scan_beams,
          scan_field_of_view,
          scan_std_dev);

      // Make a publisher for laser scan messages
      scan_pub = n.advertise<sensor_msgs::LaserScan>(scan_topic, 1);

      // Make a publisher for odometry messages
      odom_pub = n.advertise<nav_msgs::Odometry>(odom_topic, 1);

      // Make a publisher for collision boolean
      collision_pub = n.advertise<std_msgs::Bool>(collision_topic, 1);

      // Make a publisher for publishing eroded map
      eroded_pub = n.advertise<nav_msgs::OccupancyGrid>(eroded_topic, 1);

      // Make a publisher for publishing map with obstacles
      map_pub = n.advertise<nav_msgs::OccupancyGrid>("/map", 1);

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

      // Start a subscriber to listen to pose messages
      pose_sub = n.subscribe(pose_topic, 1, &RacecarSimulator::pose_callback, this);
      pose_rviz_sub = n.subscribe(pose_rviz_topic, 1, &RacecarSimulator::pose_rviz_callback, this);

      obs_sub = n.subscribe("/clicked_point", 1, &RacecarSimulator::obs_callback, this);

      // wait for one map message to get the map data array
      boost::shared_ptr<nav_msgs::OccupancyGrid const> map_ptr;
      nav_msgs::OccupancyGrid map_msg;
      map_ptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map");
      if (map_ptr != NULL) {
        map_msg = *map_ptr;
      }
      original_map = map_msg;
      current_map = map_msg;
      std::vector<int8_t> map_data_raw = map_msg.data;
      std::vector<int> map_data(map_data_raw.begin(), map_data_raw.end());
      
      map_width = map_msg.info.width;
      map_height = map_msg.info.height;
      origin_x = map_msg.info.origin.position.x;
      origin_y = map_msg.info.origin.position.y;
      map_resolution = map_msg.info.resolution;

      // create underlying map for collision
      eroded_map = std::vector<int>(map_data.begin(), map_data.end());
      for (int i=0; i<eroded_map.size(); i++) {
        if (map_data[i] != 0) {
          std::vector<int> current_rc = ind_2_rc(i);
          for (int infl_r=-inflation_size; infl_r<inflation_size; infl_r++) {
            for (int infl_c=-inflation_size; infl_c<inflation_size; infl_c++) {
              eroded_map[rc_2_ind(current_rc[0]+infl_r, current_rc[1]+infl_c)] = 100;
            }
          }
        }
      }
      original_eroded_map = eroded_map;
      eroded_map_msg.header = map_msg.header;
      eroded_map_msg.data = std::vector<int8_t>(eroded_map.begin(), eroded_map.end());
      eroded_pub.publish(eroded_map_msg);

      // create buttons for clearing obstacles, and spawning a new car
      visualization_msgs::InteractiveMarker clear_obs_button;
      clear_obs_button.header.frame_id = "/map";
      // clear_obs_button.pose.position.x = origin_x+(1/3)*map_width*map_resolution;
      // clear_obs_button.pose.position.y = origin_y+(1/3)*map_height*map_resolution;
      clear_obs_button.pose.position.x = -51;
      clear_obs_button.pose.position.y = -51;
      clear_obs_button.scale = 10;
      clear_obs_button.name = "clear_obstacles";
      clear_obs_button.description = "Clear Obstacles\n(Left Click)";
      visualization_msgs::InteractiveMarkerControl clear_obs_control;
      clear_obs_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
      clear_obs_control.name = "clear_obstacles_control";

      visualization_msgs::Marker clear_obs_marker;
      clear_obs_marker.type = visualization_msgs::Marker::CUBE;
      clear_obs_marker.scale.x = clear_obs_button.scale*0.45;
      clear_obs_marker.scale.y = clear_obs_button.scale*0.45;
      clear_obs_marker.scale.z = clear_obs_button.scale*0.45;
      clear_obs_marker.color.r = 0.5;
      clear_obs_marker.color.g = 0.5;
      clear_obs_marker.color.b = 0.5;
      clear_obs_marker.color.a = 1.0;

      clear_obs_control.markers.push_back(clear_obs_marker);
      clear_obs_control.always_visible = true;
      clear_obs_button.controls.push_back(clear_obs_control);

      im_server.insert(clear_obs_button);
      im_server.setCallback(clear_obs_button.name, boost::bind(&RacecarSimulator::clear_obstacles, this, _1));

      ROS_INFO("Simulator created.");
    }

    std::vector<int> ind_2_rc(int ind) {
      std::vector<int> rc;
      int row = floor(ind/map_width);
      int col = ind%map_width - 1;
      rc.push_back(row);
      rc.push_back(col);
      return rc;
    }

    int rc_2_ind(int r, int c) {
      return r*map_width + c;
    }

    std::vector<int> coord_2_cell_rc(double x, double y) {
      std::vector<int> rc;
      rc.push_back(static_cast<int>((y-origin_y)/map_resolution));
      rc.push_back(static_cast<int>((x-origin_x)/map_resolution));
      return rc;
    }

    bool check_collision(double x, double y) {
      std::vector<int> rc = coord_2_cell_rc(x, y);
      int val = eroded_map[rc_2_ind(rc[0], rc[1])];
      // ROS_INFO("current row: %d, current col: %d, current_val: %d", rc[0], rc[1], val);
      std_msgs::Bool bool_msg;
      bool_msg.data = (val!=0);
      collision_pub.publish(bool_msg);
      if (val!=0) ROS_INFO("collision");
      eroded_pub.publish(eroded_map_msg);
      return (val!=0);
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

      // check collision
      if (check_collision(pose.x, pose.y)) set_speed(0);

      // Add a header to the transformation
      geometry_msgs::TransformStamped ts;
      ts.transform = t;
      ts.header.stamp = timestamp;
      ts.header.frame_id = map_frame;
      ts.child_frame_id = base_frame;

      // Make an odom message as well
      nav_msgs::Odometry odom;
      odom.header.stamp = timestamp;
      odom.header.frame_id = map_frame;
      odom.child_frame_id = base_frame;
      odom.pose.pose.position.x = pose.x;
      odom.pose.pose.position.y = pose.y;
      odom.pose.pose.orientation.x = quat.x();
      odom.pose.pose.orientation.y = quat.y();
      odom.pose.pose.orientation.z = quat.z();
      odom.pose.pose.orientation.w = quat.w();
      odom.twist.twist.linear.x = speed;
      odom.twist.twist.angular.z = 
        AckermannKinematics::angular_velocity(speed, steering_angle, wheelbase);

      // Publish them
      if (broadcast_transform) br.sendTransform(ts);
      odom_pub.publish(odom);
      // Set the steering angle to make the wheels move
      set_steering_angle(steering_angle, timestamp);

      // If we have a map, perform a scan
      if (map_exists) {
        // Get the pose of the lidar, given the pose of base link
        // (base link is the center of the rear axle)
        Pose2D scan_pose;
        scan_pose.x = pose.x + scan_distance_to_base_link * std::cos(pose.theta);
        scan_pose.y = pose.y + scan_distance_to_base_link * std::sin(pose.theta);
        scan_pose.theta = pose.theta;

        // Compute the scan from the lidar
        std::vector<double> scan = scan_simulator.scan(scan_pose);

        // Convert to float
        std::vector<float> scan_(scan.size());
        for (size_t i = 0; i < scan.size(); i++)
          scan_[i] = scan[i];

        // Publish the laser message
        sensor_msgs::LaserScan scan_msg;
        scan_msg.header.stamp = timestamp;
        scan_msg.header.frame_id = scan_frame;
        scan_msg.angle_min = -scan_simulator.get_field_of_view()/2.;
        scan_msg.angle_max =  scan_simulator.get_field_of_view()/2.;
        scan_msg.angle_increment = scan_simulator.get_angle_increment();
        scan_msg.range_max = 100;
        scan_msg.ranges = scan_;
        scan_msg.intensities = scan_;

        scan_pub.publish(scan_msg);

        // Publish a transformation between base link and laser
        geometry_msgs::TransformStamped scan_ts;
        scan_ts.transform.translation.x = scan_distance_to_base_link;
        scan_ts.transform.rotation.w = 1;
        scan_ts.header.stamp = timestamp;
        scan_ts.header.frame_id = base_frame;
        scan_ts.child_frame_id = scan_frame;
        br.sendTransform(scan_ts);
      }
    }

    void obs_callback(const geometry_msgs::PointStamped &msg) {
      double x = msg.point.x;
      double y = msg.point.y;
      std::vector<int> rc = coord_2_cell_rc(x, y);
      int ind = rc_2_ind(rc[0], rc[1]);
      added_obs.push_back(ind);
      add_obs(ind);
    }

    void pose_callback(const geometry_msgs::PoseStamped & msg) {
      pose.x = msg.pose.position.x;
      pose.y = msg.pose.position.y;
      geometry_msgs::Quaternion q = msg.pose.orientation;
      tf2::Quaternion quat(q.x, q.y, q.z, q.w);
      pose.theta = tf2::impl::getYaw(quat);
    }

    void pose_rviz_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msg) {
      geometry_msgs::PoseStamped temp_pose;
      temp_pose.header = msg->header;
      temp_pose.pose = msg->pose.pose;
      pose_callback(temp_pose);
    }

    void drive_callback(const ackermann_msgs::AckermannDriveStamped & msg) {
      set_speed(msg.drive.speed);
      set_steering_angle(msg.drive.steering_angle, ros::Time::now());
    }

    void joy_callback(const sensor_msgs::Joy & msg) {
      set_speed(
          joy_max_speed * msg.axes[joy_speed_axis]);
      set_steering_angle(
          max_steering_angle * msg.axes[joy_angle_axis],
          ros::Time::now());
    }

    void add_obs(int ind) {
      std::vector<int> rc = ind_2_rc(ind);
      for (int i=-obstacle_size; i<obstacle_size; i++) {
        for (int j=-obstacle_size; j<obstacle_size; j++) {
          int current_r = rc[0]+i;
          int current_c = rc[1]+j;
          int current_ind = rc_2_ind(current_r, current_c);
          current_map.data[current_ind] = 100;
          eroded_map[current_ind] = 100;
        }
      }
      map_pub.publish(current_map);
    }

    void clear_obstacles(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
      current_map = original_map;
      map_pub.publish(current_map);
      eroded_map = original_eroded_map;
    }

    void set_speed(double speed_) {
      speed = std::min(std::max(speed_, -max_speed), max_speed);
    }

    void set_steering_angle(double steering_angle_, ros::Time timestamp) {
      steering_angle = std::min(std::max(steering_angle_, -max_steering_angle), max_steering_angle);

      // Publish the steering angle
      tf2::Quaternion quat;
      quat.setEuler(0., 0., steering_angle);
      geometry_msgs::TransformStamped ts;
      ts.transform.rotation.x = quat.x();
      ts.transform.rotation.y = quat.y();
      ts.transform.rotation.z = quat.z();
      ts.transform.rotation.w = quat.w();
      ts.header.stamp = timestamp;
      ts.header.frame_id = "front_left_hinge";
      ts.child_frame_id = "front_left_wheel";
      br.sendTransform(ts);
      ts.header.frame_id = "front_right_hinge";
      ts.child_frame_id = "front_right_wheel";
      br.sendTransform(ts);
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
