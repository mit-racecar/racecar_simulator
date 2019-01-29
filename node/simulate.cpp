#include <ros/ros.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

// TODO DELETE
#include <geometry_msgs/PoseStamped.h>

#include "racecar_simulator/pose_2d.hpp"
#include "racecar_simulator/ackermann_kinematics.hpp"

using namespace racecar_simulator;

class RacecarSimulator {
  private:
    // The car state and parameters
    Pose2D pose;
    double wheelbase;
    double speed;
    double steering_angle;
    double previous_seconds;

    // A ROS node
    ros::NodeHandle n;

    // For publishing transformations
    tf2_ros::TransformBroadcaster br;

    // A timer to update the pose
    ros::Timer update_pose_timer;
    double update_pose_rate;

    // Listen for drive commands
    ros::Subscriber drive_sub;

    // TODO DELETE
    ros::Publisher pose_pub;

  public:

    RacecarSimulator() {
      // Initialize the pose and driving commands
      pose = {.x=0, .y=0, .theta=0};
      speed = 0;
      steering_angle = 0;
      previous_seconds = ros::Time::now().toSec();

      // Fetch the ROS parameters
      wheelbase = 0.34;
      update_pose_rate = 0.01;
      std::string drive_topic = "/vesc/low_level/ackermann_cmd_mux/output";

      // TODO DELETE
      pose_pub = n.advertise<geometry_msgs::PoseStamped>("/pose", 1);

      // Start a timer to output the pose
      update_pose_timer = n.createTimer(ros::Duration(update_pose_rate), &RacecarSimulator::update_pose, this);

      // Start a subscriber to listen to drive commands
      drive_sub = n.subscribe(drive_topic, 1, &RacecarSimulator::drive_callback, this);
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

      // TODO DELETE
      geometry_msgs::PoseStamped ps;
      ps.header.stamp = timestamp;
			ps.header.frame_id = "/base_link";
			pose_pub.publish(ps);
    }

    void drive_callback(const ackermann_msgs::AckermannDriveStamped & msg) {
      speed = msg.drive.speed;
      steering_angle = msg.drive.steering_angle;
    }
};

int main(int argc, char ** argv) {
  ros::init(argc, argv, "racecar_simulator");
  RacecarSimulator rs;
  ros::spin();
  return 0;
}
