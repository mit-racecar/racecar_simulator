#include <ros/ros.h>

#include <tf2/impl/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16MultiArray.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "racecar_simulator/pose_2d.hpp"
#include "racecar_simulator/ackermann_kinematics.hpp"
#include "racecar_simulator/scan_simulator_2d.hpp"

#include "racecar_simulator/Collision.h"
#include "racecar_simulator/car_state.hpp"
#include "racecar_simulator/car_params.hpp"
#include "racecar_simulator/ks_kinematics.hpp"
#include "racecar_simulator/st_kinematics.hpp"

using namespace racecar_simulator;

class RacecarSimulator {
private:
    // The transformation frames used
    std::string map_frame, base_frame, scan_frame;

    // The car state and parameters
    CarState state;
    double previous_seconds;
    double scan_distance_to_base_link;
    double max_speed, max_steering_angle;
    double max_accel, max_steering_vel;
    double accel, steer_angle_vel;
    CarParams params;
    double width;

    // A simulator of the laser
    ScanSimulator2D scan_simulator;
    double map_free_threshold;

    // Joystick parameters
    int joy_speed_axis, joy_angle_axis;
    double joy_max_speed;
    int joy_button_idx, assist_button_idx;

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

    // Publish and subscribe to collision data
    ros::Publisher coll_pub;
    ros::Subscriber coll_sub;

    // safety margin for collisions
    double thresh;

    // precompute cosines of scan angles
    std::vector<double> cosines;

    // scan parameters
    double scan_fov;
    double scan_ang_incr;

    // for joystick controls
    double joy_desired_steer;
    double joy_accel;

    // is the driver assist active
    bool dr_assist_on;
    // is joystick active
    bool joy_on;

    // set what is currently controlling the car
    std::vector<bool> mux_controller;
    int joy_mux_idx;
    int dr_assist_mux_idx;
    int mux_size;

    // for demo
    std::vector<bool> prev_mux;

    // for when driver assist gives control back
    int prev_controller_idx;

    // pi
    const double PI = 3.1415;

    // for collision check
    bool TTC = false;
    double ttc_threshold;

    // precompute distance from lidar to edge of car for each beam
    std::vector<double> car_distances;

    // publish mux controller
    ros::Publisher mux_pub;



public:

    RacecarSimulator() {
        // Initialize the node handle
        n = ros::NodeHandle("~");


        // Initialize car state and driving commands
        state = {.x=0, .y=0, .theta=0, .velocity=0, .steer_angle=0.0, .angular_velocity=0.0, .slip_angle=0.0, .st_dyn=false};
        accel = 0.0;
        steer_angle_vel = 0.0;
        previous_seconds = ros::Time::now().toSec();

        // Get the topic names
        std::string joy_topic, drive_topic, map_topic,
                scan_topic, pose_topic, pose_rviz_topic, odom_topic, coll_topic, mux_topic;
        n.getParam("joy_topic", joy_topic);
        n.getParam("drive_topic", drive_topic);
        n.getParam("map_topic", map_topic);
        n.getParam("scan_topic", scan_topic);
        n.getParam("pose_topic", pose_topic);
        n.getParam("odom_topic", odom_topic);
        n.getParam("pose_rviz_topic", pose_rviz_topic);
        n.getParam("collision_topic", coll_topic);
        n.getParam("mux_topic", mux_topic);

        // Get the transformation frame names
        n.getParam("map_frame", map_frame);
        n.getParam("base_frame", base_frame);
        n.getParam("scan_frame", scan_frame);

        // Fetch the car parameters
        int scan_beams;
        double update_pose_rate, scan_std_dev;
        n.getParam("wheelbase", params.wheelbase);
        n.getParam("update_pose_rate", update_pose_rate);
        n.getParam("scan_beams", scan_beams);
        n.getParam("scan_field_of_view", scan_fov);
        n.getParam("scan_std_dev", scan_std_dev);
        n.getParam("map_free_threshold", map_free_threshold);
        n.getParam("scan_distance_to_base_link", scan_distance_to_base_link);
        n.getParam("max_speed", max_speed);
        n.getParam("max_steering_angle", max_steering_angle);
        n.getParam("max_accel", max_accel);
        n.getParam("max_steering_vel", max_steering_vel);
        n.getParam("friction_coeff", params.friction_coeff);
        n.getParam("height_cg", params.h_cg);
        n.getParam("l_cg2rear", params.l_r);
        n.getParam("l_cg2front", params.l_f);
        n.getParam("C_S_front", params.cs_f);
        n.getParam("C_S_rear", params.cs_r);
        n.getParam("moment_inertia", params.I_z);
        n.getParam("mass", params.mass);
        n.getParam("width", width);

        // get mux idxs
        n.getParam("mux_size", mux_size);
        n.getParam("joy_mux_idx", joy_mux_idx);
        n.getParam("dr_assist_mux_idx", dr_assist_mux_idx);


        // Get joystick parameters
        n.getParam("joy", joy_on);
        n.getParam("joy_speed_axis", joy_speed_axis);
        n.getParam("joy_angle_axis", joy_angle_axis);
        n.getParam("joy_max_speed", joy_max_speed);
        n.getParam("joy_button_idx", joy_button_idx);
        n.getParam("assist_button_idx", assist_button_idx);

        // Determine if we should broadcast
        n.getParam("broadcast_transform", broadcast_transform);

        // Initialize a simulator of the laser scanner
        scan_simulator = ScanSimulator2D(
                    scan_beams,
                    scan_fov,
                    scan_std_dev);

        // Make a publisher for laser scan messages
        scan_pub = n.advertise<sensor_msgs::LaserScan>(scan_topic, 1);

        // Make a publisher for odometry messages
        odom_pub = n.advertise<nav_msgs::Odometry>(odom_topic, 1);

        // Start a timer to output the pose
        update_pose_timer = n.createTimer(ros::Duration(update_pose_rate), &RacecarSimulator::update_pose, this);

        // If the joystick is enabled
        if (joy_on)
            // Start a subscriber to listen to joystick commands
            joy_sub = n.subscribe(joy_topic, 1, &RacecarSimulator::joy_callback, this);

        // Start a subscriber to listen to drive commands
        drive_sub = n.subscribe(drive_topic, 1, &RacecarSimulator::drive_callback, this);

        // Start a subscriber to listen to new maps
        map_sub = n.subscribe(map_topic, 1, &RacecarSimulator::map_callback, this);

        // Start a subscriber to listen to pose messages
        pose_sub = n.subscribe(pose_topic, 1, &RacecarSimulator::pose_callback, this);
        pose_rviz_sub = n.subscribe(pose_rviz_topic, 1, &RacecarSimulator::pose_rviz_callback, this);

        // Make a publisher for collision messages
        coll_pub = n.advertise<racecar_simulator::Collision>(coll_topic, 10);

        // Start a subscriber to listen to collision messages
        coll_sub = n.subscribe(coll_topic, 1, &RacecarSimulator::coll_callback, this);

        // collision safety margin
        n.getParam("coll_threshold", thresh);

        scan_ang_incr = scan_simulator.get_angle_increment();

        joy_desired_steer = 0;
        joy_accel = 0;

        // initialize mux controller to start with joystick
        mux_controller.reserve(mux_size);
        prev_mux.reserve(mux_size);
        for (int i = 0; i < mux_size; i++) {
            mux_controller[i] = false;
            prev_mux[i] = false;
        }

        mux_controller[joy_mux_idx] = true;

        dr_assist_on = true;

        // default is joy (could change)
        prev_controller_idx = joy_mux_idx;

        n.getParam("ttc_threshold", ttc_threshold);

        // precompute cosines and distance from lidar to edge of car for each beam
        cosines.reserve(scan_beams);
        car_distances.reserve(scan_beams);
        double dist_to_sides = width / 2.0;
        double dist_to_front = params.wheelbase - scan_distance_to_base_link;
        double dist_to_back = scan_distance_to_base_link;

        for (int i = 0; i < scan_beams; i++) {
            double angle = -scan_fov/2.0 + i * scan_ang_incr;
            cosines[i] = std::cos(angle);

            if (angle > 0) {
                if (angle < PI / 2.0) {
                    // between 0 and pi/2
                    double to_side = dist_to_sides / std::sin(angle);
                    double to_front = dist_to_front / std::cos(angle);
                    car_distances[i] = std::min(to_side, to_front);
                } else {
                    // between pi/2 and pi
                    double to_side = dist_to_sides / std::cos(angle - PI / 2.0);
                    double to_back = dist_to_back / std::sin(angle - PI / 2.0);
                    car_distances[i] = std::min(to_side, to_back);
                }
            } else {
                if (angle > -PI / 2.0) {
                    // between 0 and -pi/2
                    double to_side = dist_to_sides / std::sin(-angle);
                    double to_front = dist_to_front / std::cos(-angle);
                    car_distances[i] = std::min(to_side, to_front);
                } else {
                    // between -pi/2 and -pi
                    double to_side = dist_to_sides / std::cos(-angle - PI / 2.0);
                    double to_back = dist_to_back / std::sin(-angle - PI / 2.0);
                    car_distances[i] = std::min(to_side, to_back);
                }
            }
        }

        mux_pub = n.advertise<std_msgs::Int16MultiArray>(mux_topic, 10);


    }

    void update_pose(const ros::TimerEvent&) {

        // if in collision
        if (TTC) {

        }

        // std_msgs::Int16MultiArray mux_msg;
        // mux_msg.data [mux_size];
        // std::cout << mux_msg.data[0] << std::endl;
        // for (int i = 0; i < mux_size; i++) {
        // 	std::cout << int(mux_controller[i]) << std::endl;
        // 	mux_msg.data[i] = int(mux_controller[i]);
        // }
        // mux_pub.publish(mux_msg);



        // set latest joystick commands
        if (mux_controller[joy_mux_idx]) {
            set_accel(joy_accel);
            set_steer_angle_vel(compute_steer_vel(joy_desired_steer));
        }


        // Prints the mux whenever it is changed
        bool changed = false;
        for (int i = 0; i < mux_size; i++) {
            changed = changed || (mux_controller[i] != prev_mux[i]);
        }
        if (changed) {
            for (int i = 0; i < mux_size; i++) {
                std::cout << mux_controller[i] << std::endl;
                prev_mux[i] = mux_controller[i];
            }
            std::cout << std::endl;
        }



        // Update the pose
        ros::Time timestamp = ros::Time::now();
        double current_seconds = timestamp.toSec();
        state = STKinematics::update(
                    state,
                    accel,
                    steer_angle_vel,
                    params,
                    current_seconds - previous_seconds);
        state.velocity = std::min(std::max(state.velocity, -max_speed), max_speed);
        state.steer_angle = std::min(std::max(state.steer_angle, -max_steering_angle), max_steering_angle);
        previous_seconds = current_seconds;


        // Convert the pose into a transformation
        geometry_msgs::Transform t;
        t.translation.x = state.x;
        t.translation.y = state.y;
        tf2::Quaternion quat;
        quat.setEuler(0., 0., state.theta);
        t.rotation.x = quat.x();
        t.rotation.y = quat.y();
        t.rotation.z = quat.z();
        t.rotation.w = quat.w();

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
        odom.pose.pose.position.x = state.x;
        odom.pose.pose.position.y = state.y;
        odom.pose.pose.orientation.x = quat.x();
        odom.pose.pose.orientation.y = quat.y();
        odom.pose.pose.orientation.z = quat.z();
        odom.pose.pose.orientation.w = quat.w();
        odom.twist.twist.linear.x = state.velocity;
        odom.twist.twist.angular.z =
                AckermannKinematics::angular_velocity(state.velocity, state.steer_angle, params.wheelbase);

        // Publish them
        if (broadcast_transform) br.sendTransform(ts);
        odom_pub.publish(odom);
        // Set the steering angle to make the wheels move
        // Publish the steering angle
        tf2::Quaternion quat_wheel;
        quat_wheel.setEuler(0., 0., state.steer_angle);
        geometry_msgs::TransformStamped ts_wheel;
        ts_wheel.transform.rotation.x = quat_wheel.x();
        ts_wheel.transform.rotation.y = quat_wheel.y();
        ts_wheel.transform.rotation.z = quat_wheel.z();
        ts_wheel.transform.rotation.w = quat_wheel.w();
        ts_wheel.header.stamp = timestamp;
        ts_wheel.header.frame_id = "front_left_hinge";
        ts_wheel.child_frame_id = "front_left_wheel";
        br.sendTransform(ts_wheel);
        ts_wheel.header.frame_id = "front_right_hinge";
        ts_wheel.child_frame_id = "front_right_wheel";
        br.sendTransform(ts_wheel);

        // If we have a map, perform a scan
        if (map_exists) {
            // Get the pose of the lidar, given the pose of base link
            // (base link is the center of the rear axle)
            Pose2D scan_pose;
            scan_pose.x = state.x + scan_distance_to_base_link * std::cos(state.theta);
            scan_pose.y = state.y + scan_distance_to_base_link * std::sin(state.theta);
            scan_pose.theta = state.theta;

            // Compute the scan from the lidar
            std::vector<double> scan = scan_simulator.scan(scan_pose);

            // Convert to float
            std::vector<float> scan_(scan.size());
            for (size_t i = 0; i < scan.size(); i++)
                scan_[i] = scan[i];

            // Publish to collision channel
            racecar_simulator::Collision coll_msg;
            coll_msg.in_danger = false;
            coll_msg.scan_idx = -1;
            coll_msg.distance = thresh * 2.0;

            // to reset TTC
            bool no_collision = true;
            for (size_t i = 0; i < scan_.size(); i++) {
                double angle = -scan_fov/2.0 + i * scan_ang_incr;
                // TTC stuff
                // calculate projected velocity
                if (state.velocity != 0) {
                    double proj_velocity = state.velocity * cosines[i];
                    double ttc = (scan_[i] - car_distances[i]) / proj_velocity;

                    if ((ttc < ttc_threshold) && (ttc >= 0.0)) { // if it's small
                        if (!TTC) {
                            first_ttc_actions();
                        }

                        no_collision = false;
                        TTC = true;
                        std::cout << "Collision!" << std::endl;
                        std::cout << "Angle: " << angle << std::endl;
                        std::cout << "TTC: " << ttc << std::endl << std::endl;
                    }

                }

                // collision message stuff
                if (scan_[i] < thresh) {
                    if (cosines[i] * state.velocity > 0) {
                        // find closeset point
                        if (scan_[i] < coll_msg.distance) {
                            coll_msg.distance = scan_[i];
                            coll_msg.wall_ang = angle;
                            coll_msg.in_danger = true;
                            coll_msg.scan_idx = i;
                        }
                    }
                }
            }

            // reset TTC
            if (no_collision)
                TTC = false;



            coll_msg.dr_assist_active = dr_assist_on && (!TTC);
            coll_msg.speed = state.velocity;
            coll_pub.publish(coll_msg);

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

    void first_ttc_actions() {
        joy_accel = 0.0;
        joy_desired_steer = 0.0;

        // completely stop vehicle
        state.velocity = 0.0;
        state.angular_velocity = 0.0;
        state.slip_angle = 0.0;
        state.steer_angle = 0.0;
        steer_angle_vel = 0.0;
        accel = 0.0;

        // kill mux (except joystick)
        for (int i = 0; i < mux_size; i++) {
            if (i == joy_mux_idx)
                continue;
            mux_controller[i] = false;
        }

        // turn on joystick if active
        mux_controller[joy_mux_idx] = joy_on;
    }


    void pose_callback(const geometry_msgs::Pose & msg) {
        state.x = msg.position.x;
        state.y = msg.position.y;
        geometry_msgs::Quaternion q = msg.orientation;
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        state.theta = tf2::impl::getYaw(quat);
    }

    void pose_rviz_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msg) {
        pose_callback(msg -> pose.pose);
    }

    void drive_callback(const ackermann_msgs::AckermannDriveStamped & msg) {
        set_accel(msg.drive.acceleration);
        set_steer_angle_vel(compute_steer_vel(msg.drive.steering_angle));
    }

    void joy_callback(const sensor_msgs::Joy & msg) {

        // joy controller:
        // get values to be set at top of update_pose
        if (mux_controller[joy_mux_idx] && joy_on) {
            joy_accel = joy_max_speed * msg.axes[joy_speed_axis];
            joy_desired_steer = max_steering_angle * msg.axes[joy_angle_axis];
        }


        // changing mux_controller:
        if (msg.buttons[assist_button_idx]) {
            if (dr_assist_on) {
                std::cout << "Driver Assist turned off" << std::endl;
                dr_assist_on = false;
                // switch control to previous controller if driver assist was on
                if (mux_controller[dr_assist_mux_idx]) {
                    mux_controller[dr_assist_mux_idx] = false;
                    mux_controller[prev_controller_idx] = true;
                }
            }
            else {
                std::cout << "Driver Assist turned on" << std::endl;
                dr_assist_on = true;
            }
        }
        else if (msg.buttons[joy_button_idx]) {
            if (joy_on) {
                std::cout << "Joystick turned off" << std::endl;
                joy_on = false;
                mux_controller[joy_mux_idx] = false;
                // previous controller on ?
                // no, either change it manually or
                // have some other idea when there's actually something else
            }
            else {
                std::cout << "Joystick turned on" << std::endl;
                joy_on = true;
                mux_controller[joy_mux_idx] = true;
            }
        }
    }

    void coll_callback(const racecar_simulator::Collision & msg) {
        // change mux controller so driver assist takes over
        if (dr_assist_on && msg.in_danger && !TTC) {
            // turn off prev controller and remember its index
            for (int i = 0; i < mux_size; i++) {
                if (i == dr_assist_mux_idx)
                    continue;
                if (mux_controller[i]) {
                    prev_controller_idx = i;
                    mux_controller[i] = false;
                    break;
                }
            }
            mux_controller[dr_assist_mux_idx] = true;
        }

        // if no longer in danger, give control back to previous controller
        if (!msg.in_danger && mux_controller[dr_assist_mux_idx]) {
            mux_controller[prev_controller_idx] = true;
            mux_controller[dr_assist_mux_idx] = false;
        }


        // assisting done in separate driver assist node
    }

    void set_accel(double accel_) {
        accel = std::min(std::max(accel_, -max_accel), max_accel);
    }

    void set_steer_angle_vel(double steer_angle_vel_) {
        steer_angle_vel = std::min(std::max(steer_angle_vel_, -max_steering_vel), max_steering_vel);
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

    double compute_steer_vel(double desired_angle) {
        // get difference between current and desired
        double dif = (desired_angle - state.steer_angle);

        // calculate and set velocity
        double steer_vel;
        if (std::abs(dif) > .02)  // if the difference is not trivial
            steer_vel = dif / std::abs(dif) * max_steering_vel;
        else
            steer_vel = 0;


        return steer_vel;
    }

};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "racecar_simulator");
    RacecarSimulator rs;
    ros::spin();
    return 0;
}
