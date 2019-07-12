#include <ros/ros.h>
#include <ros/package.h>

// interactive marker
#include <interactive_markers/interactive_marker_server.h>

#include <tf2/impl/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32MultiArray.h>

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

#include "racecar_simulator/Collision.h"
#include "racecar_simulator/car_state.hpp"
#include "racecar_simulator/car_params.hpp"
#include "racecar_simulator/ks_kinematics.hpp"
#include "racecar_simulator/st_kinematics.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <iostream>
#include <math.h>
#include <fstream>


using namespace racecar_simulator;

class RacecarSimulator {
private:
    // The transformation frames used
    std::string map_frame, base_frame, scan_frame;
    std::string op_base_frame, op_scan_frame;

    // obstacle states (1D index) and parameters
    std::vector<int> added_obs;
    // listen for clicked point for adding obstacles
    ros::Subscriber obs_sub;
    int obstacle_size;

    // opponent state and parameters
    bool opponent_spawned = false;
    CarState opponent_pose;
    CarState previous_opponent_pose;
    double opponent_accel, opponent_steer_angle_vel;
    double car_collision_thresh;


    // interactive markers' server
    interactive_markers::InteractiveMarkerServer im_server;

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
    int joy_button_idx, copilot_button_idx, random_walk_button_idx;

    // A ROS node
    ros::NodeHandle n;

    // For publishing transformations
    tf2_ros::TransformBroadcaster br;
    tf2_ros::TransformBroadcaster op_br;

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

    // publisher for map with obstacles
    ros::Publisher map_pub;

    // publishers for opponent car
    ros::Publisher op_scan_pub;
    ros::Publisher op_odom_pub;

    // subscribers for opponent car
    ros::Subscriber op_pose_sub;
    ros::Subscriber op_pose_rviz_sub;
    ros::Subscriber op_drive_sub;

    // keep an original map for obstacles
    nav_msgs::OccupancyGrid original_map;
    nav_msgs::OccupancyGrid current_map;

    // for obstacle collision
    int map_width, map_height, inflation_size;
    double map_resolution, origin_x, origin_y;


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
    double joy_desired_velocity;

    // is the safety copilot active
    bool safety_copilot_on;
    // is joystick active
    bool joy_on;

    // set what is currently controlling the car
    std::vector<bool> mux_controller;
    int joy_mux_idx;
    int safety_copilot_mux_idx;
    int random_walker_mux_idx;
    int mux_size;

    // for demo
    std::vector<bool> prev_mux;

    // for when safety copilot gives control back
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

    // for collision logging
    std::ofstream collision_file;
    double beginning_seconds;
    int collision_count=0;


public:


    RacecarSimulator(): im_server("racecar_sim") {
        // Initialize the node handle
        n = ros::NodeHandle("~");

        // Initialize car state and driving commands
        state = {.x=0, .y=0, .theta=0, .velocity=0, .steer_angle=0.0, .angular_velocity=0.0, .slip_angle=0.0, .st_dyn=false};
        accel = 0.0;
        steer_angle_vel = 0.0;
        opponent_pose = {.x=0, .y=0, .theta=0, .velocity=0, .steer_angle=0.0, .angular_velocity=0.0, .slip_angle=0.0, .st_dyn=false};
        previous_opponent_pose = {.x=0, .y=0, .theta=0, .velocity=0, .steer_angle=0.0, .angular_velocity=0.0, .slip_angle=0.0, .st_dyn=false};
        opponent_accel = 0;
        opponent_steer_angle_vel = 0;
        previous_seconds = ros::Time::now().toSec();

        car_collision_thresh = 0.4;

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

        // Get opponent topic names
        std::string op_scan_topic, op_drive_topic, op_pose_topic, op_odom_topic, op_pose_rviz_topic;
        n.getParam("opponent_scan_topic", op_scan_topic);
        n.getParam("opponent_drive_topic", op_drive_topic);
        n.getParam("opponent_pose_topic", op_pose_topic);
        n.getParam("opponent_pose_rviz_topic", op_pose_rviz_topic);
        n.getParam("opponent_odom_topic", op_odom_topic);

        // Get the transformation frame names
        n.getParam("map_frame", map_frame);
        n.getParam("base_frame", base_frame);
        n.getParam("scan_frame", scan_frame);
        n.getParam("opponent_base_frame", op_base_frame);
        n.getParam("opponent_scan_frame", op_scan_frame);

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
        n.getParam("safety_copilot_mux_idx", safety_copilot_mux_idx);
        n.getParam("random_walker_mux_idx", random_walker_mux_idx);


        // Get joystick parameters
        n.getParam("joy", joy_on);
        n.getParam("joy_speed_axis", joy_speed_axis);
        n.getParam("joy_angle_axis", joy_angle_axis);
        n.getParam("joy_max_speed", joy_max_speed);
        n.getParam("joy_button_idx", joy_button_idx);
        n.getParam("copilot_button_idx", copilot_button_idx);
        n.getParam("random_walk_button_idx", random_walk_button_idx);

        // Determine if we should broadcast
        n.getParam("broadcast_transform", broadcast_transform);


        // Get obstacle size parameter
        n.getParam("obstacle_size", obstacle_size);

        // Initialize a simulator of the laser scanner
        scan_simulator = ScanSimulator2D(
                    scan_beams,
                    scan_fov,
                    scan_std_dev);

        // Make a publisher for laser scan messages
        scan_pub = n.advertise<sensor_msgs::LaserScan>(scan_topic, 1);
        
        // Make a publisher for opponents laser scan messages
        op_scan_pub = n.advertise<sensor_msgs::LaserScan>(op_scan_topic, 1);

        // Make a publisher for odometry messages
        odom_pub = n.advertise<nav_msgs::Odometry>(odom_topic, 1);

        op_odom_pub = n.advertise<nav_msgs::Odometry>(op_odom_topic, 1);

        // Make a publisher for publishing map with obstacles
        map_pub = n.advertise<nav_msgs::OccupancyGrid>("/map", 1);

        // Start a timer to output the pose
        update_pose_timer = n.createTimer(ros::Duration(update_pose_rate), &RacecarSimulator::update_pose, this);

        // If the joystick is enabled
        if (joy_on)
            // Start a subscriber to listen to joystick commands
            joy_sub = n.subscribe(joy_topic, 1, &RacecarSimulator::joy_callback, this);

        // Start a subscriber to listen to drive commands
        drive_sub = n.subscribe(drive_topic, 1, &RacecarSimulator::drive_callback, this);
        op_drive_sub = n.subscribe(op_drive_topic, 1, &RacecarSimulator::op_drive_callback, this);

        // Start a subscriber to listen to new maps
        map_sub = n.subscribe(map_topic, 1, &RacecarSimulator::map_callback, this);

        // Start a subscriber to listen to pose messages
        pose_sub = n.subscribe(pose_topic, 1, &RacecarSimulator::pose_callback, this);
        op_pose_sub = n.subscribe(op_pose_topic, 1, &RacecarSimulator::op_pose_callback, this);
        pose_rviz_sub = n.subscribe(pose_rviz_topic, 1, &RacecarSimulator::pose_rviz_callback, this);
        op_pose_rviz_sub = n.subscribe(op_pose_rviz_topic, 1, &RacecarSimulator::op_pose_rviz_callback, this);

        // obstacle subscriber
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

        // create button for clearing obstacles
        visualization_msgs::InteractiveMarker clear_obs_button;
        clear_obs_button.header.frame_id = "map";
        // clear_obs_button.pose.position.x = origin_x+(1/3)*map_width*map_resolution;
        // clear_obs_button.pose.position.y = origin_y+(1/3)*map_height*map_resolution;
        // TODO: find better positioning of buttons
        clear_obs_button.pose.position.x = 0;
        clear_obs_button.pose.position.y = -5;
        clear_obs_button.scale = 1;
        clear_obs_button.name = "clear_obstacles";
        clear_obs_button.description = "Clear Obstacles\n(Left Click)";
        visualization_msgs::InteractiveMarkerControl clear_obs_control;
        clear_obs_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
        clear_obs_control.name = "clear_obstacles_control";
        // make a box for the button
        visualization_msgs::Marker clear_obs_marker;
        clear_obs_marker.type = visualization_msgs::Marker::CUBE;
        clear_obs_marker.scale.x = clear_obs_button.scale*0.45;
        clear_obs_marker.scale.y = clear_obs_button.scale*0.65;
        clear_obs_marker.scale.z = clear_obs_button.scale*0.45;
        clear_obs_marker.color.r = 0.0;
        clear_obs_marker.color.g = 1.0;
        clear_obs_marker.color.b = 0.0;
        clear_obs_marker.color.a = 1.0;

        clear_obs_control.markers.push_back(clear_obs_marker);
        clear_obs_control.always_visible = true;
        clear_obs_button.controls.push_back(clear_obs_control);

        im_server.insert(clear_obs_button);
        im_server.setCallback(clear_obs_button.name, boost::bind(&RacecarSimulator::clear_obstacles, this, _1));


        // create buttons for spawning and despawning a new car
        visualization_msgs::InteractiveMarker spawn_car_button;
        spawn_car_button.header.frame_id = "map";
        spawn_car_button.pose.position.x = -2;
        spawn_car_button.pose.position.y = -5;
        spawn_car_button.scale = 1;
        spawn_car_button.name = "spawn_car";
        spawn_car_button.description = "Spawn Opponent Car\n(Left Click)";
        visualization_msgs::InteractiveMarkerControl spawn_car_control;
        spawn_car_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
        spawn_car_control.name = "spawn_car_control";

        visualization_msgs::InteractiveMarker despawn_car_button;
        despawn_car_button.header.frame_id = "map";
        despawn_car_button.pose.position.x = -2;
        despawn_car_button.pose.position.y = -8;
        despawn_car_button.scale = 1;
        despawn_car_button.name = "despawn_car";
        despawn_car_button.description = "Despawn Opponent Car\n(Left Click)";
        visualization_msgs::InteractiveMarkerControl despawn_car_control;
        despawn_car_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
        despawn_car_control.name = "despawn_car_control";

        // make a box for the button
        visualization_msgs::Marker spawn_car_marker;
        spawn_car_marker.type = visualization_msgs::Marker::CUBE;
        spawn_car_marker.scale.x = spawn_car_button.scale*0.45;
        spawn_car_marker.scale.y = spawn_car_button.scale*0.65;
        spawn_car_marker.scale.z = spawn_car_button.scale*0.45;
        spawn_car_marker.color.r = 0.0;
        spawn_car_marker.color.g = 0.5;
        spawn_car_marker.color.b = 0.5;
        spawn_car_marker.color.a = 1.0;

        spawn_car_control.markers.push_back(spawn_car_marker);
        spawn_car_control.always_visible = true;
        spawn_car_button.controls.push_back(spawn_car_control);

        im_server.insert(spawn_car_button);
        im_server.setCallback(spawn_car_button.name, boost::bind(&RacecarSimulator::spawn_car, this, _1));

        visualization_msgs::Marker despawn_car_marker;
        despawn_car_marker.type = visualization_msgs::Marker::CUBE;
        despawn_car_marker.scale.x = despawn_car_button.scale*0.45;
        despawn_car_marker.scale.y = despawn_car_button.scale*0.65;
        despawn_car_marker.scale.z = despawn_car_button.scale*0.45;
        despawn_car_marker.color.r = 0.5;
        despawn_car_marker.color.g = 0.1;
        despawn_car_marker.color.b = 0.1;
        despawn_car_marker.color.a = 1.0;

        despawn_car_control.markers.push_back(despawn_car_marker);
        despawn_car_control.always_visible = true;
        despawn_car_button.controls.push_back(despawn_car_control);

        im_server.insert(despawn_car_button);
        im_server.setCallback(despawn_car_button.name, boost::bind(&RacecarSimulator::despawn_car, this, _1));

        im_server.applyChanges();

        ROS_INFO("Simulator created.");





        // Make a publisher for collision messages
        coll_pub = n.advertise<racecar_simulator::Collision>(coll_topic, 10);

        // Start a subscriber to listen to collision messages
        coll_sub = n.subscribe(coll_topic, 1, &RacecarSimulator::coll_callback, this);

        // collision safety margin
        n.getParam("coll_threshold", thresh);

        scan_ang_incr = scan_simulator.get_angle_increment();

        joy_desired_steer = 0;
        joy_desired_velocity = 0;

        // initialize mux controller to start with joystick
        mux_controller.reserve(mux_size);
        prev_mux.reserve(mux_size);
        for (int i = 0; i < mux_size; i++) {
            mux_controller[i] = false;
            prev_mux[i] = false;
        }

        int starter = joy_mux_idx;

        mux_controller[starter] = true;

        safety_copilot_on = true;

        // default is joy (could change)
        prev_controller_idx = starter;

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

        
        mux_pub = n.advertise<std_msgs::Int32MultiArray>(mux_topic, 10);

        std::string filename;
        n.getParam("collision_file", filename);
        collision_file.open(ros::package::getPath("racecar_simulator") + "/logs/" + filename + ".txt");

        ros::Time timestamp = ros::Time::now();
        beginning_seconds = timestamp.toSec();

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

//        bool check_map_collision(double x, double y) {
//            std::vector<int> rc = coord_2_cell_rc(x, y);
//            int val = eroded_map[rc_2_ind(rc[0], rc[1])];
//            // ROS_INFO("current row: %d, current col: %d, current_val: %d", rc[0], rc[1], val);
//            std_msgs::Bool bool_msg;
//            bool_msg.data = (val!=0);
//            collision_pub.publish(bool_msg);
//            if (val!=0) ROS_INFO("collision");
//            eroded_pub.publish(eroded_map_msg);
//            return (val!=0);
//        }

//        bool check_opponent_collision() {
//            if (!opponent_spawned) {
//                return false;
//            } else {
//                return (std::sqrt((pose.x-opponent_pose.x)*(pose.x-opponent_pose.x)+(pose.y-opponent_pose.y)*(pose.y-opponent_pose.y)) < car_collision_thresh);
//            }
//        }




        void update_pose(const ros::TimerEvent&) {


            /// KEEP in sim, but compute accel and steer angle vel every update
            // set latest joystick commands
            if (mux_controller[joy_mux_idx]) {
                set_accel(compute_accel(joy_desired_velocity));
                set_steer_angle_vel(compute_steer_vel(joy_desired_steer));
            }


            /// MOVE TO: mux
            // Prints the mux whenever it is changed
            bool changed = false;
            // checks if nothing is on
            bool anything_on = false;
            for (int i = 0; i < mux_size; i++) {
                changed = changed || (mux_controller[i] != prev_mux[i]);
                anything_on = anything_on || mux_controller[i];
            }
            if (changed) {
                std::cout << "MUX: " << std::endl;
                for (int i = 0; i < mux_size; i++) {
                    std::cout << mux_controller[i] << std::endl;
                    prev_mux[i] = mux_controller[i];
                }
                std::cout << std::endl;
            }
            if (!anything_on) {
                // if no mux channel is active, halt the car
                set_accel(compute_accel(0.0));
                set_steer_angle_vel(compute_steer_vel(0.0));
            }


            /// KEEP in sim
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

            previous_opponent_pose = opponent_pose;
            opponent_pose = STKinematics::update(
                        opponent_pose,
                        opponent_accel,
                        opponent_steer_angle_vel,
                        params,
                        current_seconds - previous_seconds);
            opponent_pose.velocity = std::min(std::max(opponent_pose.velocity, -max_speed), max_speed);
            opponent_pose.steer_angle = std::min(std::max(opponent_pose.steer_angle, -max_steering_angle), max_steering_angle);

            previous_seconds = current_seconds;


            /// KEEP in sim (helper function)
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


            // opponent pose into transformation
            geometry_msgs::Transform op_t;
            op_t.translation.x = opponent_pose.x;
            op_t.translation.y = opponent_pose.y;
            tf2::Quaternion op_quat;
            op_quat.setEuler(0., 0., opponent_pose.theta);
            op_t.rotation.x = op_quat.x();
            op_t.rotation.y = op_quat.y();
            op_t.rotation.z = op_quat.z();
            op_t.rotation.w = op_quat.w();

            // Add a header to the transformation
            geometry_msgs::TransformStamped ts;
            ts.transform = t;
            ts.header.stamp = timestamp;
            ts.header.frame_id = map_frame;
            ts.child_frame_id = base_frame;

            // opponent header
            geometry_msgs::TransformStamped op_ts;
            op_ts.transform = op_t;
            op_ts.header.stamp = timestamp;
            op_ts.header.frame_id = map_frame;
            op_ts.child_frame_id = op_base_frame;

            /// KEEP in sim (helper function)
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
            odom.twist.twist.angular.z = state.angular_velocity;
            

            // opponent odom message
            nav_msgs::Odometry op_odom;
            op_odom.header.stamp = timestamp;
            op_odom.header.frame_id = map_frame;
            op_odom.child_frame_id = op_base_frame;
            op_odom.pose.pose.position.x = opponent_pose.x;
            op_odom.pose.pose.position.y = opponent_pose.y;
            op_odom.pose.pose.orientation.x = op_quat.x();
            op_odom.pose.pose.orientation.y = op_quat.y();
            op_odom.pose.pose.orientation.z = op_quat.z();
            op_odom.pose.pose.orientation.w = op_quat.w();
            op_odom.twist.twist.linear.x = opponent_pose.velocity;
            op_odom.twist.twist.angular.z = state.angular_velocity;

            /// KEEP in sim (helper function)
            // Publish them
            if (broadcast_transform) {
                br.sendTransform(ts);
                op_br.sendTransform(op_ts);
            }
            odom_pub.publish(odom);
            op_odom_pub.publish(op_odom);

            /// KEEP in sim (helper function)
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

            // opponent steering angle so it moves
            tf2::Quaternion quat_op;
            quat_op.setEuler(0., 0., opponent_pose.steer_angle);
            geometry_msgs::TransformStamped ts_op;
            ts_op.transform.rotation.x = quat_op.x();
            ts_op.transform.rotation.y = quat_op.y();
            ts_op.transform.rotation.z = quat_op.z();
            ts_op.transform.rotation.w = quat_op.w();
            ts_op.header.stamp = timestamp;
            ts_op.header.frame_id = "op_front_left_hinge";
            ts_op.child_frame_id = "op_front_left_wheel";
            op_br.sendTransform(ts_op);
            ts_op.header.frame_id = "op_front_right_hinge";
            ts_op.child_frame_id = "op_front_right_wheel";
            op_br.sendTransform(ts_op);



            /// KEEP in sim
            // If we have a map, perform a scan
            if (map_exists) {
                // Get the pose of the lidar, given the pose of base link
                // (base link is the center of the rear axle)
                Pose2D scan_pose;
                scan_pose.x = state.x + scan_distance_to_base_link * std::cos(state.theta);
                scan_pose.y = state.y + scan_distance_to_base_link * std::sin(state.theta);
                scan_pose.theta = state.theta;

                Pose2D op_scan_pose;
                op_scan_pose.x = opponent_pose.x + scan_distance_to_base_link * std::cos(opponent_pose.theta);
                op_scan_pose.y = opponent_pose.y + scan_distance_to_base_link * std::sin(opponent_pose.theta);
                op_scan_pose.theta = opponent_pose.theta;

                // Compute the scan from the lidar
                std::vector<double> scan = scan_simulator.scan(scan_pose);
                std::vector<double> op_scan = scan_simulator.scan(op_scan_pose);


                // Convert to float
                std::vector<float> scan_(scan.size());
                for (size_t i = 0; i < scan.size(); i++)
                    scan_[i] = scan[i];

                // add scan if opponent is spawned
                if (opponent_spawned) {
                    double diff_x = opponent_pose.x - scan_pose.x;
                    double diff_y = opponent_pose.y - scan_pose.y;
                    double diff_dist = std::sqrt(diff_x*diff_x + diff_y*diff_y);
                    // TODO: change angle range so that it depends on the distance to the opponent
                    //double angle_infl = 0.04;
                    double angle_infl = 1./(10*diff_dist);
                    double angle = -scan_pose.theta + std::atan2(diff_y, diff_x);
                    if (angle < scan_simulator.get_field_of_view()/2. || angle > -scan_simulator.get_field_of_view()/2.) {
                        double angle_inc = scan_simulator.get_angle_increment();
                        double angle_min = -scan_simulator.get_field_of_view()/2.;
                        int start_ind = static_cast<int>((angle-angle_infl-angle_min)/angle_inc);
                        int end_ind = static_cast<int>((angle+angle_infl-angle_min)/angle_inc);
                        for (int i=start_ind; i<=end_ind; i++) {
                            if (scan_[i] > diff_dist) scan_[i] = diff_dist;
                        }
                    }
                }

                std::vector<float> op_scan_(op_scan.size());
                for (size_t i = 0; i < op_scan.size(); i++)
                    op_scan_[i] = op_scan[i];


                /// KEEP in sim (but update all this stuff at some point) (Make TTC message)
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

                            collision_count++;
                            no_collision = false;
                            TTC = true;
                            ROS_INFO("Collision detected");
                            std::cout << "Angle: " << angle << std::endl;
                            std::cout << "TTC: " << ttc << std::endl << std::endl;
                            collision_file << "Collision #" << collision_count << " detected:\n";
                            collision_file << "TTC: " << ttc << " seconds\n";
                            collision_file << "Angle to obstacle: " << angle << " radians\n";
                            collision_file << "Time since start of sim: " << (current_seconds - beginning_seconds) << " seconds\n";
                            collision_file << "\n";
                            
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

                coll_msg.speed = state.velocity;
                coll_pub.publish(coll_msg);

                /// KEEP in sim
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

                sensor_msgs::LaserScan op_scan_msg;
                op_scan_msg.header.stamp = timestamp;
                op_scan_msg.header.frame_id = op_scan_frame;
                op_scan_msg.angle_min = -scan_simulator.get_field_of_view()/2.;
                op_scan_msg.angle_max =  scan_simulator.get_field_of_view()/2.;
                op_scan_msg.angle_increment = scan_simulator.get_angle_increment();
                op_scan_msg.range_max = 100;
                op_scan_msg.ranges = op_scan_;
                op_scan_msg.intensities = op_scan_;

                op_scan_pub.publish(op_scan_msg);


                /// KEEP in sim (helper function)
                // Publish a transformation between base link and laser
                geometry_msgs::TransformStamped scan_ts;
                scan_ts.transform.translation.x = scan_distance_to_base_link;
                scan_ts.transform.rotation.w = 1;
                scan_ts.header.stamp = timestamp;
                scan_ts.header.frame_id = base_frame;
                scan_ts.child_frame_id = scan_frame;
                br.sendTransform(scan_ts);

                geometry_msgs::TransformStamped op_scan_ts;
                op_scan_ts.transform.translation.x = scan_distance_to_base_link;
                op_scan_ts.transform.rotation.w = 1;
                op_scan_ts.header.stamp = timestamp;
                op_scan_ts.header.frame_id = op_base_frame;
                op_scan_ts.child_frame_id = op_scan_frame;
                op_br.sendTransform(op_scan_ts);
            }
        } //end of update_pose

        /// KEEP in sim
        void first_ttc_actions() {
            joy_desired_velocity = 0.0;
            joy_desired_steer = 0.0;

            // completely stop vehicle
            state.velocity = 0.0;
            state.angular_velocity = 0.0;
            state.slip_angle = 0.0;
            state.steer_angle = 0.0;
            steer_angle_vel = 0.0;
            accel = 0.0;

            // kill mux
            for (int i = 0; i < mux_size; i++) {
                mux_controller[i] = false;
            }

            // turn on joystick if active
            mux_controller[joy_mux_idx] = joy_on;

            // turn off safety copilot
            safety_copilot_on = false;
        }

        /// KEEP in sim
        void project_opponent() {
            // clear projection from last frame
            double prev_x = previous_opponent_pose.x;
            double prev_y = previous_opponent_pose.y;
            std::vector<int> prev_rc = coord_2_cell_rc(prev_x, prev_y);
            int prev_ind = rc_2_ind(prev_rc[0], prev_rc[1]);
            clear_obs(prev_ind);
            double x = opponent_pose.x;
            double y = opponent_pose.y;
            std::vector<int> rc = coord_2_cell_rc(x,y);
            int ind = rc_2_ind(rc[0], rc[1]);
            add_obs(ind);
        }

        /// KEEP in sim
        void obs_callback(const geometry_msgs::PointStamped &msg) {
            double x = msg.point.x;
            double y = msg.point.y;
            std::vector<int> rc = coord_2_cell_rc(x, y);
            int ind = rc_2_ind(rc[0], rc[1]);
            added_obs.push_back(ind);
            add_obs(ind);
        }


        /// KEEP in sim
        void pose_callback(const geometry_msgs::PoseStamped & msg) {
            state.x = msg.pose.position.x;
            state.y = msg.pose.position.y;
            geometry_msgs::Quaternion q = msg.pose.orientation;
            tf2::Quaternion quat(q.x, q.y, q.z, q.w);
            state.theta = tf2::impl::getYaw(quat);
        }


        /// KEEP in sim
        void op_pose_callback(const geometry_msgs::PoseStamped &msg) {
            opponent_pose.x = msg.pose.position.x;
            opponent_pose.y = msg.pose.position.y;
            geometry_msgs::Quaternion q = msg.pose.orientation;
            tf2::Quaternion quat(q.x, q.y, q.z, q.w);
            opponent_pose.theta = tf2::impl::getYaw(quat);
        }

        /// KEEP in sim
        void pose_rviz_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msg) {

            geometry_msgs::PoseStamped temp_pose;
            temp_pose.header = msg->header;
            temp_pose.pose = msg->pose.pose;
            pose_callback(temp_pose);
        }

        /// KEEP in sim
        void op_pose_rviz_callback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
            op_pose_callback(*msg);

        }

        /// KEEP in sim
        void drive_callback(const ackermann_msgs::AckermannDriveStamped & msg) {
            set_accel(compute_accel(msg.drive.speed));
            set_steer_angle_vel(compute_steer_vel(msg.drive.steering_angle));
        }

        /// KEEP in sim
        void op_drive_callback(const ackermann_msgs::AckermannDriveStamped &msg) {
            set_op_accel(msg.drive.speed);
            set_op_steer_angle_vel(compute_steer_vel(msg.drive.steering_angle));
        }

        /// MOVE TO: behavior control node
        void joy_callback(const sensor_msgs::Joy & msg) {

            // joy controller:
            // get values to be set at top of update_pose
            if (mux_controller[joy_mux_idx] && joy_on) {
                joy_desired_velocity = max_speed * msg.axes[joy_speed_axis];
                joy_desired_steer = max_steering_angle * msg.axes[joy_angle_axis];
            }


            // changing mux_controller:
            if (msg.buttons[copilot_button_idx]) {
                if (safety_copilot_on) {
                    std::cout << "Safety Copilot turned off" << std::endl << std::endl;
                    safety_copilot_on = false;
                    // switch control to previous controller if safety copilot was on
                    if (mux_controller[safety_copilot_mux_idx]) {
                        mux_controller[safety_copilot_mux_idx] = false;
                        mux_controller[prev_controller_idx] = true;
                    }
                }
                else {
                    std::cout << "Safety Copilot turned on" << std::endl << std::endl;
                    safety_copilot_on = true;
                }
            }
            else if (msg.buttons[joy_button_idx]) {
                if (joy_on) {
                    std::cout << "Joystick turned off" << std::endl << std::endl;
                    joy_on = false;
                    mux_controller[joy_mux_idx] = false;
                    // previous controller on ?
                    // no, either change it manually or
                    // have some other idea when there's actually something else
                }
                else {
                    std::cout << "Joystick turned on" << std::endl << std::endl;
                    joy_on = true;
                    // turn everything off
                    for (int i = 0; i < mux_size; i++) {
                        mux_controller[i] = false;
                    }
                    // turn on joystick
                    mux_controller[joy_mux_idx] = true;
                }
            }
            else if (msg.buttons[random_walk_button_idx]) {
                if (mux_controller[random_walker_mux_idx]) {
                    std::cout << "Random Walk turned off" << std::endl << std::endl;
                    // turn off random walker
                    mux_controller[random_walker_mux_idx] = false;
                    // turn on joystick if it's active
                    mux_controller[joy_mux_idx] = joy_on;
                }
                else {
                    std::cout << "Random Walk turned on" << std::endl << std::endl;
                    // turn everything off
                    for (int i = 0; i < mux_size; i++) {
                        mux_controller[i] = false;
                    }
                    // turn on random walker
                    mux_controller[random_walker_mux_idx] = true; 
                }
            }
        }

        /// MOVE TO: behavioral control node
        void coll_callback(const racecar_simulator::Collision & msg) {
            // change mux controller so safety copilot takes over
            if (safety_copilot_on && msg.in_danger && !TTC) {
                // turn off prev controller and remember its index
                for (int i = 0; i < mux_size; i++) {
                    if (i == safety_copilot_mux_idx)
                        continue;
                    if (mux_controller[i]) {
                        prev_controller_idx = i;
                        mux_controller[i] = false;
                        break;
                    }
                }
                mux_controller[safety_copilot_mux_idx] = true;
            }

            // if no longer in danger, give control back to previous controller
            if (!msg.in_danger && mux_controller[safety_copilot_mux_idx]) {
                mux_controller[prev_controller_idx] = true;
                mux_controller[safety_copilot_mux_idx] = false;
            }

            // computation done in separate safety copilot node
        }

        /// KEEP in sim
        void set_accel(double accel_) {
            accel = std::min(std::max(accel_, -max_accel), max_accel);
        }

        /// KEEP in sim
        void set_steer_angle_vel(double steer_angle_vel_) {
            steer_angle_vel = std::min(std::max(steer_angle_vel_, -max_steering_vel), max_steering_vel);
        }

        /// KEEP in sim
        void add_obs(int ind) {
            std::vector<int> rc = ind_2_rc(ind);
            for (int i=-obstacle_size; i<obstacle_size; i++) {
                for (int j=-obstacle_size; j<obstacle_size; j++) {
                    int current_r = rc[0]+i;
                    int current_c = rc[1]+j;
                    int current_ind = rc_2_ind(current_r, current_c);
                    current_map.data[current_ind] = 100;
                }
            }
            map_pub.publish(current_map);
        }

        /// KEEP in sim
        void clear_obs(int ind) {
            std::vector<int> rc = ind_2_rc(ind);
            for (int i=-obstacle_size; i<obstacle_size; i++) {
                for (int j=-obstacle_size; j<obstacle_size; j++) {
                    int current_r = rc[0]+i;
                    int current_c = rc[1]+j;
                    int current_ind = rc_2_ind(current_r, current_c);
                    current_map.data[current_ind] = 0;

                }
            }
            map_pub.publish(current_map);
        }

        /// KEEP in sim
        // button callbacks
        void clear_obstacles(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
            bool clear_obs_clicked = false;
            if (feedback->event_type == 3) {
                clear_obs_clicked = true;
            }
            if (clear_obs_clicked) {
                ROS_INFO("Clearing obstacles.");
                current_map = original_map;
                map_pub.publish(current_map);

                clear_obs_clicked = false;
            }
        }

        /// KEEP in sim
        void spawn_car(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
            bool spawn_car_clicked = false;
            if (feedback->event_type == 3) {
                spawn_car_clicked = true;
            }
            if (spawn_car_clicked && !opponent_spawned) {
                ROS_INFO("Spawning opponent.");
                spawn_car_clicked = false;
                opponent_spawned = true;
            }
        }

        /// KEEP in sim
        void despawn_car(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
            bool despawn_car_clicked = false;
            if (feedback->event_type == 3) {
                despawn_car_clicked = true;
            }
            if (despawn_car_clicked && opponent_spawned) {
                ROS_INFO("Despawning opponent.");
                despawn_car_clicked = false;
                opponent_spawned = false;
            }
        }

        /// KEEP in sim
        void set_op_accel(double accel_) {
            opponent_accel = std::min(std::max(accel_, -max_accel), max_accel);
        }

        /// KEEP in sim
        void set_op_steer_angle_vel(double steer_angle_vel_) {
            opponent_steer_angle_vel = std::min(std::max(steer_angle_vel_, -max_steering_vel), max_steering_vel);
        }

        /// KEEP in sim
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

        /// KEEP in sim
        double compute_steer_vel(double desired_angle) {
            // get difference between current and desired
            double dif = (desired_angle - state.steer_angle);

            // calculate velocity
            double steer_vel;
            if (std::abs(dif) > .02)  // if the difference is not trivial
                steer_vel = dif / std::abs(dif) * max_steering_vel;
            else
                steer_vel = 0;


            return steer_vel;
        }

        /// KEEP in sim
        double compute_accel(double desired_velocity) {
            // get difference between current and desired
            double dif = (desired_velocity - state.velocity);

            double kp = 2.0 * max_accel / max_speed;

            // calculate acceleration
            double acceleration = kp * dif;

            return acceleration;
        }

    };

    int main(int argc, char ** argv) {
        ros::init(argc, argv, "racecar_simulator");
        RacecarSimulator rs;
        ros::spin();
        return 0;
    }
