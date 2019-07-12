#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>

#include <fstream>

#include "racecar_simulator/car_state.hpp"
#include "racecar_simulator/precompute.hpp"

using namespace racecar_simulator;

class BehaviorController {
private:
	// A ROS node
    ros::NodeHandle n;

    // Listen for messages from joystick, keyboard, laser scan, odometry, and IMU
    ros::Subscriber joy_sub;
    ros::Subscriber key_sub;
    ros::Subscriber laser_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber imu_sub;

    // Publisher for mux controller
    ros::Publisher mux_pub;

    // Mux indices
    int joy_mux_idx;
    int key_mux_idx;
    int safety_copilot_mux_idx;
    int random_walker_mux_idx;

    // Mux controller array
    std::vector<bool> mux_controller;
    int mux_size;

    // Button indices
    int joy_button_idx;
    int key_button_idx;
    int random_walk_button_idx;
    int safety_copilot_button_idx;

    // Key indices
    std::string joy_key_char;
    std::string keyboard_key_char;
    std::string copilot_key_char;
    std::string random_walk_key_char;

    // Is safety copilot on?
    bool safety_copilot_on;

    // Previous controller before safety copilot took control
    int prev_controller_idx;

    // To roughly keep track of vehicle state
    racecar_simulator::CarState state;

    // precompute distance from lidar to edge of car for each beam
	std::vector<double> car_distances;

	// precompute cosines of scan angles
	std::vector<double> cosines;

	// for collision detection
	double ttc_threshold;
	bool in_collision=false;

	// for collision logging
	std::ofstream collision_file;
	double beginning_seconds;
	int collision_count=0;



public:
	BehaviorController() {
		// Initialize the node handle
		n = ros::NodeHandle("~");

		// get topic names
		std::string scan_topic, odom_topic, imu_topic, joy_topic, keyboard_topic, mux_topic;
		n.getParam("scan_topic", scan_topic);
		n.getParam("odom_topic", odom_topic);
		n.getParam("imu_topic", imu_topic);
		n.getParam("joy_topic", joy_topic);
		n.getParam("mux_topic", mux_topic);
		n.getParam("keyboard_topic", keyboard_topic);

		// Make a publisher for mux messages
        mux_pub = n.advertise<std_msgs::Int32MultiArray>(mux_topic, 10);

        // Start subscribers to listen to laser scan, joy, IMU, and odom messages
        laser_sub = n.subscribe(scan_topic, 1, &BehaviorController::laser_callback, this);
        joy_sub = n.subscribe(joy_topic, 1, &BehaviorController::joy_callback, this);
        imu_sub = n.subscribe(imu_topic, 1, &BehaviorController::imu_callback, this);
        odom_sub = n.subscribe(odom_topic, 1, &BehaviorController::odom_callback, this);
        key_sub = n.subscribe(keyboard_topic, 1, &BehaviorController::key_callback, this);

        // Get mux indices
        n.getParam("joy_mux_idx", joy_mux_idx);
        n.getParam("key_mux_idx", key_mux_idx);
        n.getParam("safety_copilot_mux_idx", safety_copilot_mux_idx);
        n.getParam("random_walker_mux_idx", random_walker_mux_idx);

        // Get button indices
        n.getParam("joy_button_idx", joy_button_idx);
        n.getParam("key_button_idx", key_button_idx);
        n.getParam("copilot_button_idx", safety_copilot_button_idx);
        n.getParam("random_walk_button_idx", random_walk_button_idx);

        // Get key indices
        n.getParam("joy_key_char", joy_key_char);
        n.getParam("keyboard_key_char", keyboard_key_char);
        n.getParam("copilot_key_char", copilot_key_char);
        n.getParam("random_walk_key_char", random_walk_key_char);

        // Initialize the mux controller 
        n.getParam("mux_size", mux_size);
        mux_controller.reserve(mux_size);
        for (int i = 0; i < mux_size; i++) {
            mux_controller[i] = false;
        }

        // Start with safety copilot off and previous controller to joystick
        safety_copilot_on = false;
        prev_controller_idx = joy_mux_idx;

        // Initialize state
        state = {.x=0.0, .y=0.0, .theta=0.0, .velocity=0.0, .steer_angle=0.0, .angular_velocity=0.0, .slip_angle=0.0, .st_dyn=false};

        // Get params for precomputation and collision detection
        int scan_beams;
        double scan_fov, scan_ang_incr, wheelbase, width, scan_distance_to_base_link;
        n.getParam("ttc_threshold", ttc_threshold);
        n.getParam("scan_beams", scan_beams);
        n.getParam("scan_distance_to_base_link", scan_distance_to_base_link);
        n.getParam("width", width);
        n.getParam("wheelbase", wheelbase);
        n.getParam("scan_field_of_view", scan_fov);
        scan_ang_incr = scan_fov / scan_beams;

        // Precompute cosine and distance to car at each angle of the laser scan
        cosines = Precompute::get_cosines(scan_beams, -scan_fov/2.0, scan_ang_incr);
		car_distances = Precompute::get_car_distances(scan_beams, wheelbase, width, 
				scan_distance_to_base_link, -scan_fov/2.0, scan_ang_incr);

		// Create collision file to be written to
		std::string filename;
		n.getParam("collision_file", filename);
		collision_file.open(ros::package::getPath("racecar_simulator") + "/logs/" + filename + ".txt");
		beginning_seconds = ros::Time::now().toSec();

	}

	/// ---------------------- GENERAL HELPER FUNCTIONS ----------------------

	void publish_mux() {
		// make mux message
		std_msgs::Int32MultiArray mux_msg;
		mux_msg.data.clear();
        // push data onto message
		for (int i = 0; i < mux_size; i++) {
			mux_msg.data.push_back(int(mux_controller[i]));
		}

        // publish mux message
		mux_pub.publish(mux_msg);
	}

	void change_controller(int controller_idx) {
		// This changes the controller to the input index and publishes it

		if (mux_controller[safety_copilot_mux_idx]) {
			// if the safety copilot is active, don't change to anything else
			return;
		}

		// turn everything off
		for (int i = 0; i < mux_size; i++) {
			mux_controller[i] = false;
		}
        // turn on desired controller
		mux_controller[controller_idx] = true;

		publish_mux();
	}

	void collision_checker(const sensor_msgs::LaserScan & msg) {
		// This function calculates TTC to see if there's a collision, and 
		// if safety copilot should take over
		if (state.velocity != 0) {
			for (size_t i = 0; i < msg.ranges.size(); i++) {
				double angle = msg.angle_min + i * msg.angle_increment;

            	// calculate projected velocity
				double proj_velocity = state.velocity * cosines[i];
				double ttc = (msg.ranges[i] - car_distances[i]) / proj_velocity;

				// if it's small, there's a collision
                if ((ttc < ttc_threshold) && (ttc >= 0.0)) { 
                	// Send a blank mux and write to file
                	collision_helper();

                	in_collision = true;

	                collision_count++;
	                collision_file << "Collision #" << collision_count << " detected:\n";
	                collision_file << "TTC: " << ttc << " seconds\n";
	                collision_file << "Angle to obstacle: " << angle << " radians\n";
	                collision_file << "Time since start of sim: " << (ros::Time::now().toSec() - beginning_seconds) << " seconds\n";
	                collision_file << "\n";
	                return;
            	}
        	}
        	// if it's gone through all beams without detecting a collision, reset in_collision
        	in_collision = false;
    	}
	}

	void collision_helper() {
		// This function will turn off safety copilot, clear the mux and publish it

		safety_copilot_on = false;

		// turn everything off
		for (int i = 0; i < mux_size; i++) {
			mux_controller[i] = false;
		}

		publish_mux();
	}

	void toggle_mux(int mux_idx, std::string driver_name) {
		// This takes in an index and the name of the planner/driver and 
		// toggles the mux appropiately
		if (mux_controller[mux_idx]) {
			ROS_INFO_STREAM(driver_name << " turned off");
			mux_controller[mux_idx] = false;
			publish_mux();
		}
		else {
			ROS_INFO_STREAM(driver_name << " turned on");
			change_controller(mux_idx);
		}
	}


	/// ---------------------- CALLBACK FUNCTIONS ----------------------

	void joy_callback(const sensor_msgs::Joy & msg) {
		// Changing mux_controller:
		if (msg.buttons[joy_button_idx]) { 
			// joystick
			toggle_mux(joy_mux_idx, "Joystick");
		}
		if (msg.buttons[key_button_idx]) { 
			// keyboard
			toggle_mux(key_mux_idx, "Keyboard");
		}
		else if (msg.buttons[safety_copilot_button_idx]) { 
			// safety copilot (can't use toggle_mux due to prev_controller stuff)
			if (safety_copilot_on) {
				ROS_INFO("Safety Copilot turned off");
				safety_copilot_on = false;
                // switch control to previous controller if safety copilot was on
				if (mux_controller[safety_copilot_mux_idx]) {
					mux_controller[safety_copilot_mux_idx] = false;
					change_controller(prev_controller_idx);
				}
			}
			else {
				ROS_INFO("Safety Copilot turned on");
				safety_copilot_on = true;
			}
		}
		else if (msg.buttons[random_walk_button_idx]) { 
			// random walker
			toggle_mux(random_walker_mux_idx, "Random Walker");
		}
		// ***Add new else if statement here for new planning method***
		// if (msg.buttons[new_button_idx]) {
		// 	// new planner
		// 	toggle_mux(new_mux_idx, "New Planner");
		// }

	}

	void key_callback(const std_msgs::String & msg) {
		// Changing mux controller:
		if (msg.data == joy_key_char) {
			// joystick
			toggle_mux(joy_mux_idx, "Joystick");
		} else if (msg.data == keyboard_key_char) {
			// keyboard
			toggle_mux(key_mux_idx, "Keyboard");
		} else if (msg.data == copilot_key_char) {
			// safety copilot (can't use toggle_mux due to prev_controller stuff)
			if (safety_copilot_on) {
				ROS_INFO("Safety Copilot turned off");
				safety_copilot_on = false;
                // switch control to previous controller if safety copilot was on
				if (mux_controller[safety_copilot_mux_idx]) {
					mux_controller[safety_copilot_mux_idx] = false;
					change_controller(prev_controller_idx);
				}
			}
			else {
				ROS_INFO("Safety Copilot turned on");
				safety_copilot_on = true;
			}
		} else if (msg.data == random_walk_key_char) {
			// random walker
			toggle_mux(random_walker_mux_idx, "Random Walker");
		} 
		// ***Add new else if statement here for new planning method***
		// if (msg.data == new_key_char) {
		// 	// new planner
		// 	toggle_mux(new_mux_idx, "New Planner");
		// }

	}

	void laser_callback(const sensor_msgs::LaserScan & msg) {
		// check for a collision
		collision_checker(msg);


	}

    void odom_callback(const nav_msgs::Odometry & msg) {
    	// Keep track of state to be used elsewhere
    	state.velocity = msg.twist.twist.linear.x;
    	state.angular_velocity = msg.twist.twist.angular.z;
    	state.x = msg.pose.pose.position.x;
    	state.y = msg.pose.pose.position.y;
	}

	void imu_callback(const sensor_msgs::Imu & msg) {

	}






};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "behavior_controller");
    BehaviorController bc;
    ros::spin();
    return 0;
}