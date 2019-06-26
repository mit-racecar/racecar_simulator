#include <ros/ros.h>

// Publish to a topic with this message type
#include <ackermann_msgs/AckermannDriveStamped.h>
// AckermannDriveStamped messages include this message type
#include <ackermann_msgs/AckermannDrive.h>

// Subscribe to a topic with this message type
#include <std_msgs/Int32MultiArray.h>

// for printing
#include <iostream>

// for RAND_MAX
#include <cstdlib>


class RandomWalker {
private:
	// A ROS node
    ros::NodeHandle n;

    // car parameters
    double max_speed, max_steering_angle;

    // Listen for mux messages
    ros::Subscriber mux_sub;

    // Publish drive data
    ros::Publisher drive_pub;

    // index of random walker on mux
    int random_walker_mux_idx;

    // previous desired steering angle
    double prev_angle=0.0;


public:
	RandomWalker() {
		// Initialize the node handle
        n = ros::NodeHandle("~");

        // get topic names
        std::string drive_topic, mux_topic;
        n.getParam("drive_topic", drive_topic);
        n.getParam("mux_topic", mux_topic);

        // get car parameters
        n.getParam("max_speed", max_speed);
        n.getParam("max_steering_angle", max_steering_angle);

        // get mux idx to pay attention to
        n.getParam("random_walker_mux_idx", random_walker_mux_idx);


        // Make a publisher for drive messages
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);

        // Start a subscriber to listen to mux messages
        mux_sub = n.subscribe(mux_topic, 1, &RandomWalker::mux_callback, this);


	}


	void mux_callback(const std_msgs::Int32MultiArray & msg) {

		// if this mux channel isn't on, don't do anything
        if (!bool(msg.data[random_walker_mux_idx])) 
        	return;

        // initialize message to be published
        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::AckermannDrive drive_msg;

        /// SPEED CALCULATION:
        // set constant speed to be half of max speed
        drive_msg.speed = max_speed / 2.0;


        /// STEERING ANGLE CALCULATION
        // random number between 0 and 1
        double random = ((double) rand() / RAND_MAX);
        // good range to cause lots of turning
        double range = max_steering_angle / 2.0;
        // compute random amount to change desired angle by (between -range and range)
        double rand_ang = range * random - range / 2.0;

        // sometimes change sign so it turns more (basically add bias to continue turning in current direction)
        random = ((double) rand() / RAND_MAX);
        if ((random > .8) && (prev_angle != 0)) {
        	double sign_rand = rand_ang / std::abs(rand_ang);
        	double sign_prev = prev_angle / std::abs(prev_angle);
        	rand_ang *= sign_rand * sign_prev;
        }

        // set angle (add random change to previous angle)
        drive_msg.steering_angle = std::min(std::max(prev_angle + rand_ang, -max_steering_angle), max_steering_angle);

        // reset previous desired angle
        prev_angle = drive_msg.steering_angle;


        // set drive message in drive stamped message
        drive_st_msg.drive = drive_msg;

        // publish AckermannDriveStamped message to drive topic
        drive_pub.publish(drive_st_msg);


    }

}; // end of class definition


int main(int argc, char ** argv) {
    ros::init(argc, argv, "random_walker");
    RandomWalker rw;
    ros::spin();
    return 0;
}

