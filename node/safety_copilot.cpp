#include <ros/ros.h>

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <std_msgs/Int32MultiArray.h>


#include "racecar_simulator/Collision.h"


class SafetyCopilot {
private:
    // A ROS node
    ros::NodeHandle n;

    // car parameters
    double max_speed, max_steering_angle, max_steering_vel, max_accel;

    // Listen for collision messages
    ros::Subscriber coll_sub;

    // Listen for mux messages
    ros::Subscriber mux_sub;

    // Publish drive data
    ros::Publisher drive_pub;

    // Pi
    const double PI = 3.1415;

    // index of safety copilot on mux
    int safety_copilot_mux_idx;

    // boolean set by mux callback
    bool safety_copilot_on;

public:

    SafetyCopilot() {
        // Initialize the node handle
        n = ros::NodeHandle("~");

        // get topic names
        std::string coll_topic, drive_topic, mux_topic;
        n.getParam("drive_topic", drive_topic);
        n.getParam("collision_topic", coll_topic);
        n.getParam("mux_topic", mux_topic);

        // get car parameters
        n.getParam("max_speed", max_speed);
        n.getParam("max_steering_angle", max_steering_angle);
        n.getParam("max_steering_vel", max_steering_vel);
        n.getParam("max_accel", max_accel);

        n.getParam("safety_copilot_mux_idx", safety_copilot_mux_idx);


        // Make a publisher for drive messages
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);

        // Start a subscriber to listen to collision messages
        coll_sub = n.subscribe(coll_topic, 1, &SafetyCopilot::coll_callback, this);

        // Start a subscriber to listen to mux messages
        mux_sub = n.subscribe(mux_topic, 1, &SafetyCopilot::mux_callback, this);

        safety_copilot_on = false;

    }


    void coll_callback(const racecar_simulator::Collision & msg) {

        // if not active or set on by mux, don't waste time calculating
        if (!msg.copilot_active && !safety_copilot_on)
            return;

        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::AckermannDrive drive_msg;


        //if close enough to obstacle, change steering and/or speed
        if (msg.in_danger) {

            // get sign of speed
            double speed_sgn;
            if (msg.speed == 0)
                speed_sgn = 0;
            else
                speed_sgn = msg.speed / (std::abs(msg.speed));


            // slow down if heading straight towards obstacle or going fast
            // values to tune- angle threshold base off of max steering angle?
            // speed threshold base off of max accel?

            // to account for heading backwards
            double angle2obs;
            if (std::abs(msg.wall_ang) < PI/2)
                angle2obs = std::abs(msg.wall_ang);
            else
                angle2obs = PI - std::abs(msg.wall_ang);

            if ((angle2obs < .5) || (std::abs(msg.speed) > 2)) // params to be tuned
                drive_msg.speed = -max_speed * speed_sgn;
            else
                drive_msg.speed = 0.0;


            // set desired steering angle to be away from obstacle
            drive_msg.steering_angle = -(msg.wall_ang /std::abs(msg.wall_ang));


            // set drive message in drive stamped message
            drive_st_msg.drive = drive_msg;

            // publish AckermannDriveStamped message to drive topic
            drive_pub.publish(drive_st_msg);
       }

    }



    void mux_callback(const std_msgs::Int32MultiArray & msg) {
        // listen to mux to turn this node off and on
        safety_copilot_on = bool(msg.data[safety_copilot_mux_idx]);
    }

};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "safety_copilot");
    SafetyCopilot sc;
    ros::spin();
    return 0;
}

