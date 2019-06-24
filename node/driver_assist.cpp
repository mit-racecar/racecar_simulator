#include <ros/ros.h>

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include "racecar_simulator/pose_2d.hpp"

#include "racecar_simulator/Collision.h"


class DriverAssist {
private:
    // A ROS node
    ros::NodeHandle n;

    // car parameters
    double max_speed, max_steering_angle, max_steering_vel, max_accel;

    // Listen for collision messages
    ros::Subscriber coll_sub;

    // Publish drive data
    ros::Publisher drive_pub;

    const double PI = 3.1415;

public:

    DriverAssist() {
        // Initialize the node handle
        n = ros::NodeHandle("~");

        // get topic names
        std::string coll_topic, drive_topic;
        n.getParam("drive_topic", drive_topic);
        n.getParam("collision_topic", coll_topic);

        // get car parameters
        n.getParam("max_speed", max_speed);
        n.getParam("max_steering_angle", max_steering_angle);
        n.getParam("max_steering_vel", max_steering_vel);
        n.getParam("max_accel", max_accel);


        // Make a publisher for drive messages
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);

        // Start a subscriber to listen to collision messages
        coll_sub = n.subscribe(coll_topic, 1, &DriverAssist::coll_callback, this);
    }


    void coll_callback(const racecar_simulator::Collision & msg) {

        // if not active don't waste time calculating
        if (!msg.dr_assist_active)
            return;

        ackermann_msgs::AckermannDriveStamped drive_msg;
        ackermann_msgs::AckermannDrive dr;


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
                dr.acceleration = -max_accel * speed_sgn;
            else
                dr.acceleration = 0.0;


            // set desired steering angle to be away from obstacle
            dr.steering_angle = -(msg.wall_ang /std::abs(msg.wall_ang));



            drive_msg.drive = dr;
            drive_pub.publish(drive_msg);
       }





    }

};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "driver_assist");
    DriverAssist da;
    ros::spin();
    return 0;
}

