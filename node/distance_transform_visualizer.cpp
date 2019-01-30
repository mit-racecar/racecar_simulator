#include <limits>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include "racecar_simulator/distance_transform.hpp"

using namespace racecar_simulator;

class DistanceTransformVisualizer {
  private:
    // Anything below this is considered free
    double free_threshold_probability;

    // A ROS node
    ros::NodeHandle n;

    // A timer to update the pose
    ros::Timer update_pose_timer;
    double update_pose_rate;

    // Listen for a map
    ros::Subscriber map_sub;

    // Publish the distance transform
    ros::Publisher dt_pub;

  public:

    DistanceTransformVisualizer() {
      // Fetch the ROS parameters
      std::string map_topic = "/map";
      std::string dt_topic = "/dt";
      free_threshold_probability = 0.3;

      // Make a publisher for the distance transform
      dt_pub = n.advertise<nav_msgs::OccupancyGrid>(dt_topic, 1, true);

      // Start a subscriber to listen to new maps
      map_sub = n.subscribe(map_topic, 1, &DistanceTransformVisualizer::map_callback, this);
    }

    void map_callback(const nav_msgs::OccupancyGrid & map_msg) {
      nav_msgs::OccupancyGrid dt_msg;
      dt_msg.header = map_msg.header;
      dt_msg.info = map_msg.info;

      std::vector<double> dt(map_msg.data.size());

      // Make occupied inf and unoccupied 0
      for (size_t i = 0; i < map_msg.data.size(); i++) {
        if (map_msg.data[i]/100. <= free_threshold_probability and
            map_msg.data[i] >= 0) {
          dt[i] = 99999;
        } else {
          dt[i] = 0;
        }
      }
     
      // Apply the distance transform
      DistanceTransform::distance_2d(
          dt,
          dt_msg.info.width,
          dt_msg.info.height);

      // Determine the maximum value
      double max_dist = *std::max_element(dt.begin(), dt.end());
      
      // Put into map message
      dt_msg.data = std::vector<int8_t>(dt.size());
      for (size_t i = 0; i < dt.size(); i++) {
        dt_msg.data[i] = 100 * (1 - dt[i]/max_dist);
      }

      dt_pub.publish(dt_msg);
    }
};

int main(int argc, char ** argv) {
  ros::init(argc, argv, "distance_transform_visualizer");
  DistanceTransformVisualizer dt;
  ros::spin();
  return 0;
}
