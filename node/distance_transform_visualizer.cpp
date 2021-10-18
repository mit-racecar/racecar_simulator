#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "racecar_simulator/distance_transform.hpp"

namespace distance_transform_visualizer
{

class DistanceTransformVisualizer : public rclcpp::Node {
  private:
    // Anything below this is considered free
    double map_free_threshold_;

    // Listen for a map
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

    // Publish the distance transform
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr dt_pub_;

  public:

    DistanceTransformVisualizer() : rclcpp::Node("distance_transform_visualizer") {

      // Fetch the ROS parameters
      std::string map_topic, dt_topic;
      map_topic = declare_parameter<std::string>("map_topic");
      dt_topic = declare_parameter<std::string>("distance_transform_topic");
      map_free_threshold_ = declare_parameter<double>("map_free_threshold");

      // Make a publisher for the distance transform
      dt_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(dt_topic, rclcpp::QoS(1));

      // Start a subscriber to listen to new maps
      map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(map_topic, rclcpp::QoS(1), 
        std::bind(&DistanceTransformVisualizer::map_callback, this, std::placeholders::_1));
    }

    void map_callback(const nav_msgs::msg::OccupancyGrid & map_msg) {
      nav_msgs::msg::OccupancyGrid dt_msg;
      dt_msg.header = map_msg.header;
      dt_msg.info = map_msg.info;

      std::vector<double> dt(map_msg.data.size());

      // Make occupied inf and unoccupied 0
      for (size_t i = 0; i < map_msg.data.size(); i++) {
        if (map_msg.data[i]/100. <= map_free_threshold_ and
            map_msg.data[i] >= 0) {
          dt[i] = 99999;
        } else {
          dt[i] = 0;
        }
      }
     
      // Apply the distance transform
      racecar_simulator::DistanceTransform::distance_2d(
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

      dt_pub_->publish(dt_msg);
    }
};

} // namespace distance_transform_visualizer

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<distance_transform_visualizer::DistanceTransformVisualizer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
