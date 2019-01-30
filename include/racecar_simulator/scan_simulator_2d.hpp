#pragma once

#include <random>

#include "racecar_simulator/pose_2d.hpp"

namespace racecar_simulator {

class ScanSimulator2D {

  private:

    // Laser settings
    int num_beams;
    double field_of_view;
    double angle_increment;

    // Ray tracing settings
    double ray_tracing_epsilon;

    // The distance transform
    double resolution;
    size_t width, height;
    Pose2D origin;
    std::vector<double> dt;

    // Static output vector
    std::vector<double> scan_output;

    // Noise generation
    std::mt19937 noise_generator;
    std::normal_distribution<double> noise_dist;

  public:

    ScanSimulator2D() {}

    ScanSimulator2D(
        int num_beams_, 
        double field_of_view_, 
        double scan_std_dev, 
        double ray_tracing_epsilon=0.0001);

    void set_map(
        const std::vector<double> & map, 
        size_t height, 
        size_t width, 
        double resolution,
        const Pose2D & origin,
        double free_threshold);

    const std::vector<double> scan(const Pose2D & pose);

    double distance_transform(const Pose2D & pose) const;

    double trace_ray(const Pose2D & pose) const;

    double get_field_of_view() const {return field_of_view;}
    double get_angle_increment() const {return angle_increment;}
};

}
