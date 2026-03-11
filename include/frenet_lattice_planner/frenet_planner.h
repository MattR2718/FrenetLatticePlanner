#ifndef FRENET_LATTICE_PLANNER__FRENET_PLANNER_H
#define FRENET_LATTICE_PLANNER__FRENET_PLANNER_H

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

#include "quintic_polynomial.h"

#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <cmath>
#include <mutex>
#include <csignal>
#include <thread>
#include <chrono>


// Hold track data
struct Waypoint{
    double x;
    double y;
    double yaw_rad;
    double s; // Distance along track
};

// Store trajectory
struct FrenetTrajectory{
    std::vector<double> t; // Time
    std::vector<double> d; // Lateral Positions
    std::vector<double> s; // Longitudinal Positions

    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> yaw;

    double cost = 0.0;
};

class FrenetPlanner : public rclcpp::Node {
public:
    FrenetPlanner();

    void stop_car();

private:
    double curr_car_x = 0.0;
    double curr_car_y = 0.0;
    double curr_car_yaw = 0.0;

    // Cost function Weights
    double w_offset = 2.0; // return to racing line
    double w_smoothness = 2.0; // Prefer straight lines
    double w_obstacle = 8.0; // Dodge obstacles more
    double w_consistency = 2.0; // Penalty for switching path
    double prev_best_d = 0.0; // Remember dist chosen last frame


    std::vector<Waypoint> global_path; // Store waypoint patj

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub;


    std::mutex obstacle_mutex;
    std::vector<geometry_msgs::msg::Point> global_obstacles;

    // Draw lattice
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
    bool draw_lattice;
    void publish_lattice_markers(const std::vector<FrenetTrajectory>& safe_lattice, const std::vector<FrenetTrajectory>& unsafe_lattice, const FrenetTrajectory& chosen_traj);

    // Load waypoint csv
    void load_waypoints(const std::string& filename);

    // Callback when we receive a new position
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    // Convert X, Y to s, d
    void get_frenet_state(double cx, double cy, double &cs, double &cd);

    // Generate lattice
    std::vector<FrenetTrajectory> generate_lattice(double cs, double cd, double cv);


    // FInd the exact X, Y and Yaw on the line for a given s
    Waypoint get_reference_waypoint(double target_s);

    // Convert an entire trajectory from (s, d) to (X, Y)
    void convert_to_cartesian(FrenetTrajectory& traj);

    bool check_collision(const FrenetTrajectory& traj);

    void calculate_trajectory_costs(std::vector<FrenetTrajectory>& safe_lattice);


    // Pure pursiut tuning parameters
    double lookahead_distance = 1.0;
    double wheelbase = 0.33; // Legth of car

    bool check_line_of_sight(double x1, double y1, double x2, double y2);
    void track_path(const FrenetTrajectory& target_path, double current_speed);

};

#endif  // FRENET_LATTICE_PLANNER__FRENET_PLANNER_H