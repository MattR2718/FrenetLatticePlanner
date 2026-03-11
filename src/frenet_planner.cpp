#include "frenet_lattice_planner/frenet_planner.h"

FrenetPlanner::FrenetPlanner() : Node("frenet_planner_node") {

    // Declare a paremeter for the waypoint path
    // Can be overriden when launching
    this->declare_parameter<std::string>("waypoint_file_path", "/home/matthew/PurePursuitWaypoints/Spielberg.csv");
    
    std::string filename;
    this->get_parameter("waypoint_file_path", filename);
    load_waypoints(filename);

    // Declare parameter for drawing boolean
    this->declare_parameter<bool>("draw_lattice", true);
    this->get_parameter("draw_lattice", draw_lattice);

    // Initialize the Marker Publisher
    marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/lattice_markers", 10);

    // Initialise Odometry subscriber
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/ego_racecar/odom", 10,
        std::bind(&FrenetPlanner::odom_callback, this, std::placeholders::_1)
    );

    // Initialise Scan Callback
    scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&FrenetPlanner::scan_callback, this, std::placeholders::_1)
    );

    drive_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

}

void FrenetPlanner::stop_car(){
    ackermann_msgs::msg::AckermannDriveStamped brake_msg;
    brake_msg.header.stamp = this->get_clock()->now();
    brake_msg.header.frame_id = "base_link";
    
    brake_msg.drive.speed = 0.0;
    brake_msg.drive.steering_angle = 0.0;

    // Publish a few times to guarantee the message makes it over the DDS network
    for (int i = 0; i < 3; i++) {
        drive_pub->publish(brake_msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void FrenetPlanner::publish_lattice_markers(const std::vector<FrenetTrajectory>& safe_lattice, const std::vector<FrenetTrajectory>& unsafe_lattice, const FrenetTrajectory& chosen_traj){
    visualization_msgs::msg::MarkerArray marker_array;

    // Create a "Delete All" marker to clear the old lattice from the previous frame
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    int id = 0; // Keep track of global ID across both loops

    // Helper to draw lattice
    auto add_lattice_to_markers = [&](const std::vector<FrenetTrajectory>& lattice, float r, float g, float b, float a = 0.8) {
        for (const auto& traj : lattice) {
            visualization_msgs::msg::Marker line_marker;
            
            line_marker.header.frame_id = "map"; 
            line_marker.header.stamp = this->get_clock()->now();
            line_marker.ns = "frenet_lattice";
            line_marker.id = id++;
            
            line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line_marker.action = visualization_msgs::msg::Marker::ADD;
            line_marker.scale.x = 0.03; // Line width
            
            // Set the color
            line_marker.color.r = r;
            line_marker.color.g = g;
            line_marker.color.b = b;
            line_marker.color.a = a; // Slightly transparent

            // Loop through the trajectory's points
            for (size_t i = 0; i < traj.x.size(); ++i) {
                geometry_msgs::msg::Point p;
                p.x = traj.x[i];
                p.y = traj.y[i];
                p.z = 0.0;
                line_marker.points.push_back(p);
            }
            marker_array.markers.push_back(line_marker);
        }
    };

    add_lattice_to_markers(safe_lattice, 0.0, 1.0, 0.0, 0.4);
    add_lattice_to_markers(unsafe_lattice, 1.0, 0.0, 0.0, 0.4);

    std::vector<FrenetTrajectory> t = {chosen_traj};
    add_lattice_to_markers(t, 0.0, 0.0, 1.0);

    // Publish to RViz
    marker_pub->publish(marker_array);
}

void FrenetPlanner::load_waypoints(const std::string& filename){
    std::ifstream file(filename);

    if(!file.is_open()){
        RCLCPP_ERROR(this->get_logger(), "ERROR: COULD NOT OPEN FILE: [%s]", filename.c_str());
        return;
    }

    std::string linetxt;
    double acc_s = 0.0;
    bool first = true;
    Waypoint prev;

    while(std::getline(file, linetxt)){
        std::stringstream ss(linetxt);
        std::string val;
        Waypoint wp;

        // Parse x, y, yaw
        if (std::getline(ss, val, ',')) wp.x = std::stod(val);
        if (std::getline(ss, val, ',')) wp.y = std::stod(val);
        //if (std::getline(ss, val, ',')) wp.yaw_rad = (std::stod(val) - 90) * (M_PI / 180.0);

        // Calculate s
        if(first){
            first = false;
            wp.s = 0.0;

            global_path.push_back(wp);
            prev = wp;
        } else{
            double dist = std::hypot(wp.x - prev.x, wp.y - prev.y);
            if(dist < 1e-4) continue;
            
            acc_s += dist;
            wp.s = acc_s;

            global_path.push_back(wp);
            prev = wp;
        }

        
    }


    // Calculate perfect yaw
    for(size_t i = 0; i < global_path.size(); i++){
        if(i < global_path.size() - 1){
            double dx = global_path[i+1].x - global_path[i].x;
            double dy = global_path[i+1].y - global_path[i].y;
            global_path[i].yaw_rad = std::atan2(dy, dx);
        } else {
            double dx = global_path[0].x - global_path[i].x;
            double dy = global_path[0].y - global_path[i].y;

            // Safety check if first and last points are practically identical
            if (std::hypot(dx, dy) < 1e-4) {
                global_path[i].yaw_rad = global_path[i-1].yaw_rad;
            } else {
                global_path[i].yaw_rad = std::atan2(dy, dx);
            }
        }
    }


    if(!global_path.empty()){
        RCLCPP_INFO(this->get_logger(), "SUCCESS! Loaded %zu waypoints.", global_path.size());
        RCLCPP_INFO(this->get_logger(), "Total track length (s): %.2f meters.", global_path.back().s);
    }else{
        RCLCPP_INFO(this->get_logger(), "ERROR: NO WAYPOINTS LOADED");
    }
}

void FrenetPlanner::get_frenet_state(double cx, double cy, double &cs, double &cd){
    if(global_path.empty()){
        return;
    }

    // Find closest waypoint
    Waypoint c_wp = *std::min_element(global_path.begin(), global_path.end(), [&](const Waypoint& a, const Waypoint& b) {
        double dist_a = std::hypot(cx - a.x, cy - a.y);
        double dist_b = std::hypot(cx - b.x, cy - b.y);
        return dist_a < dist_b; 
    });

    cs = c_wp.s;

    // Calculate signed d
    double dx = cx - c_wp.x;
    double dy = cy - c_wp.y;

    // Normal vector of track
    double nx = -std::sin(c_wp.yaw_rad);
    double ny = std::cos(c_wp.yaw_rad);

    cd = (dx * nx) + (dy * ny);
}

void FrenetPlanner::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
    // Extract x and y
    curr_car_x = msg->pose.pose.position.x;
    curr_car_y = msg->pose.pose.position.y;


    // Extract current yaw using TF2 Quaternions
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, curr_car_yaw);


    // Extract current speed
    double curr_v = msg->twist.twist.linear.x;
    // If car is still, give small fake velocity
    if(std::abs(curr_v) < 0.1){
        curr_v = 2.5;
    }

    double curr_s = 0.0;
    double curr_d = 0.0;

    // Get current state
    get_frenet_state(curr_car_x, curr_car_y, curr_s, curr_d);

    // Generate lattice
    std::vector<FrenetTrajectory> full_lattice = generate_lattice(curr_s, curr_d, curr_v);

    // Convert lattice to cartesian
    for(auto& traj : full_lattice){
        convert_to_cartesian(traj);
    }

    std::vector<FrenetTrajectory> safe_lattice;
    std::vector<FrenetTrajectory> unsafe_lattice;

    // Prune lattice
    for(const auto& traj : full_lattice){
        if(check_collision(traj)){
            unsafe_lattice.push_back(traj);
        }else{
            safe_lattice.push_back(traj);
        }
    }

    // Cost evaluation
    FrenetTrajectory best_trajectory;
    bool found_path = false;
    if(!safe_lattice.empty()){
        calculate_trajectory_costs(safe_lattice);

        auto best_it = std::min_element(safe_lattice.begin(), safe_lattice.end(),
            [](const FrenetTrajectory&a, const FrenetTrajectory& b){
                return a.cost < b.cost;
            }
        );

        best_trajectory = *best_it;
        found_path = true;
    }


    if(draw_lattice){
        publish_lattice_markers(safe_lattice, unsafe_lattice, best_trajectory);
    }

    if (found_path) {
        RCLCPP_INFO(this->get_logger(), "Best Path Cost: %.2f | Target D: %.2f", 
                    best_trajectory.cost, best_trajectory.d.back());
                    
        prev_best_d = best_trajectory.d.back();
        // TODO: Pass 'best_trajectory.x' and 'best_trajectory.y' to your path tracking controller (like Pure Pursuit)
        track_path(best_trajectory, curr_v);
    } else {
        RCLCPP_WARN(this->get_logger(), "EMERGENCY: No safe paths found! Brake!");
        // TODO: Publish a 0 velocity drive command here
        ackermann_msgs::msg::AckermannDriveStamped recovery_msg;
        recovery_msg.header.stamp = this->get_clock()->now();
        recovery_msg.header.frame_id = "base_link";

        // Reverse at -1.0 m/s
        recovery_msg.drive.speed = -1.0; 
        
        // Steer straight back
        recovery_msg.drive.steering_angle = 0.0; 
        
        drive_pub->publish(recovery_msg);
        
    }

    /*
    // Print out a summary to the terminal
    RCLCPP_INFO(this->get_logger(), "Generated %zu trajectories in the lattice, %zu are safe.", full_lattice.size(), safe_lattice.size());
    if (!safe_lattice.empty()) {
        RCLCPP_INFO(this->get_logger(), "First trajectory has %zu points. End target: s=%.2f, d=%.2f", 
                    safe_lattice[0].s.size(), safe_lattice[0].s.back(), safe_lattice[0].d.back());
    }
    */
}

void FrenetPlanner::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    std::vector<geometry_msgs::msg::Point> new_obstacles;

    for(size_t i = 0; i < msg->ranges.size(); i++){
        double r = msg->ranges[i];

        // Ignore infinite, NaN, or out-of-range readings
        if(std::isinf(r) || std::isnan(r) || r < msg->range_min || r > msg->range_max){
            continue;
        }

        double angle = msg->angle_min + i * msg->angle_increment;

        double local_x = r * std::cos(angle);
        double local_y = r * std::sin(angle);

        geometry_msgs::msg::Point global_pt;
        global_pt.x = curr_car_x + (local_x * std::cos(curr_car_yaw) - local_y * std::sin(curr_car_yaw));
        global_pt.y = curr_car_y + (local_x * std::sin(curr_car_yaw) + local_y * std::cos(curr_car_yaw));
        global_pt.z = 0.0;

        new_obstacles.push_back(global_pt);
    }

    std::lock_guard<std::mutex> lock(obstacle_mutex);
    global_obstacles = new_obstacles;
}

std::vector<FrenetTrajectory> FrenetPlanner::generate_lattice(double cs, double cd, double cv){
    std::vector<FrenetTrajectory> lattice;

    //
    //
    // Tuning parameters
    // Add more to allow for more possibilities in the lattice
    std::vector<double> target_lat_offsets = {
        -1.0, -0.8, -0.6, -0.4, -0.2, -0.1, -0.075, -0.05, -0.025,
        0.0, 
        0.025, 0.05, 0.075, 0.1, 0.2, 0.4, 0.6, 0.8, 1.0
    };
    std::vector<double> target_times = {1.5, 2.0, 2.5}; // Lookahead times in s
    double time_step = 0.05; // Resolution of trajectories

    // Assume cars lateral velocity and acceleration are 0
    double cdv = 0.0;
    double cda = 0.0;

    for(double T : target_times){
        for(double df : target_lat_offsets){
            FrenetTrajectory traj;

            // Generate quintic polynomial for lateral movement
            // Target vel and acc are 0 as we want car to stabilise parallel to track
            QuinticPolynomial lat_qp(cd, cdv, cda, df, 0.0, 0.0, T);

            // Step thorugh time to build trajectory points
            for(double t = 0.0; t <= T; t += time_step){
                traj.t.push_back(t);
                traj.d.push_back(lat_qp.calc_point(t));

                // For basic planner, assume constant forward velocity
                // Calculate longitudinal position
                double spos = cs + (cv * t);
                traj.s.push_back(spos);
            }

            lattice.push_back(traj);
        }
    }

    return lattice;
}

// Gets a waypoint representation for a target s
Waypoint FrenetPlanner::get_reference_waypoint(double target_s){
    // Check that S is within the track
    //if(target_s <= global_path.front().s) return global_path.front();
    //if(target_s >= global_path.back().s) return global_path.back();

    if(global_path.empty()) return Waypoint();

    double max_s = global_path.back().s;

    target_s = std::fmod(target_s, max_s);

    if(target_s < 0.0) target_s += max_s;

    // Binary Search to fine two waypoints that surround target s
    auto wp_it = std::upper_bound(global_path.begin(), global_path.end(), Waypoint{0.0, 0.0, 0.0, target_s}, [](const Waypoint& a, const Waypoint& b){
        return a.s < b.s;
    });

    if(wp_it == global_path.begin()){
        return global_path.front();
    }

    Waypoint wp1 = *wp_it--;
    Waypoint wp0 = *wp_it;

    // Linear interpolation for smooth X, Y
    double ratio = (target_s - wp0.s) / (wp1.s - wp0.s);

    Waypoint interp_wp;
    interp_wp.x = wp0.x + ratio * (wp1.x - wp0.x);
    interp_wp.y = wp0.y + ratio * (wp1.y - wp0.y);

    // Snap to wp0 yaw to avoid complex wrap around math
    //interp_wp.yaw_rad = wp0.yaw_rad;
    //interp_wp.s = target_s;

    // Smoothly interpolate yaw
    double yaw_diff = wp1.yaw_rad - wp0.yaw_rad;
    while(yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI;
    while(yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;
    interp_wp.yaw_rad = wp0.yaw_rad + (ratio * yaw_diff);

    interp_wp.s = target_s;

    return interp_wp;

}

void FrenetPlanner::convert_to_cartesian(FrenetTrajectory &traj){
    traj.x.clear();
    traj.y.clear();
    traj.yaw.clear();

    // Calculate X and Y for every point
    for(size_t i = 0; i < traj.s.size(); i++){
        Waypoint ref_wp = get_reference_waypoint(traj.s[i]);

        // Get Normal vector pointing 90 deg left of track dir
        double nx = -std::sin(ref_wp.yaw_rad);
        double ny = std::cos(ref_wp.yaw_rad);

        // Project out by d
        double px = ref_wp.x + (nx * traj.d[i]);
        double py = ref_wp.y + (ny * traj.d[i]);

        traj.x.push_back(px);
        traj.y.push_back(py);
    }

    // Calculate the new yaw of the trajectory
    for(size_t i = 0; i < traj.x.size(); i++){
        if(i == 0 && traj.x.size() > 1){
            double dx = traj.x[1] - traj.x[0];
            double dy = traj.y[1] - traj.y[0];
            traj.yaw.push_back(std::atan2(dy, dx));
        }else{
            double dx = traj.x[i] - traj.x[i-1];
            double dy = traj.y[i] - traj.y[i-1];
            traj.yaw.push_back(std::atan2(dy, dx));
        }
    }

}

bool FrenetPlanner::check_collision(const FrenetTrajectory& traj){
    // F1TENTH scale: 0.3m radius ~= car size
    double collision_rad = 0.2;
    double square_safe_dist = collision_rad * collision_rad;

    // Lock mutex to read latest obstacles safety
    std::lock_guard<std::mutex> lock(obstacle_mutex);

    // If no lidar data, assume its unsafe
    if(global_obstacles.empty()) return false;

    // Step through trajectory
    // Can skip by 2 or 3 to speed up if it runs too slow
    for(size_t i = 0; i < traj.x.size(); i++){
        for(const auto& obs : global_obstacles){
            double dx = traj.x[i] - obs.x;
            double dy = traj.y[i] - obs.y;

            // If any trajectory is inside collision circle then crash
            if((dx * dx + dy * dy) <= square_safe_dist){
                return true;
            }
        }
    }

    return false;
}

void FrenetPlanner::calculate_trajectory_costs(std::vector<FrenetTrajectory>& safe_lattice){
    std::lock_guard<std::mutex> lock(obstacle_mutex);

    for(auto& traj : safe_lattice){
        double cost_offset = std::abs(traj.d.back());

        // How violently is the car swerving
        double total_time = traj.t.back();
        double d_diff = std::abs(traj.d.back() - traj.d.front());
        double cost_smoothness = (total_time > 0.0) ? (d_diff / total_time) : 0.0;

        // Obstacle proximity
        double min_dist_sq = std::numeric_limits<double>::max();

        if(!global_obstacles.empty()){
            // Check every few points to save CPU
            for(size_t i = 0; i < traj.x.size(); i += 3){
                for(const auto& obs : global_obstacles){
                    double dx = traj.x[i] - obs.x;
                    double dy = traj.y[i] - obs.y;
                    double dist_sq = dx * dx + dy * dy;
                    if(dist_sq < min_dist_sq){
                        min_dist_sq = dist_sq;
                    }
                }
            }
        }

        double cost_obs = 0.0;
        //Only penalise if the path gets closer than ~1.4m
        // Use 1/dist to make cost spike as it gets closer
        if(min_dist_sq > 0.01 && min_dist_sq < std::pow(1.4, 2)){
            cost_obs = 1.0 / std::sqrt(min_dist_sq);
        }

        double cost_consistency = std::abs(traj.d.back() - prev_best_d);

        traj.cost = (w_offset * cost_offset) + 
                    (w_smoothness * cost_smoothness) + 
                    (w_obstacle * cost_obs) +
                    (w_consistency * cost_consistency);
    }
}

bool FrenetPlanner::check_line_of_sight(double x1, double y1, double x2, double y2){
    std::lock_guard<std::mutex> lock(obstacle_mutex);
    if (global_obstacles.empty()) return true;

    double dx = x2 - x1;
    double dy = y2 - y1;
    double dist = std::hypot(dx, dy);

    if (dist < 0.01) return true; // Too short to matter

    // Discretize the line into 10cm steps
    int steps = std::ceil(dist / 0.1); 
    double step_x = dx / steps;
    double step_y = dy / steps;

    // Use a slightly smaller radius for the LoS check than the full car body
    // to allow *some* corner cutting, but not enough to crash. 0.2m is safe.
    double los_safe_dist_sq = 0.2 * 0.2; 

    // Walk along the line and check against obstacles
    for (int i = 0; i <= steps; i++) {
        double cx = x1 + i * step_x;
        double cy = y1 + i * step_y;

        for (const auto& obs : global_obstacles) {
            double ox = cx - obs.x;
            double oy = cy - obs.y;
            if ((ox * ox + oy * oy) <= los_safe_dist_sq) {
                return false; // Line of sight is blocked!
            }
        }
    }
    return true;
}

void FrenetPlanner::track_path(const FrenetTrajectory& target_path, double current_speed){

    if (target_path.x.empty()) return;

    double max_lookahead = std::clamp(current_speed * 0.5, 0.5, 3.0);

    // Find the furthest lookahead point with a clear line of sight
    double target_x = target_path.x.back();
    double target_y = target_path.y.back();

    for (size_t i = 0; i < target_path.x.size(); i++) {
        double dx = target_path.x[i] - curr_car_x;
        double dy = target_path.y[i] - curr_car_y;
        double dist = std::hypot(dx, dy);

        // ONLY consider points that are at least 0.2m in front of us.
        if (dist > 0.2 && dist <= max_lookahead) {
            if (check_line_of_sight(curr_car_x, curr_car_y, target_path.x[i], target_path.y[i])) {
                target_x = target_path.x[i];
                target_y = target_path.y[i];
            } else {
                // Line of sight blocked, stop searching
                break;
            }
        } else if (dist > max_lookahead) {
            // We've passed our lookahead limit, stop searching
            break;
        }
    }

    // 2. Transform the target point to the car's local frame
    double dx = target_x - curr_car_x;
    double dy = target_y - curr_car_y;

    // Rotate by -yaw to make the car the origin (0,0) facing perfectly forward along the X-axis
    double local_x = dx * std::cos(-curr_car_yaw) - dy * std::sin(-curr_car_yaw);
    double local_y = dx * std::sin(-curr_car_yaw) + dy * std::cos(-curr_car_yaw);

    // 3. Calculate the Pure Pursuit Steering Angle
    double steering_angle = 0.0;
    
    double true_lookahead_sq = (local_x * local_x) + (local_y * local_y);
    if (true_lookahead_sq > 0.01) {
        // Calculate the curvature of the arc
        double curvature = (2.0 * local_y) / true_lookahead_sq;
        
        // Convert curvature to a physical steering angle using the wheelbase
        steering_angle = std::atan(curvature * wheelbase); 
    }

    // Clip the steering angle to the physical limits of an F1TENTH car (~0.4 radians)
    steering_angle = std::clamp(steering_angle, -0.4, 0.4);

    // 4. Calculate a predictive safe velocity based on upcoming curvature
    double max_curvature = 0.0;
    
    // Loop through the future trajectory points to find the sharpest curve
    for (size_t i = 1; i < target_path.x.size() - 1; i++) {
        double x1 = target_path.x[i-1], y1 = target_path.y[i-1];
        double x2 = target_path.x[i],   y2 = target_path.y[i];
        double x3 = target_path.x[i+1], y3 = target_path.y[i+1];
        
        // Calculate the area of the triangle formed by these 3 points
        double area = 0.5 * std::abs(x1*(y2 - y3) + x2*(y3 - y1) + x3*(y1 - y2));
        
        // Calculate the lengths of the 3 sides of the triangle
        double a = std::hypot(x1 - x2, y1 - y2);
        double b = std::hypot(x2 - x3, y2 - y3);
        double c = std::hypot(x3 - x1, y3 - y1);
        
        // Curvature (kappa) = 4 * Area / (a * b * c)
        if ((a * b * c) > 0.0001) {
            double curvature = (4.0 * area) / (a * b * c);
            if (curvature > max_curvature) {
                max_curvature = curvature; // Save the sharpest part of the upcoming track
            }
        }
    }

    // Now, use physics to calculate the maximum safe speed!
    // F1TENTH tires can typically handle ~3.0 to 4.5 m/s^2 of lateral acceleration
    double max_lateral_accel = 3.5; 
    double target_velocity = 6.0;   // Absolute top speed on straightaways
    
    // If there is a noticeable turn coming up anywhere in our trajectory
    if (max_curvature > 0.05) { 
        double curve_speed_limit = std::sqrt(max_lateral_accel / max_curvature);
        
        // Limit our speed to whichever is lower: our top speed, or the physics limit of the curve
        target_velocity = std::min(target_velocity, curve_speed_limit);
    }

    // Clamp to a safe minimum speed so the car doesn't stall in incredibly sharp hairpins
    target_velocity = std::clamp(target_velocity, 1.0, 6.0);

    // 5. Publish the Drive Command
    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = this->get_clock()->now();
    drive_msg.header.frame_id = "base_link"; // standard for F1TENTH
    
    drive_msg.drive.steering_angle = steering_angle;
    drive_msg.drive.speed = target_velocity;

    drive_pub->publish(drive_msg);
}


std::shared_ptr<FrenetPlanner> g_node = nullptr;

// Custom Signal Handler for Ctrl+C
void sigint_handler(int sig) {
    // Silence unused variable warning
    (void)sig; 
    
    if (g_node) {
        RCLCPP_WARN(g_node->get_logger(), "CTRL+C DETECTED! Braking...");
        g_node->stop_car();
    }
    
    // Let ROS 2 shut down completely
    rclcpp::shutdown();
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    g_node = std::make_shared<FrenetPlanner>();
    
    // Override the default ROS 2 SIGINT (Ctrl+C) handler with our own
    std::signal(SIGINT, sigint_handler);
    
    // Spin normally until Ctrl+C is pressed
    rclcpp::spin(g_node);
    
    return 0;
}