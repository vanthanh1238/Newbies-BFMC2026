#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <Eigen/Dense>
#include <tf2/utils.h>
#include <algorithm>
#include <iomanip>
#include <path_planning/planning/PathManager.h>
#include <path_planning/utils/Utility.hpp>
#include <serial/serial.h>

class StanleyControlNode {
private:
    ros::NodeHandle nh; 
    ros::Subscriber odom_sub;
    ros::Publisher car_cmd_pub;
    ros::Publisher error_pub;
    ros::Timer timer;

    // --- Controller Parameters ---
    const double k_gain = 0.4;       
    const double wheelbase = 0.118;    
    const double max_steer = 35.0;   
    const double k_soft = 0.1;       


    serial::Serial ser; 
    // --- State Variables ---
    nav_msgs::Odometry last_odom;
    bool path_ready = false;
    bool odom_received = false;

    // Utility utils;
    // std::unique_ptr<Utility> utils;
public:
    StanleyControlNode() {
        // Initialize Utility (This handles your publishers internally)
        // utils = std::make_unique<Utility>(nh);
        // Subscribe to Filtered Odometry from EKF
        odom_sub = nh.subscribe("/odometry/filtered", 1, &StanleyControlNode::odometryCallback, this);
        
        car_cmd_pub = nh.advertise<std_msgs::String>("/car1/command", 1);
        error_pub = nh.advertise<std_msgs::Float64MultiArray>("/stanley/errors", 1);
        // Initialize PathManager
        PathManager::init(nh); 
        if (PathManager::loading_condensed_path()) {
            path_ready = true;
            ROS_INFO("Stanley Node: Path successfully loaded.");
        }

        // Control Loop at 10Hz
        timer = nh.createTimer(ros::Duration(0.1), &StanleyControlNode::controlLoop, this);

        // --- Serial Port Initialization ---
        try {
            ser.setPort("/dev/ttyACM0"); // Check your port (ttyACM0 or ttyUSB0)
            ser.setBaudrate(115200);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser.setTimeout(to);
            ser.open();
        } catch (serial::IOException& e) {
            ROS_ERROR_STREAM("Unable to open port ");
        }

        if(ser.isOpen()) {
            ROS_INFO_STREAM("Serial Port initialized");
        }
    }


    void sendSerial(double speed, double steer) {
        if (ser.isOpen()) {
            std::stringstream ss_v, ss_s;

            // Format: #speed:value;;\r\n
            ss_v << "#speed:" << std::fixed << std::setprecision(2) << speed * 400.0 << ";;\r\n";
            
            // Format: #steer:value;;\r\n
            ss_s << "#steer:" << std::fixed << std::setprecision(2) << steer * 10 << ";;\r\n";

            ser.write(ss_v.str());
            ser.write(ss_s.str());
        }
    }

    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        last_odom = *msg;
        odom_received = true;
    }

    void stopCar() {
        std_msgs::String msg;
        msg.data = "{\"action\": \"1\", \"speed\": 0.0}"; 
        car_cmd_pub.publish(msg);
        msg.data = "{\"action\": \"2\", \"steerAngle\": 0.0}";
        car_cmd_pub.publish(msg);
        sendSerial(0.0, 0.0);
    }

    void controlLoop(const ros::TimerEvent&) {
        if (!path_ready || !odom_received) return;

        // 1. Extract State from Filtered Odometry
        // nav_msgs/Odometry has pose.pose...
        double x = last_odom.pose.pose.position.x;
        double y = last_odom.pose.pose.position.y;
        double yaw_v = tf2::getYaw(last_odom.pose.pose.orientation);
        
        // Use velocity from EKF if available, otherwise use target
        double v = std::max(0.1, last_odom.twist.twist.linear.x); 
        //  double v = 0.25; // Target velocity

        // 2. Find Waypoint
        Eigen::Vector3d current_state(x , y, yaw_v);
        int target_idx;
        PathManager::find_next_waypoint(target_idx, current_state);

        // 3. Stanley Tracking Point (Front Axle)
        // Stanley works best when tracking from the front axle
        
        double tx = x + wheelbase * cos(yaw_v);
        double ty = y + wheelbase * sin(yaw_v);

        // 4. End Condition
        double dist_to_end_sq = pow(PathManager::state_refs.bottomRows(1)(0,0) - x, 2) + 
                                pow(PathManager::state_refs.bottomRows(1)(0,1) - y, 2);
        
        if (target_idx > PathManager::state_refs.rows() - 1 || dist_to_end_sq < 0.01) {
            ROS_INFO("Goal Reached. Stopping.");
            stopCar();
            return; 
        }

        // 5. Reference Pose
        double ref_x   = PathManager::state_refs(target_idx, 0);
        double ref_y   = PathManager::state_refs(target_idx, 1);
        double ref_yaw = PathManager::state_refs(target_idx, 2);

        // 6. Stanley Errors
        // Heading Error (normalized)
        double angError = ref_yaw - yaw_v;
        angError = -atan2(sin(angError), cos(angError));

        // Cross-track Error (e)
        // Calculated relative to the front axle (tx, ty)
        double dx = tx - ref_x;
        double dy = ty - ref_y;
        
        // Predict the cross track error based on the heading
        double posError = -(dx * sin(ref_yaw) - dy * cos(ref_yaw));

        // 7. Control Law: delta = phi + atan(k*e / v)
        double delta = angError + atan2(k_gain * posError, k_soft + v);

        // 8. Output Formatting
        double steerDeg = std::clamp(delta * 180.0 / M_PI, -max_steer, max_steer);
        

        // 9. Publish
        std_msgs::String speed_msg, steer_msg;
        std::stringstream ss_v, ss_s;
        
        ss_v << "{\"action\": \"1\", \"speed\": " << std::fixed << std::setprecision(2) << PathManager::v_ref << "}";
        speed_msg.data = ss_v.str();
        car_cmd_pub.publish(speed_msg);

        ss_s << "{\"action\": \"2\", \"steerAngle\": " << std::fixed << std::setprecision(2) << steerDeg << "}";
        steer_msg.data = ss_s.str();
        car_cmd_pub.publish(steer_msg);
        sendSerial( PathManager::v_ref, steerDeg);
        // // In controlLoop after calculating errors:
        // std_msgs::Float64MultiArray err_msg;
        // err_msg.data.resize(3);
        // err_msg.data.push_back(posError);
        // err_msg.data.push_back(angError * 180.0 / M_PI); // Plotting degrees is easier to read
        // err_msg.data.push_back(steerDeg);
        // error_pub.publish(err_msg); 

        // Instead of stringstreams, use your Utility method:
        // utils->publish_cmd_vel(steerDeg, PathManager::v_ref, true);



        // ROS_INFO("Control Sent [ID: %d] -> Speed: %.2f, Steer: %.2f | Errors -> Pos: %.2f, Ang: %.2f deg", 
        //   target_idx, PathManager::v_ref, steerDeg, posError, angError * 180.0 / M_PI);
    }
};

int main(int argc, char** argv) {
   
    ros::init(argc, argv, "stanley_controller");
    StanleyControlNode node;
    ros::spin();
    return 0;
}