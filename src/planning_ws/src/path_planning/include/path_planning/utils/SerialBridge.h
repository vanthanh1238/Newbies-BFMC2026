#pragma once

#include <boost/asio.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <memory>
#include <thread>

namespace Sensing {

class SerialBridge {
public:
    SerialBridge(ros::NodeHandle& nh) {
        imu_pub = nh.advertise<sensor_msgs::Imu>("/car1/imu", 10);
        odom_pub = nh.advertise<nav_msgs::Odometry>("/car1/odometry", 10);

        try {
            serial = std::make_shared<boost::asio::serial_port>(io, "/dev/ttyACM0");
            serial->set_option(boost::asio::serial_port_base::baud_rate(115200));
            start_async_read();
            worker_thread = std::thread([this]() { io.run(); });
            ROS_INFO("Sensing Node: Connected to /dev/ttyACM0");
        } catch (const std::exception& e) {
            ROS_ERROR("Sensing Node: Serial Error: %s", e.what());
        }
    }

    ~SerialBridge() { io.stop(); if(worker_thread.joinable()) worker_thread.join(); }

private:
    boost::asio::io_service io;
    std::shared_ptr<boost::asio::serial_port> serial;
    std::thread worker_thread;
    ros::Publisher imu_pub, odom_pub;

    char rxBuf[1024];
    std::size_t rxLen = 0;

    void start_async_read() {
        serial->async_read_some(boost::asio::buffer(rxBuf + rxLen, 1024 - rxLen),
            [this](const boost::system::error_code& ec, std::size_t n) {
                if (!ec) { rxLen += n; scan_frames(); start_async_read(); }
            });
    }

    void scan_frames() {
        // Logic to find @5: or @7: and ending with ;;\r\n
        for (std::size_t i = 0; i + 3 < rxLen; ++i) {
            if (rxBuf[i] == '@' && rxBuf[i+2] == ':') {
                char id = rxBuf[i+1];
                for (std::size_t j = i; j + 3 < rxLen; ++j) {
                    if (rxBuf[j] == ';' && rxBuf[j+1] == ';' && rxBuf[j+2] == '\r') {
                        process_payload(id, &rxBuf[i+3], j - (i+3));
                        std::size_t consumed = j + 4;
                        std::memmove(rxBuf, rxBuf + consumed, rxLen - consumed);
                        rxLen -= consumed;
                        i = -1; break;
                    }
                }
            }
        }
    }

    void process_payload(char id, char* p, std::size_t len) {
        if (id == '5') { // Encoder
            double speed = atof(p);
            nav_msgs::Odometry msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "odom";
            msg.child_frame_id = "chassis";
            // Pose data is ignored by your EKF config, but we'll set identity to be safe
            msg.pose.pose.position.x = 0.0;
            msg.pose.pose.position.y = 0.0;
            msg.pose.pose.orientation.w = 1.0;

            // linear.x is the 7th element in the config array
            msg.twist.twist.linear.x = speed * 0.01; 
            msg.twist.twist.linear.y = 0.0; 
            msg.twist.twist.linear.z = 0.0;

            // Set Covariance (Crucial for EKF weighting)
            // Index 0 of covariance is Linear X variance
            double var = 0.05; // Example variance value
            msg.twist.covariance[0] = var;   // X velocity 
            msg.twist.covariance[7] = var;   // Y velocity variancevariance (though set to 0.0)
            msg.twist.covariance[35] = 0.01; // Small variance for Yaw rate if needed

            odom_pub.publish(msg);

        } 
        else if (id == '7') { // IMU: roll;pitch;yaw;gx;gy;gz;ax;ay;az
            double r_deg, p_deg, y_deg;
            double gx, gy, gz;
            double ax, ay, az;

            // Parse 9 values from the sequence
            int parsed = sscanf(p, "%lf;%lf;%lf;%lf;%lf;%lf;%lf;%lf;%lf", 
                                &r_deg, &p_deg, &y_deg, 
                                &gx, &gy, &gz, 
                                &ax, &ay, &az);

            if (parsed >= 9) {
                sensor_msgs::Imu msg;
                msg.header.stamp = ros::Time::now();
                msg.header.frame_id = "chassis";

                // 1. Orientation: Convert Degrees to Radians and then to Quaternion
                tf2::Quaternion q;
                q.setRPY(r_deg * M_PI / 180.0, 
                        p_deg * M_PI / 180.0, 
                        -y_deg * M_PI / 180.0); // Negation often needed for BFMC coordinate alignment
                msg.orientation = tf2::toMsg(q);

                // 2. Angular Velocity (Gyroscope)
                // Ensure your hardware sends these in rad/s. If deg/s, multiply by M_PI/180.0
                msg.angular_velocity.x = gx;
                msg.angular_velocity.y = gy;
                msg.angular_velocity.z = gz;

                // 3. Linear Acceleration (Accelerometer)
                // Ensure units are m/s^2
                msg.linear_acceleration.x = ax;
                msg.linear_acceleration.y = ay;
                msg.linear_acceleration.z = az;

                // 4. Covariance (Using your defined constants)
                msg.orientation_covariance[0] = 0.001;
                msg.orientation_covariance[4] = 0.001;
                msg.orientation_covariance[8] = 0.001;

                msg.angular_velocity_covariance[0] = 0.01;
                msg.angular_velocity_covariance[4] = 0.01;
                msg.angular_velocity_covariance[8] = 0.01;

                msg.linear_acceleration_covariance[0] = 0.01;
                msg.linear_acceleration_covariance[4] = 0.01;
                msg.linear_acceleration_covariance[8] = 0.01;

                imu_pub.publish(msg);
            }
        }
    }
};
}