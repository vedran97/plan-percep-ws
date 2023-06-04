/**
 * @file CurrVelListener.cpp
 * @brief This file contains the implementation of the CurrVelListener node
 * @author Vedant Ranade
*/
#include <iostream>
#include <fstream>
#include <string>
#include <termios.h>
#include <fcntl.h>
#include <cstring>
#include <unistd.h>
#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include <rosserial_terpbot/MessageParser.h>
#include <rpi-rt/rt.hpp>
#include <cmath>

/**
 * @brief Left  Motor Ticks
*/
float leftTicks = 0;

/**
 * @brief Right Motor Ticks
*/
float rightTicks = 0;

/// @brief Filter constant alpha = 0.7
static const constexpr float alpha = 0.7;
/**
 * @brief Filter Velocity
 * @param raw
 * @param filter
 * @return filtered velocity
 * @note Filter constant alpha = 0.7
*/
static inline float filterVelocity(const float raw, float& filter) {
  return filter = filter + alpha * ((raw - filter));
}
/// @brief Message to store the the current odometry data, to be published on a ros topic
geometry_msgs::Pose2D curr_odom;

/// @brief Publishes the current odometry data to a ros topic
/// @details Filters left and right motor ticks, converts them to velocity , calculates the current odometry data and publishes it to a ros topic
/// @param data This contains the current tickrate raw data
/// @param curr_odom_publisher  This is the publisher object to publish the current odometry data
void publishOdom(geometry_msgs::Pose2D& data,ros::Publisher& curr_odom_publisher){
    auto ticks = data;

    auto velX = filterVelocity(ticks.x,leftTicks);
    auto velY = filterVelocity(ticks.y,rightTicks);

    curr_odom.x = curr_odom.x + (((6.45/2)/2)* (2*3.14159/495) *(velX + velY)*cos(curr_odom.theta));
    curr_odom.y = curr_odom.y + (((6.45/2)/2)* (2*3.14159/495) *(velX + velY)*sin(curr_odom.theta));
    curr_odom.theta = curr_odom.theta + (((6.45/2)/19.2)*(2*3.14159/495) * (velY - velX));

    curr_odom.theta = atan2(sin(curr_odom.theta),cos(curr_odom.theta));


    //# Publish updated Pose2D message to '/CURR_ODOM' topic
    curr_odom_publisher.publish(curr_odom);
}

/// @brief  Main function
int main(int argc, char **argv) {
    /// @brief Set the CPU affinity and priority for the current process
    rpi_rt::rt_settings rt(rpi_rt::CPUS::CPU3, 99, 100);
    rt.applyAffinity();
    // rt.applyPriority();

    // This blocl of code opens a serial port to read the data from the arduino
    auto uart0_filestream = ::open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode
    if (uart0_filestream == -1)
    {
      //ERROR - CAN'T OPEN SERIAL PORT
      printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
      return -1;
    }

    struct termios options;
    tcgetattr(uart0_filestream, &options);
    options.c_cflag = B2000000 | CS8 | CLOCAL | CREAD;		//<Set baud rate
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(uart0_filestream, TCIFLUSH);
    tcsetattr(uart0_filestream, TCSANOW, &options);
    // block of code ends here

    /// @brief Initialize the ros node and topics
    ros::init(argc, argv, "vel_listener");
    ros::NodeHandle nh;
    ros::Publisher curr_vel_pub = nh.advertise<geometry_msgs::Pose2D>("CURR_VEL", 1000);
    ros::Publisher curr_odom_pub =  nh.advertise<geometry_msgs::Pose2D>("CURR_ODOM", 1000);

    /// Used to store current tickrate data
    geometry_msgs::Pose2D curr_vel_msg;

    /// Used to read lines from an input stream opened on a serial port
    std::string line;
    std::ifstream tty("/dev/ttyACM0");

    /// Used to store the current tickrate data
    auto currRate = terpbot::msgs::CurrentTickRate();

    if (tty.is_open()) {
      std::cout << "tty is open" << std::endl;
        while (getline(tty, line)&&ros::ok()) {
            // 11 is the length of serialized message for CurrentTickRate
            if (line.length()==11) {
                currRate.deserialize((unsigned char*)line.c_str());
                curr_vel_msg.x = currRate.leftTickRate;  
                curr_vel_msg.y = currRate.rightTickRate;  
                curr_vel_pub.publish(curr_vel_msg);
                ros::spinOnce();
                publishOdom(curr_vel_msg,curr_odom_pub);
                ros::spinOnce();
            }
        }
        tty.close();
        close(uart0_filestream);
    }else{
      close(uart0_filestream);
    }
    return 0;
}