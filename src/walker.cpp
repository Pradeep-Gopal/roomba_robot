/**
 *@file main.cpp
 *@author Pradeep Gopal
 *@copyright MIT License
 *@brief This program will run the walker
 */

/**
 *  MIT License
 *
 *  Copyright (c) 2019 Pradeep Gopal
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a
 *  copy of this software and associated documentation files (the "Software"),
 *  to deal in the Software without restriction, including without
 *  limitation the rights to use, copy, modify, merge, publish, distribute,
 *  sublicense, and/or sell copies of the Software, and to permit persons to
 *  whom the Software is furnished to do so, subject to the following
 *  conditions:
 *
 *  The above copyright notice and this permission notice shall be included
 *  in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 *  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 *  DEALINGS IN THE SOFTWARE.
 */

#include <iostream>
#include "walker/walker.h"

/**
 * @brief      Constructor of the Walker class
 */
Walker::Walker() {
    // Publishing the velocities
    publishVelocities = nh.advertise < geometry_msgs::Twist
    > ("cmd_vel", 1000);
    // Subscribing to LaserScan topic to detect obstacles
    subscribeData = nh.subscribe<sensor_msgs::LaserScan> ("/scan", 1000,
                                              &Walker::laserData, this);
    // Defining the initial linear and angular velocities
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;
    publishVelocities.publish(msg);
    obstacle = false;
}

/**
 * @brief      Destructor of the Walker class
 */
Walker::~Walker() {
    // Stop the robot at the end of the program
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;
    publishVelocities.publish(msg);
}

/**
   * @brief      Callback function for LaserScan
   * @param      msg, constant pointer to store messages
   * @return     none
   */
void Walker::laserData(const sensor_msgs::LaserScan::ConstPtr& msg) {
    for (auto i : msg->ranges) {
        if (i < 0.7) {
            obstacle = true;
            return;
        }
    }
    obstacle = false;
    return;
}

/**
 * @brief      Function which returns the obstacle flag
               The flag determines if the object is present or not
 * @param      none
 * @return     return of type bool.
 *             1 if obstacle is detected, 0 otherwise
 */
bool Walker::obstaclePresence() {
    return obstacle;
}

/**
 * @brief      Logic to run the robot after detecting an object
 * @param      none
 * @return     none
 */
void Walker::runRobot() {
    // Check if obstacle is present nearby
    if (obstaclePresence() == true) {
        ROS_INFO_STREAM("Obstacle is present and "
                        "turning the robot to avoid collision");
        // Stop the robot
        msg.linear.x = 0.0;
        // Turn the robot
        msg.angular.z = 1.0;
    } else {
        ROS_INFO_STREAM("Obstacle is not present");
        // Stop turning the robot
        msg.angular.z = 0.0;
        // Set the forward linear speed
        msg.linear.x = 0.2;
    }
    publishVelocities.publish(msg);
}
