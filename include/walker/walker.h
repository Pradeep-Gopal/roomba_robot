/**
 *@file main.cpp
 *@author Pradeep Gopal
 *@copyright MIT License
 *@brief This program will run the walker
 */

/**
 *  MIT License
 *
 *  Copyright (c) 2020 Pradeep Gopal
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

#ifndef INCLUDE_WALKER_H_
#define INCLUDE_WALKER_H_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

/**
 * @brief      Walker class
 */
class Walker {
private:
    // Node handler
    ros::NodeHandle nh;
    // Publish velocities
    ros::Publisher publishVelocities;
    // Subscribe the LaserScan topic
    ros::Subscriber subscribeData;
    // Variable to store velocities
    geometry_msgs::Twist msg;
    // Variable to determine whether object is present or not
    bool obstacle;

public:
    /**
     * @brief      Constructor for Walker class
     */
    Walker();
    /**
     * @brief      Destructor for Walker class
     */
    ~Walker();
    /**
     * @brief      Callback function for LaserScan
     * @param      msg, constant pointer to store messages
     * @return     none
     */
    void laserData(const sensor_msgs::LaserScan::ConstPtr& msg);

    /**
     * @brief      Function which returns the obstacle flag
     *             The flag determines if the object is present or not
     * @param      none
     * @return     return of type bool.
     *             1 if obstacle is detected, 0 otherwise
     */
    bool obstaclePresence();
    /**
     * @brief      Logic to run the robot after detecting an object
     * @param      none
     * @return     none
     */
    void runRobot();
};

#endif
