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

#include "walker/walker.h"

/**
 * @brief      main function
 * param       integer (argc) and character array (argv)
 * @return     0
 */
int main(int argc, char* argv[]) {
    // Initializing the ros node
    ros::init(argc, argv, "roomba_robot");
    // Creating an object for the Walker class
    Walker walk;
    // Publish at a frequency of 10 Hz
    ros::Rate loop(10);
    // Run the loop until ros dies
    while (ros::ok()) {
        // Run the walker behaviour by calling runRobot() function
        walk.runRobot();
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}

