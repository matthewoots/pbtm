/*
* main.cpp
*
* ---------------------------------------------------------------------
* Created by Matthew (matthewoots@gmail.com) in 2022
*
*  This program is free software; you can redistribute it and/or
*  modify it under the terms of the GNU General Public License
*  as published by the Free Software Foundation; either version 2
*  of the License, or (at your option) any later version.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
* ---------------------------------------------------------------------
*/

#include <iostream>
#include <ros/ros.h>
#include <pbtm.h>

int main (int argc, char** argv) 
{        
    ros::init (argc, argv, "pbtm_node");
    ros::NodeHandle nh("~");
    pbtm_class pbtm_class(nh);
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
    return 0;

}