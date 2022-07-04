/*
* helper.h
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

#ifndef HELPER_H
#define HELPER_H

#include <iostream>
#include <string>
#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>
#include <Eigen/Dense>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>

using namespace Eigen;
using namespace std;

namespace helper
{
    // *** Helper functions ***

    Eigen::Vector3d rotate_vector(
        Eigen::Vector3d rotation, Eigen::Vector3d translation)
    {
        // https://github.com/felipepolido/EigenExamples
        // for affine3d examples
        double deg2rad = - 1.0 / 180.0 * M_PI;

        Eigen::Quaterniond q;
        q = AngleAxisd(rotation.x() * deg2rad, Eigen::Vector3d::UnitX())
            * AngleAxisd(rotation.y() * deg2rad, Eigen::Vector3d::UnitY())
            * AngleAxisd(rotation.z() * deg2rad, Eigen::Vector3d::UnitZ());
        
        // w,x,y,z
        Eigen::Quaterniond rot(q.w(), q.x(), q.y(), q.z());
        rot.normalize();

        Eigen::Quaterniond p;
        p.w() = 0;
        p.vec() = - translation;
        Eigen::Quaterniond rotatedP = rot * p * rot.inverse(); 
        
        return rotatedP.vec();
    }

    Eigen::Vector3d euler_rpy(Eigen::Matrix3d R)
    {
        Eigen::Vector3d euler_out;
        // Each vector is a row of the matrix
        Eigen::Vector3d m_el[3];
        m_el[0] = Vector3d(R(0,0), R(0,1), R(0,2));
        m_el[1] = Vector3d(R(1,0), R(1,1), R(1,2));
        m_el[2] = Vector3d(R(2,0), R(2,1), R(2,2));

        // Check that pitch is not at a singularity
        if (abs(m_el[2].x()) >= 1)
        {
            euler_out.z() = 0;

            // From difference of angles formula
            double delta = atan2(m_el[2].y(),m_el[2].z());
            if (m_el[2].x() < 0)  //gimbal locked down
            {
                euler_out.y() = M_PI / 2.0;
                euler_out.x() = delta;
            }
            else // gimbal locked up
            {
                euler_out.y() = -M_PI / 2.0;
                euler_out.x() = delta;
            }
        }
        else
        {
            euler_out.y() = - asin(m_el[2].x());

            euler_out.x() = atan2(m_el[2].y()/cos(euler_out.y()), 
                m_el[2].z()/cos(euler_out.y()));

            euler_out.z() = atan2(m_el[1].x()/cos(euler_out.y()), 
                m_el[0].x()/cos(euler_out.y()));
        }

        return euler_out;
    }

    geometry_msgs::Point vector_to_point(Eigen::Vector3d v)
    {
        geometry_msgs::Point tmp;
        tmp.x = v.x(); 
        tmp.y = v.y(); 
        tmp.z = v.z();

        return tmp;
    }

    Eigen::Vector3d point_to_vector(geometry_msgs::Point p)
    {
        Eigen::Vector3d tmp;
        tmp.x() = p.x; 
        tmp.y() = p.y; 
        tmp.z() = p.z;

        return tmp;
    }

    geometry_msgs::Vector3 vector_to_ros_vector(Eigen::Vector3d v)
    {
        geometry_msgs::Vector3 tmp;
        tmp.x = v.x(); 
        tmp.y = v.y(); 
        tmp.z = v.z();

        return tmp;
    }

    Eigen::Vector3d ros_vector_to_vector(geometry_msgs::Vector3 rv)
    {
        Eigen::Vector3d tmp;
        tmp.x() = rv.x; 
        tmp.y() = rv.y; 
        tmp.z() = rv.z;

        return tmp;
    }

    geometry_msgs::Quaternion quaternion_to_orientation(
        Eigen::Quaterniond q)
    {
        geometry_msgs::Quaternion tmp;
        tmp.x = q.x(); 
        tmp.y = q.y(); 
        tmp.z = q.z();
        tmp.w = q.w();

        return tmp;
    }

    Eigen::Quaterniond orientation_to_quaternion(
        geometry_msgs::Quaternion q)
    {
        Eigen::Quaterniond tmp;
        tmp.x() = q.x; 
        tmp.y() = q.y; 
        tmp.z() = q.z;
        tmp.w() = q.w;

        return tmp;
    }

    double constrain_between_180(double x)
    {
        x = fmod(x + M_PI,2*M_PI);
        if (x < 0)
            x += 2*M_PI;
        return x - M_PI;
    }

    double constrain_between_90(double x)
    {
        x = fmod(x + M_PI_2,M_PI);
        if (x < 0)
            x += M_PI;
        return x - M_PI_2;
    }

    Eigen::Affine3d posestamped_to_affine(geometry_msgs::PoseStamped ps)
    {
        Eigen::Affine3d output = Eigen::Affine3d::Identity();
        Eigen::Quaterniond orientation = Eigen::Quaterniond(
            ps.pose.orientation.w,
            ps.pose.orientation.x,
            ps.pose.orientation.y,
            ps.pose.orientation.z
        );
        Eigen::Vector3d position = Eigen::Vector3d(
            ps.pose.position.x,
            ps.pose.position.y,
            ps.pose.position.z
        );
        output.translation() = position;
        output.linear() = orientation.toRotationMatrix();

        return output;
    }

    Eigen::Quaterniond calculate_quadcopter_orientation(
        Eigen::Vector3d acc, double yaw_rad)
    {
        Eigen::Vector3d alpha = acc + Eigen::Vector3d(0,0,9.81);
        Eigen::Vector3d xC(cos(yaw_rad), sin(yaw_rad), 0);
        Eigen::Vector3d yC(-sin(yaw_rad), cos(yaw_rad), 0);
        Eigen::Vector3d xB = (yC.cross(alpha)).normalized();
        Eigen::Vector3d yB = (alpha.cross(xB)).normalized();
        Eigen::Vector3d zB = xB.cross(yB);

        Eigen::Matrix3d R;
        R.col(0) = xB;
        R.col(1) = yB;
        R.col(2) = zB;

        Eigen::Quaterniond q(R);
        return q;
    }

    // *** End Helper functions ***
   
}

#endif