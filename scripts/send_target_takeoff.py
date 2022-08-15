#!/usr/bin/env python3

import sys
import rospy
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Point

# launch this with
# python send_target_takeoff.py idx

def takeoff(index, point):
    jt = JointTrajectory()

    jt.header.stamp = rospy.Time.now()
    drone_name = 'drone'+str(index)
    jt.joint_names.append(drone_name)
   
    jtp = JointTrajectoryPoint()
    jtp.positions.append(point.x)
    jtp.positions.append(point.y)
    jtp.positions.append(point.z)
    # 1 is takeoff
    jtp.time_from_start = rospy.Duration(1.0)
    jt.points.append(jtp)

    return jt

def takeoff_publisher():
    rospy.init_node('takeoff_publisher'+sys.argv[1], anonymous=True)
    pub = rospy.Publisher('/trajectory/points', JointTrajectory, latch=True, queue_size=20)

    point = Point()

    start_sleep = rospy.Rate(0.5) # 0.5hz
    start_sleep.sleep()

    pub.publish(takeoff(int(sys.argv[1]), point))
    
    print('completed sending takeoff')


if __name__ == '__main__':
    takeoff_publisher()