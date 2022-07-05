#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Point

pub0 = rospy.Publisher('/trajectory/points', JointTrajectory, queue_size=10)

def home(num, point):
    jt = JointTrajectory()

    jt.header.stamp = rospy.Time.now()
    idx = str(num)
    jt.joint_names.append('drone'+idx)
   
    jtp = JointTrajectoryPoint()
    jtp.positions.append(point.x)
    jtp.positions.append(point.y)
    jtp.positions.append(point.z)
    jtp.time_from_start = rospy.Duration(4.0)
    jt.points.append(jtp)

    return jt

def home_publisher():
    rospy.init_node('home_publisher', anonymous=True)
    rate_hz = 0.2
    rate = rospy.Rate(rate_hz) # 0.2hz
    start_sleep = rospy.Rate(0.5) # 0.5hz
    drone_index = 0

    point = Point()

    start_sleep.sleep()
    count = 0

    while not rospy.is_shutdown():
        count = count + 1
        if count > 1:
            break

        pub0.publish(home(drone_index, point))

        rate.sleep()
    
    print('completed sending home')


if __name__ == '__main__':
    home_publisher()