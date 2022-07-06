#!/usr/bin/env python3

import rospy
import random
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Point

def waypoint(num, count, point):
    jt = JointTrajectory()

    jt.header.stamp = rospy.Time.now()
    idx = str(num)
    jt.joint_names.append('drone'+idx)

    for i in range(count):
        jtp = JointTrajectoryPoint()
        point1 = generate_random_point(point)
        jtp.positions.append(point1.x)
        jtp.positions.append(point1.y)
        jtp.positions.append(point1.z)
        point = point1
        if i == 0:
            jtp.time_from_start = rospy.Duration(3.0)
        jt.points.append(jtp)

    return jt

def generate_random_point(prev):
    point = Point()
    point.x = prev.x + round(random.uniform(-3, 3), 2)
    point.y = prev.y + round(random.uniform(-3, 3), 2)
    point.z = 2.0 + round(random.uniform(-0.2, 0.2), 2)

    return point

def waypoint_publisher():
    rospy.init_node('waypoint_publisher', anonymous=True)
    rate_hz = 0.1
    rate = rospy.Rate(rate_hz) # 0.08hz
    start_sleep = rospy.Rate(0.5) # 0.5hz

    point = Point()
    point.x = 0.0
    point.y = 0.0
    point.z = 2.0

    start_sleep.sleep()
    count = 0

    while not rospy.is_shutdown():

        pub0 = rospy.Publisher('/trajectory/points', JointTrajectory, queue_size=10)
        count = count + 1
        print("iteration " + str(count))

        pub0.publish(waypoint(0, 3, point))

        rate.sleep()


if __name__ == '__main__':
    waypoint_publisher()