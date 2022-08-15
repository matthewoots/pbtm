#!/usr/bin/env python3

import sys
import rospy
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory

# launch this with
# python send_target.py idx x y z

def waypoint(index, x, y, z):
    jt = JointTrajectory()

    jt.header.stamp = rospy.Time.now()
    drone_name = 'drone'+str(index)
    jt.joint_names.append(drone_name)
   
    jtp = JointTrajectoryPoint()
    jtp.positions.append(x)
    jtp.positions.append(y)
    jtp.positions.append(z)
    # 3 is mission
    jtp.time_from_start = rospy.Duration(3.0)
    jt.points.append(jtp)

    return jt

def waypoint_publisher():
    rospy.init_node('waypoint_publisher'+sys.argv[1], anonymous=True)
    pub = rospy.Publisher('/trajectory/points', JointTrajectory, latch=True, queue_size=20)

    start_sleep = rospy.Rate(0.5) # 0.5hz

    start_sleep.sleep()

    epoch = rospy.Time.now()
    while pub.get_num_connections() < 1:
        # Do nothing
        if (rospy.Time.now() - epoch).to_sec() > 5.0:
            print("No connections in 5s")
            return

    pub.publish(waypoint(int(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])))

    print('completed sending target')

if __name__ == '__main__':
    waypoint_publisher()