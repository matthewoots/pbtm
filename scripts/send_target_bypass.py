#!/usr/bin/env python3

import sys
import rospy
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory

# launch this with
# python send_target_bypass.py idx x y z

def bypass(jt, cmd_3d, size):
    
    # for i in range(size):
    jtp = JointTrajectoryPoint()
    jtp.positions.append(cmd_3d[0])
    jtp.positions.append(cmd_3d[1])
    jtp.positions.append(cmd_3d[2])
    # 4 is bypass
    jtp.time_from_start = rospy.Duration(4.0)
    jt.points.append(jtp)

    # return jt

def bypass_publisher():
    # len_cmds = len(sys.argv) - 1
    cmd_3d = []
    for i in range(2,len(sys.argv)):
        cmd_3d.append((float)(sys.argv[i]))

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

    jt = JointTrajectory()
    jt.header.stamp = rospy.Time.now()
    drone_name = 'drone'+sys.argv[1]
    jt.joint_names.append(drone_name)

    bypass(jt, cmd_3d, 1)

    pub.publish(jt)

    print('completed sending target bypass')

if __name__ == '__main__':
    bypass_publisher()