#!/usr/bin/env python3

import sys
import rospy
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory

# launch this with
# python send_target.py idx x y z x y z

def waypoint(jt, cmd_3d, size):
    
    for i in range(size):
        jtp = JointTrajectoryPoint()
        jtp.positions.append(cmd_3d[i*3+0])
        jtp.positions.append(cmd_3d[i*3+1])
        jtp.positions.append(cmd_3d[i*3+2])
        # 3 is mission
        jtp.time_from_start = rospy.Duration(3.0)
        jt.points.append(jtp)

    # return jt

def waypoint_publisher():
    len_cmds = len(sys.argv) - 1
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

    waypoint(jt, cmd_3d, (int)(len_cmds/3))

    pub.publish(jt)

    print('completed sending target')

if __name__ == '__main__':
    waypoint_publisher()