#!/usr/bin/env python3

import sys
import rospy
import math
import time
import numpy as np
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory

# launch this with
# python send_target_bypass.py idx x y z

def normalize_angle(angle):
    """
    Wrap the angle between -pi and pi.
    Args:
        angle (float): angle to wrap.
    Returns:
         The wrapped angle.
    """
    a = normalize_angle_positive(angle)
    if a > np.pi:
        a -= 2. * np.pi

    return a 

def normalize_angle_positive(angle):
    """
    Wrap the angle between 0 and 2 * pi.
    Args:
        angle (float): angle to wrap.
    Returns:
         The wrapped angle.
    """
    pi_2 = 2. * np.pi

    return np.fmod(np.fmod(angle, pi_2) + pi_2, pi_2) 

def bypass(jt, cmd_3d, size):
    
    # for i in range(size):
    jtp = JointTrajectoryPoint()
    jtp.positions.append(cmd_3d[0])
    jtp.positions.append(cmd_3d[1])
    jtp.positions.append(cmd_3d[2])
    # 4 is bypass
    jtp.time_from_start = rospy.Duration(4.0)
    jt.points.append(jtp)

class auto_publisher:
    def __init__(self):
        self.pose = Point()
        self.register_pose = []
        self.register_bearing = 0.0
        self.registered = False
        self.target = JointTrajectory()
        self.init = False
        self.counter = 0
        self.cmd = []
        self.pub = rospy.Publisher( \
            '/trajectory/points', JointTrajectory, latch=True, queue_size=20)
        self.sub = rospy.Subscriber( \
            "/drone" + sys.argv[1] + "/uav/nwu", PoseStamped, self.callback, queue_size=20)
        epoch = rospy.Time.now()
        while self.pub.get_num_connections() < 1:
            # Do nothing
            if (rospy.Time.now() - epoch).to_sec() > 5.0:
                print("No connections in 5s")
                return

    def callback(self, data):
        # print(self.pose)
        self.pose = data.pose.position
        bearing = math.atan2(self.pose.y, self.pose.x)

        if not self.registered:
            h = math.sqrt( \
                math.pow(self.pose.x, 2) + \
                math.pow(self.pose.y, 2))
            self.register_pose.append(h * math.cos(bearing))
            self.register_pose.append(h * math.sin(bearing))
            self.register_pose.append(self.pose.z)
            self.register_bearing = bearing
            self.registered = True

        if not self.init:
            self.counter += 1
            mod = self.counter % 2 
            mul = -1
            if mod == 0:
                mul = 1
            # next_bearing = \
            #     normalize_angle(self.register_bearing - math.pi)
            # h = math.sqrt( \
            #     math.pow(self.register_pose[0], 2) + \
            #     math.pow(self.register_pose[1], 2))
            
            self.target = JointTrajectory()
            self.cmd = []
            # self.cmd.append(h * math.cos(next_bearing))
            # self.cmd.append(h * math.sin(next_bearing))
            self.cmd.append(mul * self.register_pose[0])
            self.cmd.append(mul * self.register_pose[1])
            self.cmd.append(self.register_pose[2])

            drone_name = 'drone' + sys.argv[1]
            self.target.joint_names.append(drone_name)

            bypass(self.target, self.cmd, 1)

            self.pub.publish(self.target)
            print("[" + str(self.counter) + "] publish new point")
            print("[" + str(self.cmd[0]) + " " + \
                str(self.cmd[1]) + " " + \
                str(self.cmd[2]) + "]")
            self.init = True
            return
        
        distance = math.sqrt( \
        math.pow(self.cmd[0] - self.pose.x, 2) + \
        math.pow(self.cmd[1] - self.pose.y, 2) + \
        math.pow(self.cmd[2] - self.pose.z, 2))

        # print( "distance left (" + str(distance) + ")")
        
        # if (distance < 0.2):
        if (distance < 0.40):
            self.init = False
            # time.sleep(2)
            time.sleep(0.025)
            return



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
    rospy.init_node('auto_command_publisher')
    auto_publisher()
    rospy.spin()