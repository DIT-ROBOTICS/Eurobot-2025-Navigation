#!/usr/bin/env python
from circle_fit import taubinSVD
from collections import deque
import statistics
from scipy import stats
import math
import numpy as np

import rospy 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

log_freq = 100
log_cnt = 0

class RivalOdom:
    def __init__(self, x_, y_, time_):
        self.x = x_
        self.y = y_
        self.time = time_

rival_odoms = deque()
def RivalOdomCallback(data):
    cur_x = data.pose.pose.position.x
    cur_y = data.pose.pose.position.y
    while len(rival_odoms) > 0 and rospy.get_time() - rival_odoms[0].time > 0.2:
        if len(rival_odoms) < 5:
            break
        rival_odoms.popleft()

    rival_odoms.append(RivalOdom(cur_x, cur_y, rospy.get_time()))

def fit_rival_circle(circle_pub):
    while len(rival_odoms) > 0 and rospy.get_time() - rival_odoms[0].time > 0.2:
        if len(rival_odoms) < 5:
            break
        rival_odoms.popleft()

    if len(rival_odoms) < 5:
        global log_cnt
        global log_freq
        if log_cnt % log_freq == 0:
            rospy.logwarn('Not enough rival odoms')
        log_cnt += 1
        log_cnt %= log_freq
        return

    circle_msg = Twist()
    x_arr = []
    y_arr = []
    for odom in rival_odoms:
        x_arr.append(odom.x)
        y_arr.append(odom.y)

    if statistics.stdev(x_arr) == 0.0 and statistics.stdev(y_arr) == 0.0:
        circle_msg.linear.x = x_arr[-1]
        circle_msg.linear.y = y_arr[-1]
        circle_msg.linear.z = 0
        circle_msg.angular.x = 0
        circle_msg.angular.y = 0
        circle_msg.angular.z = 0
        circle_pub.publish(circle_msg)
        return 
        
    slope, intercept = np.polyfit(np.array(x_arr), np.array(y_arr), 1)

    # linear x: odom x
    # linear y: odom y
    # linear z: dir
    # angular x: slope
    # angular y: intercept
    # angular z: stderr
    half_len = min((int)(len(x_arr) * 0.8), len(x_arr) - 2)
    first_half_x = x_arr[0:half_len]
    second_half_x = x_arr[half_len:-1]
    circle_msg.linear.x = x_arr[-1]
    circle_msg.linear.y = y_arr[-1]
    circle_msg.linear.z = (1 if statistics.mean(second_half_x) - statistics.mean(first_half_x) > 0 else -1)
    circle_msg.angular.x = slope
    circle_msg.angular.y = intercept
    circle_msg.angular.z = min(max(statistics.stdev(x_arr), statistics.stdev(y_arr)), 0.24) * 25
    circle_pub.publish(circle_msg)

if __name__ == '__main__':
    rospy.init_node('fit_traj_circle', anonymous=True)
    ns_prefix = rospy.get_param('~robot_name', 'robot')

    rospy.Subscriber('/rival/final_pose', Odometry, RivalOdomCallback)
    circle_pub = rospy.Publisher('circle_fit', Twist, queue_size=1)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        fit_rival_circle(circle_pub)
        rate.sleep()