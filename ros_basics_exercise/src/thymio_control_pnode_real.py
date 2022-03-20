#!/usr/bin/env python3

import math
import rospy
from ros_basics_msgs.msg import SimplePoseStamped
from ros_basics_msgs.msg import SimpleVelocities
from ros_basics_msgs.srv import CurrentWaypoint
from ros_basics_msgs.srv import RemoveWaypoint
from ros_basics_msgs.srv import SetWaypoints
from ros_basics_msgs.srv import CheckWaypointReached
from geometry_msgs.msg import Pose2D
from ros_basics_msgs.msg import ProximitySensors

robot_pose = None
error_prev = [0, 0]
F = 10
file = None

STATE_FREE = 0
STATE_OBS = 1
state = 0
TIME = 5
timer = 0
max_v = 0.07
max_w = 2
sensor_threshold = 3500

"""
def check_waypoint():
    global robot_pose
    current_goal = call_current_waypoint()
    rospy.wait_for_service('remove_waypoint')

    pos_x = robot_pose.pose.xyz.x
    pos_y = robot_pose.pose.xyz.y
    goal_x = current_goal.goal.x
    goal_y = current_goal.goal.y
    dist = math.sqrt( ((goal_x-pos_x)**2) + ((goal_y-pos_y)**2) )

    if dist < 0.04:
        try:
            rm_waypoint = rospy.ServiceProxy('remove_waypoint', RemoveWaypoint)
            resp = rm_waypoint(0)
        except rospy.ServiceException as e:
            print("check_waypoint: Service call failed: %s"%e)
"""

def check_waypoint():
    global robot_pose
    rospy.wait_for_service('check_waypoint_reached')
    try:
        chk_wpt_reached = rospy.ServiceProxy('check_waypoint_reached', CheckWaypointReached)
        resp = chk_wpt_reached(robot_pose, True)
    except rospy.ServiceException as e:
        print("check_waypoint: Service call failed: %s"%e)


def myCallback(_data):
    global robot_pose, file
    rospy.loginfo(_data.pose.xyz)
    robot_pose = _data
    file.write(str(robot_pose.pose.xyz.x) + ", ")
    file.write(str(robot_pose.pose.xyz.y) + ", ")
    file.write(str(robot_pose.pose.rpy.yaw) + "\n")

def callSensors(_data):
    global state, STATE_OBS, timer, sensor_threshold
    proximity_sensors = _data
    list_sens = proximity_sensors.values
    rospy.loginfo("sensors : %s",list_sens)
    max_val = 0
    idx_max = 0
    for i in range (len(list_sens)):
        if list_sens[i] > max_val:
            max_val = list_sens[i]
            idx_max = i
    if max_val >= sensor_threshold:
        state = STATE_OBS
        timer = 0
        front_speed = 0.035
        rotate_speed = 2
        #front_left_most
        if idx_max == 0:
            talker(-front_speed, -rotate_speed)
        #front_left
        elif idx_max == 1:
            talker(-front_speed, -rotate_speed/2)
        #front_middle
        elif idx_max == 2:
            talker(-front_speed, -rotate_speed/2)
        #front_right
        elif idx_max == 3:
            talker(-front_speed, rotate_speed/2)
        #front_right_most
        elif idx_max == 4:
            talker(-front_speed, rotate_speed)
        #back_right
        elif idx_max == 5:
            talker(front_speed, rotate_speed/2)
        #back_left
        elif idx_max == 6:
            talker(front_speed, -rotate_speed/2)
        

def listener():
    rospy.Subscriber('robot_pose', SimplePoseStamped, myCallback)
    rospy.Subscriber('proximity_sensors', ProximitySensors, callSensors)


def call_current_waypoint():
    rospy.wait_for_service('current_waypoint')
    try:
        get_cur_waypt = rospy.ServiceProxy('current_waypoint', CurrentWaypoint)
        resp = get_cur_waypt()
    except rospy.ServiceException as e:
        print("Service call failed: %s" %e)
    return resp


def talker(v, w):
    global max_v, max_w
    if abs(v) > max_v:
        v = max_v * (v/abs(v))
    if abs(w) > max_w:
        w = max_w * (w/abs(w))
    pub = rospy.Publisher('set_velocities', SimpleVelocities)
    nVel = SimpleVelocities(v, w)
    rospy.loginfo(nVel)
    pub.publish(nVel)


def check_end():
    rospy.wait_for_service('current_waypoint')
    try:
        get_cur_waypt = rospy.ServiceProxy('current_waypoint', CurrentWaypoint)
        resp = get_cur_waypt()
        return resp.is_empty
    except rospy.ServiceException as e:
        print("Service call failed: %s" %e)

def control(current_goal):
    global robot_pose, error_prev, F
    kp_fwd = 0.8
    kp_ang = 5
    kd_fwd = 0.01 * F
    kd_ang = 0.01 * F
    pos_x = robot_pose.pose.xyz.x
    pos_y = robot_pose.pose.xyz.y
    pos_th = robot_pose.pose.rpy.yaw
    goal_x = current_goal.goal.x
    goal_y = current_goal.goal.y
    
    goal_th = math.atan2((goal_y-pos_y), (goal_x-pos_x)+0.0000001)
    
    dist = math.sqrt( ((goal_x-pos_x)**2) + ((goal_y-pos_y)**2) )

    angle = goal_th - pos_th
    if angle > math.pi:
        angle -= 2*math.pi
    elif angle < -math.pi:
        angle += 2*math.pi

    speed_fwd = kp_fwd * dist + kd_fwd * (dist - error_prev[0])
    speed_ang = kp_ang * angle + kd_ang * (angle - error_prev[1])
    error_prev = [dist, angle]
    return float(speed_fwd), float(speed_ang)


def spin():
    global state, timer, TIME, STATE_FREE, STATE_OBS
    if not check_end() :
        if state == STATE_FREE:
            # 1) call the corresponding service to check if the waypoint was reached
            check_waypoint()

            # 2) call the corresponding service to get the current waypoint or waypoint list
            current_goal = call_current_waypoint()

            # 3) implement your PID/PD/P logic
            v, w = control(current_goal)

            # 4) publish your computed velocities in the set_velocities topic
            talker(v, w)

        # if there is an obstacle avoids it during TIME loops
        else :
            timer += 1
            if timer == TIME :
                state = STATE_FREE

    # 5) if there are no waypoints left then set the velocities to 0 and wait for the next waypoint
    else :
        talker(0, 0)



if __name__ == '__main__':
    file = open("path.txt", "w")

    rospy.init_node('thymio_control_pnode', anonymous=True)
    robot_pose = SimplePoseStamped()

    # subscribe to the topics
    listener()

    loop_rate = rospy.Rate(F)
    while not rospy.is_shutdown():
       spin()
       file.flush()
       loop_rate.sleep()

