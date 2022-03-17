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

STATE_FREE = 0
STATE_OBS = 1
state = 0
TIME = 5
timer = 0
max_v = 0.04
max_w = 1
sensor_threshold = 0.03


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
"""

def myCallback(_data):
    global robot_pose
    rospy.loginfo(_data.pose.xyz)
    robot_pose = _data

def callSensors(_data):
    global state, STATE_OBS, timer, sensor_threshold
    proximity_sensors = _data
    list_sens = proximity_sensors.values
    rospy.loginfo("sensors : %s",list_sens)
    min_val = 10
    idx_min = 0
    for i in range (len(list_sens)):
        if list_sens[i] < min_val:
            min_val = list_sens[i]
            idx_min = i
    if min_val <= sensor_threshold:
        state = STATE_OBS
        timer = 0
        front_speed = 0.04
        rotate_speed = 0.7
        #front_left_most
        if idx_min == 0:
            talker(-front_speed, -rotate_speed)
        #front_left
        elif idx_min == 1:
            talker(-front_speed, -rotate_speed/2)
        #front_middle
        elif idx_min == 2:
            talker(-front_speed, 0)
        #front_right
        elif idx_min == 3:
            talker(-front_speed, rotate_speed/2)
        #front_right_most
        elif idx_min == 4:
            talker(-front_speed, rotate_speed)
        #back_right
        elif idx_min == 5:
            talker(front_speed, rotate_speed/2)
        #back_left
        elif idx_min == 6:
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


def path_to_follow():
    x_w = [0.10021, 0.17, 0.17237, 0.09320, 0.0, 0.0010, -0.08418, 0.0010, -0.16034, -0.16034]
    y_w = [0.0030, 0.0030, 0.08017, 0.11825, 0.11725, 0.0010, -0.11825, -0.11624, -0.06814, 0.05110, 0.12126]
    pose_l = []
    for i, x in enumerate(x_w):
        p = Pose2D()
        p.x = x_w[i]
        p.y = y_w[i]
        pose_l.append(p)
    rospy.wait_for_service('set_waypoints')
    try:
        set_wpt = rospy.ServiceProxy('set_waypoints', SetWaypoints)
        resp = set_wpt(pose_l)
    except rospy.ServiceException as e:
        print("path_to_follow: Service call failed: %s"%e)


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
    kp_ang = 1.5
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
    speed_ang = kp_ang * angle + kd_fwd * (angle - error_prev[1])
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

    # 6) if there are no waypoints left then set the velocities to 0 and wait for the next waypoint
    else :
        talker(0, 0)



if __name__ == '__main__':
    rospy.init_node('thymio_control_pnode', anonymous=True)
    robot_pose = SimplePoseStamped()
    
    # sets the waypoints of the path to follow
    path_to_follow()

    # subscribe to the topics
    listener()

    loop_rate = rospy.Rate(F)
    while not rospy.is_shutdown():
       spin()
       loop_rate.sleep()

