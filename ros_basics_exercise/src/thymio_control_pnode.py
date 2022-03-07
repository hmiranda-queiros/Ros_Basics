#!/usr/bin/env python3


import math
import rospy
from ros_basics_msgs.msg import SimplePoseStamped
from ros_basics_msgs.msg import SimpleVelocities
from ros_basics_msgs.srv import CurrentWaypoint
from ros_basics_msgs.srv import CheckWaypointReached
from ros_basics_msgs.srv import SetWaypoints
from geometry_msgs.msg import Pose2D

robot_pose = None
error_prev = [0, 0]
F = 10

def check_waypoint():
    global robot_pose
    rospy.wait_for_service('check_waypoint_reached')
    try:
        chk_wpt_reached = rospy.ServiceProxy('check_waypoint_reached', CheckWaypointReached)
        resp = chk_wpt_reached(robot_pose, True)
    except rospy.ServiceException as e:
        print("check_waypoint: Service call failed: %s"%e)


def myCallback(_data):
    global robot_pose
    rospy.loginfo(_data.pose.xyz)
    robot_pose = _data


def listener():
    rospy.Subscriber('robot_pose', SimplePoseStamped, myCallback)


def call_current_waypoint():
    rospy.wait_for_service('current_waypoint')
    try:
        get_cur_waypt = rospy.ServiceProxy('current_waypoint', CurrentWaypoint)
        resp = get_cur_waypt()
    except rospy.ServiceException as e:
        print("Service call failed: %s" %e)
    return resp


def path_to_follow():
    x_w = [0.10021, 0.20644, 0.17237, 0.09320, 0.0, 0.0010, -0.08418, 0.0010, -0.16034, -0.16034]
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
    pub = rospy.Publisher('set_velocities', SimpleVelocities)
    nVel = SimpleVelocities(v, w)
    rospy.loginfo(nVel)
    pub.publish(nVel)

def check_end():
    rospy.wait_for_service('current_waypoint')
    try:
        get_cur_waypt = rospy.ServiceProxy('current_waypoint', CurrentWaypoint)
        _, is_empty = get_cur_waypt()
        if is_empty:
            talker(0, 0)
    except rospy.ServiceException as e:
        print("Service call failed: %s" %e)


def control(current_goal):
    global robot_pose, error_prev, F
    kp_fwd = 1
    kp_ang = 1
    kd_fwd = 0.01 * F
    kd_ang = 0.01 * F
    pos_x = robot_pose.pose.xyz.x
    pos_y = robot_pose.pose.xyz.y
    pos_th = robot_pose.pose.rpy.yaw
    goal_x = current_goal.goal.x
    goal_y = current_goal.goal.y
    
    goal_th = math.atan2((goal_y-pos_y), (goal_x-pos_x)+0.0000001)
    if goal_x - pos_x < 0:
        goal_th = goal_th - math.pi
    
    # if goal_th > math.pi:
    #     goal_th = goal_th - 2*math.pi
    # if goal_th < -math.pi:
    #     goal_th = goal_th + 2*math.pi
    dist = math.sqrt( ((goal_x-pos_x)**2) + ((goal_y-pos_y)**2) )
    angle = goal_th - pos_th
    speed_fwd = kp_fwd * dist + kd_fwd * (dist - error_prev[0])
    speed_ang = kp_ang * angle + kd_fwd * (angle - error_prev[1])
    error_prev = [dist, angle]
    return float(speed_fwd), float(speed_ang)


def call_sensors():
    pass


def spin():
    # 1) call the corresponding service to check if the waypoint was reached
    check_waypoint()

    # 2) subscribe to the robot_pose topic to get the robot's current pose
    

    # 3) call the corresponding service to get the current waypoint or waypoint list
    current_goal = call_current_waypoint()

    # 4) implement your PID/PD/P logic
    v, w = control(current_goal)

    # 5) publish your computed velocities in the set_velocities topic
    talker(v, w)

    call_sensors()
    # 6) if there are no waypoints left then set the velocities to 0 and wait for the next waypoint
    # check_end()



if __name__ == '__main__':
    rospy.init_node('thymio_control_pnode', anonymous=True)
    robot_pose = SimplePoseStamped()
    robot_pose.pose.xyz.x = 0
    robot_pose.pose.xyz.y = 0
    path_to_follow()
    listener()
    loop_rate = rospy.Rate(F)
    while not rospy.is_shutdown():
       spin()
       loop_rate.sleep()






"""
sPose = SimplePoseStamped()
sPose.pose.xyz.x = 0.0
sPose.pose.xyz.y = 0.0
print(sPose.pose)

def myCallback(_data):
    rospy.loginfo(_data.pose.xyz)


def listener():
    rospy.init_node('teste_node_listener', anonymous=True)
    rospy.Subscriber('robot_pose', SimplePoseStamped, myCallback)
    rospy.spin()

def talker():
    rospy.init_node('teste_node_publisher', anonymous=True)
    pub = rospy.Publisher('set_velocities', SimpleVelocities)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        nVel = SimpleVelocities(-0.05,-0.05)
        rospy.loginfo(nVel)
        pub.publish(nVel)
        rate.sleep()

    rospy.spin()

rospy.init_node('test_node')

def path_to_follow():
    x_w = [0.10021, 0.20644, 0.17237, 0.09320, 0.0, 0.0010, -0.08418, 0.0010, -0.16034, -0.16034]
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

path_to_follow()

loop_rate = rospy.Rate(10)

rospy.wait_for_service('current_waypoint')

try:
    get_cur_waypt = rospy.ServiceProxy('current_waypoint', CurrentWaypoint)
    resp = get_cur_waypt()
    print(resp, type(resp))
    print(resp.goal.x, resp.goal.y)
    print("rrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr")
except rospy.ServiceException as e:
    print("Service call failed: %s" % e)
"""