#!/usr/bin/env python3


import rospy
from ros_basics_msgs.msg import SimplePoseStamped
from ros_basics_msgs.msg import SimpleVelocities
from ros_basics_msgs.srv import CurrentWaypoint
from ros_basics_msgs.srv import CheckWaypointReached
from ros_basics_msgs.srv import SetWaypoints
from geometry_msgs.msg import Pose2D


"""
def check_waypoint(x, y):
    sPose = SimplePoseStamped()
    sPose.pose.xyz.x = x
    sPose.pose.xyz.y = y
    rospy.wait_for_service('check_waypoint_reached')
    try:
        chk_wpt_reached = rospy.ServiceProxy('check_waypoint_reached', CheckWaypointReached)
        resp = chk_wpt_reached(sPose, True)
    except rospy.ServiceException as e:
        print("check_waypoint: Service call failed: %s"%e)


def myCallback(_data):
    rospy.loginfo(_data.pose.xyz)


def listener():
    rospy.Subscriber('robot_pose', SimplePoseStamped, myCallback)


def call_current_waypoint():
    rospy.wait_for_service('current_waypoint')
    try:
        get_cur_waypt = rospy.ServiceProxy('current_waypoint', CurrentWaypoint)
        resp = get_cur_waypt()
        return resp.goal.x, resp.goal.y
    except rospy.ServiceException as e:
        print("Service call failed: %s" %e)
        talker(0, 0)


def path_to_follow():
    #x_w = [0.10021, 0.20644, 0.17237, 0.09320, 0.0, 0.0010, -0.08418, 0.0010, -0.16034, -0.16034]
    #y_w = [0.0030, 0.0030, 0.08017, 0.11825, 0.11725, 0.0010, -0.11825, -0.11624, -0.06814, 0.05110, 0.12126]
    x_w = [0, 0, 0, 0]
    y_w = [0, 0, 0, 0]
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


def talker(vx, vy):
    pub = rospy.Publisher('set_velocities', SimpleVelocities)
    nVel = SimpleVelocities(vx, vy)
    rospy.loginfo(nVel)
    pub.publish(nVel)


def spin(x, y):
    # 1) call the corresponding service to check if the waypoint was reached
    check_waypoint(x, y)

    # 2) subscribe to the robot_pose topic to get the robot's current pose
    listener()
    
    # 3) call the corresponding service to get the current waypoint or waypoint list
    x, y = call_current_waypoint()

    # 4) implement your PID/PD/P logic
    # path_to_follow()

    # 5) publish your computed velocities in the set_velocities topic
    talker(-0.05, -0.05)

    # 6) if there are no waypoints left then set the velocities to 0 and wait for the next waypoint
    call_current_waypoint()

    return x, y


if __name__ == '__main__':
    rospy.init_node('thymio_control_pnode', anonymous=True)
    path_to_follow()
    #x = 0
    #y = 0
    #loop_rate = rospy.Rate(10)
    #while not rospy.is_shutdown():
       #x, y = spin(x, y)
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

if __name__ == '__main__':
    print("*******************************************")
    print("Testing subscribing to robot pose pusblisher")
    listener()