#!/usr/bin/env python

import numpy as np
from numpy import linalg as LA

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, SetBool, SetBoolRequest
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState

from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from IPython import embed

from moveit_msgs.msg import MoveItErrorCodes
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name

class StateMachine(object):
    def __init__(self):

        self.node_name = "Student SM"

        # Access rosparams
        self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.pick_srv_nm = rospy.get_param(rospy.get_name() + '/pick_srv')
        # Added Hector
        self.place_srv_nm = rospy.get_param(rospy.get_name() + '/place_srv')

        #self.aruco_pose_top = rospy.get_param(rospy.get_name() + '/marker_pose_topic')
        self.cube_pose = rospy.get_param(rospy.get_name() + '/cube_pose')

        rospy.loginfo("%s: ...A...", self.node_name)
        # Subscribe to topics

        # Wait for service providers
        rospy.wait_for_service(self.mv_head_srv_nm, timeout=30)
        # ADDED HECTOR
        rospy.wait_for_service(self.pick_srv_nm, timeout=30)

        # Instantiate publishers
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)
        # ADDED BY HECTOR
        #self.cmd_vel_pub = rospy.Publisher(self.aruco_pose_top, PoseStamped, queue_size=10)
		#self.pick_gui = rospy.Publisher(self.pick_srv_nm, SetBool, queue_size=10)
        rospy.loginfo("%s: ...B...", self.node_name)


        # Set up action clients
        rospy.loginfo("%s: Waiting for play_motion action server...", self.node_name)
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)
        if not self.play_motion_ac.wait_for_server(rospy.Duration(1000)):
            rospy.logerr("%s: Could not connect to /play_motion action server", self.node_name)
            exit()
        rospy.loginfo("%s: Connected to play_motion action server", self.node_name)

        # Init state machine
        self.state = 0
        rospy.sleep(3)
        self.check_states()


    def check_states(self):

        while not rospy.is_shutdown() and self.state != 6:

            # State 0: Move the robot "manually" to door
            # We change state 0 because we are already in front of the table

            # State 0:  Tuck arm
            if self.state == 0:
                rospy.loginfo("%s: Tucking the arm...", self.node_name)
                goal = PlayMotionGoal()
                goal.motion_name = "home"
                goal.skip_planning = True
                self.play_motion_ac.send_goal(goal)
                fail_tucking = self.play_motion_ac.wait_for_result(rospy.Duration(10.0))

                if fail_tucking:
                    self.play_motion_ac.cancel_goal()
                    rospy.logerr("%s: play_motion failed to tuck arm, reset simulation", self.node_name)
                    self.state = 5
                else:
                    rospy.loginfo("%s: Arm pre-grasped.", self.node_name)
                    self.state = 1

                rospy.sleep(1)

            # State 1: Move head down
            if self.state == 1:
                try:
                    rospy.loginfo("%s: Lowering robot head", self.node_name)
                    move_head_srv = rospy.ServiceProxy(self.mv_head_srv_nm, MoveHead)
                    move_head_req = move_head_srv("down")

                    if move_head_req.success == True:
                        self.state = 2
                        rospy.loginfo("%s: Move head down succeded!", self.node_name)
                    else:
                        rospy.loginfo("%s: Move head down failed!", self.node_name)
                        self.state = 6

                    rospy.sleep(3)

                except rospy.ServiceException, e:
                    print "Service call to move_head server failed: %s"%e

            # State 1: Pick cube
            if self.state == 2:
                rospy.loginfo("%s: Picking the cube...", self.node_name)
                pick_the_cube = rospy.ServiceProxy(self.pick_srv_nm, SetBool)

                pick_cube_req = pick_the_cube(True)

                if pick_cube_req.success == True:
                    self.state = 3
                    rospy.loginfo("%s: Pick cube succeded!", self.node_name)
                else:
                    rospy.loginfo("%s: Pick cube failed!", self.node_name)
                    self.state = 6

                rospy.sleep(1)

            # State 3:  Move the robot "manually" to chair
            if self.state == 3:
                move_msg = Twist()
                move_msg.angular.z = -1

                rate = rospy.Rate(10)
                converged = False
                cnt = 0
                rospy.loginfo("%s: Moving towards table", self.node_name)
                while not rospy.is_shutdown() and cnt < 31:
                    self.cmd_vel_pub.publish(move_msg)
                    rate.sleep()
                    cnt = cnt + 1

                move_msg.linear.x = 1
                move_msg.angular.z = 0
                cnt = 0
                while not rospy.is_shutdown() and cnt < 10:
                    self.cmd_vel_pub.publish(move_msg)
                    rate.sleep()
                    cnt = cnt + 1

                self.state = 4
                rospy.sleep(1)

            # State 4:  Lower robot head service
            if self.state == 4:
            	try:
                    rospy.loginfo("%s: Lowering robot head", self.node_name)
                    move_head_srv = rospy.ServiceProxy(self.mv_head_srv_nm, MoveHead)
                    move_head_req = move_head_srv("down")

                    if move_head_req.success == True:
                        self.state = 5
                        rospy.loginfo("%s: Move head down succeded!", self.node_name)
                    else:
                        rospy.loginfo("%s: Move head down failed!", self.node_name)
                        self.state = 6

                    rospy.sleep(3)

                except rospy.ServiceException, e:
                    print "Service call to move_head server failed: %s"%e

            # State 5: Leave the cube
            if self.state == 5:
                rospy.loginfo("%s: Leaving the cube...", self.node_name)
                place_the_cube = rospy.ServiceProxy(self.place_srv_nm, SetBool)

                place_cube_req = place_the_cube(True)

                if place_cube_req.success == True:
                    rospy.loginfo("%s: Place cube succeded! Job done", self.node_name)
                    return

                else:
                    rospy.loginfo("%s: Place cube failed!", self.node_name)
                    self.state = 6

                rospy.sleep(1)


            # Error handling
            if self.state == 6:
                rospy.logerr("%s: State machine failed. Check your code and try again!", self.node_name)
                return

        rospy.loginfo("%s: State machine finished!", self.node_name)
        return


if __name__ == "__main__":

    rospy.init_node('main_state_machine')
    try:
        StateMachine()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
