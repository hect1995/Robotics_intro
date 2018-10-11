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

    def callback_pick_pos(self, msg):
        self.next_goal.target_pose = msg

    def callback_place_pos(self, msg):
        self.place_goal.target_pose = msg

    def pos_check_aruco_there(self, msg):
        self.time_data = msg.header.stamp.secs



    def __init__(self):

        self.node_name = "Student SM"

        # Access rosparams
        self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.pick_srv_nm = rospy.get_param(rospy.get_name() + '/pick_srv')
        # Added Hector
        self.place_srv_nm = rospy.get_param(rospy.get_name() + '/place_srv')
        self.pick_top = rospy.get_param(rospy.get_name() + '/pick_pose_topic')
        self.place_top = rospy.get_param(rospy.get_name() + '/place_pose_topic')


        self.aruco_pose_top = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')
        self.cube_pose = rospy.get_param(rospy.get_name() + '/cube_pose')
        self.localize_myself = rospy.get_param(rospy.get_name() + '/global_loc_srv')
        self.clear_cost_map = rospy.get_param(rospy.get_name() + '/clear_costmaps_srv')

        rospy.loginfo("%s: ...A...", self.node_name)
        # Subscribe to topics
        self.next_goal = MoveBaseGoal()
        self.next_goal.target_pose.header.frame_id = "map"
        self.pick_pose_sub = rospy.Subscriber(self.pick_top, PoseStamped, self.callback_pick_pos)
        self.place_goal = MoveBaseGoal()
        self.place_goal.target_pose.header.frame_id = "map"
        self.place_pose_sub = rospy.Subscriber(self.place_top, PoseStamped, self.callback_place_pos)

        self.check_coord = rospy.Subscriber(self.aruco_pose_top, PoseStamped, self.pos_check_aruco_there)


        # Wait for service providers
        rospy.wait_for_service(self.mv_head_srv_nm, timeout=30)
        # ADDED HECTORstd_srvs
        rospy.wait_for_service(self.pick_srv_nm, timeout=30)

        # Instantiate publishers
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)
        #self.cmd_vel_pub = rospy.Publisher("move_base/goal", Twist, queue_size=10)



        # Set up action clients
        rospy.loginfo("%s: Waiting for play_motion action server...", self.node_name)
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)
        if not self.play_motion_ac.wait_for_server(rospy.Duration(1000)):
            rospy.logerr("%s: Could not connect to /play_motion action server", self.node_name)
            exit()
        rospy.loginfo("%s: Connected to play_motion action server", self.node_name)

        self.move_to_dir = SimpleActionClient("/move_base", MoveBaseAction)
        if not self.move_to_dir.wait_for_server(rospy.Duration(1000)):
            rospy.logerr("%s: Could not connect to /move_base action server", self.node_name)
            exit()
        rospy.loginfo("%s: Connected to move_base action server", self.node_name)


        # Init state machine
        self.state = 0
        #rospy.sleep(1)
        self.check_states()

    def check_states(self):

        while not rospy.is_shutdown() and self.state != 6:

            # State 0:  Tuck arm
            if self.state == 0:
                rospy.loginfo("%s: Tucking the arm...", self.node_name)
                goal = PlayMotionGoal()
                goal.motion_name = 'home'
                goal.skip_planning = True
                self.play_motion_ac.send_goal(goal)
                fail_tucking = self.play_motion_ac.wait_for_result(rospy.Duration(10.0))

                if fail_tucking:
                    self.play_motion_ac.cancel_goal()
                    rospy.logerr("%s: play_motion failed to tuck arm, reset simulation", self.node_name)
                    self.state = 6
                else:
                    rospy.loginfo("%s: Arm tucked.", self.node_name)
                    self.state = 9

                #rospy.sleep(1)

            # State 9:  Localize myself
            if self.state == 9:
                rospy.loginfo("%s: STATE 0", self.node_name)
                localize_myself_var = rospy.ServiceProxy(self.localize_myself, Empty)
                localize_req = localize_myself_var()
                rospy.loginfo("%s: Localization initialized", self.node_name)
                self.state = 10
                #rospy.sleep(1)

            # State 10:  Turn around in order to localize in the right direction
            if self.state == 10:
                rate = rospy.Rate(10)
                movement = Twist()
                movement.angular.z = 1
                cnt = 0
                while cnt<20:
                    self.cmd_vel_pub.publish(movement)
                    rate.sleep()
                    cnt = cnt + 1
                self.state = 11
                rospy.loginfo("%s: After turning Im supposed to know where am I", self.node_name)
                #rospy.sleep(1)

            # State 11:  Clear costmaps
            if self.state == 11:
                clear_cost_m = rospy.ServiceProxy(self.clear_cost_map, Empty)
                clear_cost_m()
                self.state = 12
                rospy.loginfo("%s: Costmaps cleared", self.node_name)
                #rospy.sleep(1)

            # State 12:  Send navigation goal
            if self.state == 12:
                self.move_to_dir.wait_for_server()
                self.move_to_dir.send_goal(self.next_goal)
                reached_point = self.move_to_dir.wait_for_result()
                #rospy.sleep(1)
                if reached_point:
                    self.move_to_dir.cancel_goal()
                    rospy.loginfo("%s: I reach the table to pick the cube", self.node_name)
                    self.state = 13
                else:
                    rospy.logerr("%s: I couldnt reach the table", self.node_name)
                    self.state = 6




            # State 13: Move head down
            if self.state == 13:
                try:
                    rospy.loginfo("%s: Lowering robot head", self.node_name)
                    move_head_srv = rospy.ServiceProxy(self.mv_head_srv_nm, MoveHead)
                    move_head_req = move_head_srv("down")

                    if move_head_req.success == True:
                        self.state = 14
                        rospy.loginfo("%s: Move head down succeded!", self.node_name)
                    else:
                        rospy.logerr("%s: Move head down failed!", self.node_name)
                        self.state = 6

                    #rospy.sleep(3)

                except rospy.ServiceException, e:
                    print "Service call to move_head server failed: %s"%e

            # State 14: Pick cube
            if self.state == 14:
                rospy.loginfo("%s: Picking the cube...", self.node_name)
                pick_the_cube = rospy.ServiceProxy(self.pick_srv_nm, SetBool)

                pick_cube_req = pick_the_cube(True)

                if pick_cube_req.success == True:
                    self.state = 15
                    rospy.loginfo("%s: Pick cube succeded!", self.node_name)
                else:
                    rospy.loginfo("%s: Pick cube failed!", self.node_name)
                    self.state = 6

                #rospy.sleep(1)

            # State 15:  Clear costmaps
            if self.state == 15:
                clear_cost_m = rospy.ServiceProxy(self.clear_cost_map, Empty)
                clear_cost_m()
                self.state = 16
                rospy.loginfo("%s: Costmaps cleared", self.node_name)
                #rospy.sleep(1)

            # State 16:  Heads up
            if self.state == 16:
                try:
                    rospy.loginfo("%s: Upping robot head", self.node_name)
                    move_head_srv = rospy.ServiceProxy(self.mv_head_srv_nm, MoveHead)
                    move_head_req = move_head_srv("up")

                    if move_head_req.success == True:
                        self.state = 17
                        rospy.loginfo("%s: Move head doupwn succeded!", self.node_name)
                    else:
                        rospy.logerr("%s: Move head up failed!", self.node_name)
                        self.state = 6

                    #rospy.sleep(3)

                except rospy.ServiceException, e:
                    print "Service call to move_head server failed: %s"%e


            # State 15:  Move the robot to chair
            if self.state == 17:
                self.move_to_dir.wait_for_server()
                self.move_to_dir.send_goal(self.place_goal)
                reached_point = self.move_to_dir.wait_for_result()
                rospy.sleep(1)
                if reached_point:
                    self.move_to_dir.cancel_goal()
                    rospy.logerr("%s: I reach the table to place the cube", self.node_name)
                    self.state = 18
                else:
                    rospy.loginfo("%s: I couldnt reach the table to place the cube", self.node_name)
                    self.state = 6
                #rospy.sleep(1)


            # State 18:  Lower robot head service
            if self.state == 18:
            	try:
                    rospy.loginfo("%s: Lowering robot head", self.node_name)
                    move_head_srv = rospy.ServiceProxy(self.mv_head_srv_nm, MoveHead)
                    move_head_req = move_head_srv("down")

                    if move_head_req.success == True:
                        self.state = 19
                        rospy.loginfo("%s: Move head down succeded!", self.node_name)
                    else:
                        rospy.loginfo("%s: Move head down failed!", self.node_name)
                        self.state = 6

                    #rospy.sleep(1)

                except rospy.ServiceException, e:
                    print "Service call to move_head server failed: %s"%e

            # State 5: Leave the cube
            if self.state == 19:
                rospy.loginfo("%s: Leaving the cube...", self.node_name)
                place_the_cube = rospy.ServiceProxy(self.place_srv_nm, SetBool)

                place_cube_req = place_the_cube(True)

                if place_cube_req.success == True:

                    rospy.loginfo("%s: Now we check if the cube is there", self.node_name)
                    self.state = 20

                else:
                    rospy.loginfo("%s: Place cube failed!", self.node_name)
                    self.state = 6

            if self.state == 20:
                if (rospy.get_rostime().secs > 1 + self.time_data):
                    rospy.logerr("%s: CUBE LOST", self.node_name)
                    self.state = 6
                    
                else:
                    rospy.loginfo("%s: CUBE GOOD", self.node_name)
                    return


                #rospy.sleep(1)


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
