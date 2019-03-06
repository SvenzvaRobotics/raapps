#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2019 Svenzva Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Svenzva Robotics LLC nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
The main program to conduct drawing from GCODE / Numerical Control (.nc) files.

This program is the interface layer between GCode and robot commands.
Only a subset of GCodes are supported, and due to the workspace of robot arms,
not all X/Y/Z coordinates will be achievable with the robot's orientation constrained



Author: Maxwell Svetlik
"""

import rospy
import rospkg
import actionlib
import rospkg
import tf
import moveit_commander
import moveit_msgs.msg
import sys


from geometry_msgs.msg import PoseStamped, Point, Quaternion
from gcode_processor.gcode_interpreter import *

class DrawGCode:

	def __init__(self, filename):
		self.gci = GCodeInterpreter(filename)
                pass

	def goto_draw_pose(self):
		#goto draw pose
		return

	def maintain_orientation(self):
		position = Point()
		roll = 3.14159/2
		yaw = 3.14159/2
		pitch = 0

		ps = PoseStamped()
		ps.header.frame_id = "/world"
		ps.pose.position = position

		quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
		ps.pose.orientation.x = quat[0]
		ps.pose.orientation.y = quat[1]
		ps.pose.orientation.z = quat[2]
		ps.pose.orientation.w = quat[3]


		#visualize the
		#pub = rospy.Publisher('/orientation', PoseStamped, queue_size=10)
		#while not rospy.is_shutdown():
		#	pub.publish(ps)
		#	rospy.sleep(5)
		return ps.pose.orientation

	def moveit_example(self):
		moveit_commander.roscpp_initialize(sys.argv)
		robot = moveit_commander.RobotCommander()
		scene = moveit_commander.PlanningSceneInterface()
		group = moveit_commander.MoveGroupCommander("svenzva_arm")

		display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory)

		target_pose_publisher = rospy.Publisher("/target_pose",PoseStamped, queue_size=2)

		print "============ Reference frame: %s" % group.get_planning_frame()
		print "============ Reference frame: %s" % group.get_end_effector_link()
		print "============ Robot Groups:"
		print robot.get_group_names()
		print "============ Printing robot state"
		print robot.get_current_state()
		print "============"


		joint_goal = group.get_current_joint_values()
		joint_goal[0] = 0
		joint_goal[1] = -3.1415/4
		joint_goal[2] = -2
		joint_goal[3] = 3.1415/2
		joint_goal[4] = 0
		joint_goal[5] = 3.1415/3

		group.go(joint_goal, wait=True)
		group.stop()


                res = 0
                while not rospy.is_shutdown() and res is not -1:
		    cur_rob_pose = group.get_current_pose().pose
                    res = self.gci.get_fnext()
                    if res == -1:
                        return
                    print res.get_formatted_data()
                    if res.code == "G1" or res.code == "G0":
                        gcode_pose = Point()
                        if res.x != "":
                            gcode_pose.x = float(res.x)
                        else:
                            gcode_pose.x = cur_rob_pose.position.x

                        if res.y != "":
                            gcode_pose.y = float(res.y)
                        else:
                            gcode_pose.y = cur_rob_pose.position.y

                        if res.z != "":
                            gcode_pose.z = float(res.z)


                        pub_pose = PoseStamped()
                        pub_pose.header.frame_id = "base_link"
                        #pub_pose.pose.orientation = self.maintain_orientation()
                        pub_pose.pose.position = gcode_pose


                        rospy.sleep(1)
                        target_pose_publisher.publish(pub_pose)
                        target_pose_publisher.publish(pub_pose)
                        target_pose_publisher.publish(pub_pose)
                        target_pose_publisher.publish(pub_pose)
                        rospy.sleep(1)
                        group.set_pose_target(pub_pose)
                        plan1 = group.plan()
                        plan = group.go(wait=True)



if __name__=='__main__':
	rospy.init_node('draw_gcode_node', anonymous=True)
	dgc = DrawGCode("/home/maxwell/catkin_ws/src/raapps/gcode_processor/gcode/revel_gcode_test_line_left.nc")
        dgc.moveit_example()


