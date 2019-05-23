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
import control_msgs
import rospkg
import tf
import moveit_commander
import moveit_msgs.msg
import sys
import rospkg
import math

from geometry_msgs.msg import PoseStamped, Point, Quaternion, PolygonStamped, Point32, Polygon
from gcode_processor.gcode_interpreter import *
from moveit_msgs.srv import GetPositionIK
from svenzva_msgs.msg import *
from control_msgs.msg import FollowJointTrajectoryAction , FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

from sensor_msgs.msg import JointState


class XYZTuple:
    def __init__(self,x,y,z):
        self.x = x
        self.y = y
        self.z = z

    def x(self):
        return self.x
    def y(self):
        return self.y
    def z(self):
        return self.z


class RevelGCodeCommander:

	def __init__(self, filename):
		self.gci = GCodeInterpreter(filename)
		self.joint_states = JointState()
		joint_states_sub = rospy.Subscriber('/joint_states', JointState, self.js_cb, queue_size=1)
		rospy.sleep(2)

                # parameters for smoothing point to point movements
                self.delta_pos=0.02 #how close every joint needs to be to the target joint position, in radians
                self.delta_time=0.05 #velocity time step to look at the next position this far in the future

	def js_cb(self, data):
	    self.joint_states = data


	def maintain_orientation(self):
		position = Point()
		roll = 3.14159
		yaw = 0
		pitch = 0

		ps = PoseStamped()
		ps.header.frame_id = "/world"
		ps.pose.position = position

		quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
		ps.pose.orientation.x = quat[0]
		ps.pose.orientation.y = quat[1]
		ps.pose.orientation.z = quat[2]
		ps.pose.orientation.w = quat[3]


		return ps.pose.orientation

        """
        Using controller parameters defined globally, determines when the robot is 'close enough' to the
        target (commanded) position
        """
	def wait_for_stall(self, q_ar):
	    time = rospy.get_rostime()
	    max_time = rospy.Duration(10.0)
            gr = [4,6,6,2,3,1]
	    while rospy.get_rostime() < time + max_time:
		stalled = True
		for i in range(0,6):
                    vel_component = self.joint_states.velocity[i] * 0.229 * 3.1415 / 30 * self.delta_time
		    if abs(self.joint_states.position[i] - q_ar[i] + vel_component) > self.delta_pos:
			stalled &= False
		if stalled:
		    #rospy.loginfo("Arm reached target position.")
		    return
		rospy.sleep(0.05)
	    #rospy.loginfo("Arm stalled: did not reach target position after 10 seconds.")
	    return


        """
        Preprocesses a gcode file, stuffing them into a list of stucts
        """
        def stuff_gcode(self):
            gcode_ar = []
            res = 0
            maximum_distance = 0.05
            last_point = Point()

            while not rospy.is_shutdown() and res is not -1:
                res = self.gci.get_fnext()
                if res == -1:
                    return gcode_ar
                print res.get_formatted_data()

                if res.code == "G1" or res.code == "G0":
                    gcode_pose = last_point
                    if res.x != "":
                        gcode_pose.x = float(res.x)

                    if res.y != "":
                        gcode_pose.y = float(res.y)

                    last_point = gcode_pose

                    x = res.x
                    y = res.y
                    z = res.z
                    t = XYZTuple(x,y,z)
                    gcode_ar.append(XYZTuple(x,y,z))
            return gcode_ar


        # For some longer linear movements, it may be advantageous to interpolate points along the line
        # and break the longer linear movements into more smaller ones.
        def interpolate_ar(self, ar):
            interpolated_ar = []
            max_dist = 0.02
            item = ar[0]
            i = 1

            while i < len(ar):
                next_item = ar[i]
                interpolated_ar.append(item)

                dist = math.sqrt(math.pow(float(next_item.x) - float(item.x), 2) + math.pow(float(next_item.y) - float(item.y), 2))
                if dist > max_dist:
                    j = 0
                    while j < (dist / max_dist) -1:
                        j +=1
                        x_p = float(item.x) + j*max_dist*(float(next_item.x) / float(item.x))
                        y_p = float(item.y) + j*max_dist*(float(next_item.y) / float(item.y))
                        interpolated_ar.append(XYZTuple(x_p, y_p, item.z))


                item = next_item
                i+=1

            return interpolated_ar


        """
        this uses MoveIt for Inverse Kinematic Computation, but does not use it for path planning
        so while this may potentially collide with the environment, while executing, it will allow
        for smoother, faster point-to-point trajectories

        Visualization information is published on /target_pose and /drawing/gcode_path topics

        Note that the x,y, and z offset values should be in METERS regardless of the unit type used in the gcode file
        """
        def execute_gcode(self, x_offset, y_offset, z_offset, simulation=False):
                moveit_commander.roscpp_initialize(sys.argv)
                robot = moveit_commander.RobotCommander()
		scene = moveit_commander.PlanningSceneInterface()
		group = moveit_commander.MoveGroupCommander("svenzva_arm")
		path_pose_publisher = rospy.Publisher("/drawing/gcode_path",PolygonStamped, queue_size=2)
		target_pose_publisher = rospy.Publisher("/target_pose",PoseStamped, queue_size=2)
                if not simulation:
                    action_client = actionlib.SimpleActionClient('revel/follow_joint_trajectory', control_msgs.msg.FollowJointTrajectoryAction)
                    action_client.wait_for_server()


                rospy.wait_for_service('compute_ik')
                compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

                ik_req = moveit_msgs.msg.PositionIKRequest()
                ik_req.group_name = "svenzva_arm"
                ik_req.ik_link_name = "base_link"

                first_z = True
                first_z_pos = 0.05


                last_point = Point()
                delta_copy = self.delta_pos
                gcode_ar = self.stuff_gcode()

                poly_stmp_msg = PolygonStamped()
                poly_stmp_msg.header.frame_id = "base_link"
                poly_msg = Polygon()
                poly_stmp_msg.polygon = poly_msg

                i = 0
                while not rospy.is_shutdown() and i < len(gcode_ar):
		    cur_rob_pose = group.get_current_pose().pose
                    ik_req.robot_state = robot.get_current_state()
                    z_stall_condition = False
                    z_delta = 0.0

                    res = gcode_ar[i]
                    i+=1


                    gcode_pose = last_point

                    if res.x != "":
                        gcode_pose.x = float(res.x + x_offset)

                    if res.y != "":
                        gcode_pose.y = float(res.y + y_offset)

                    if res.z != "":
                        first_z = False
                        z_delta = abs(gcode_pose.z - res.z - z_offset)
                        print z_delta

                        gcode_pose.z = float(res.z+z_offset)

                    if first_z:
                        gcode_pose.z = first_z_pos

                    #begin display publishing
                    poly_point = Point32()
                    poly_point.x = gcode_pose.x
                    poly_point.y = gcode_pose.y
                    poly_point.z = gcode_pose.z

                    poly_msg.points.append(poly_point)

                    last_point = gcode_pose
                    pub_pose = PoseStamped()
                    pub_pose.header.frame_id = "base_link"

                    pub_pose.pose.orientation = self.maintain_orientation()
                    pub_pose.pose.position = gcode_pose


                    path_pose_publisher.publish(poly_stmp_msg)
                    path_pose_publisher.publish(poly_stmp_msg)

                    target_pose_publisher.publish(pub_pose)
                    target_pose_publisher.publish(pub_pose)
                    #end display publishing

                    if len(ik_req.pose_stamped_vector) > 0:
                        ik_req.pose_stamped_vector.pop()

                    ik_req.pose_stamped_vector.append(pub_pose)

                    try:
                        resp = compute_ik(ik_req)
                        pos= resp.solution.joint_state.position
                        goal = SvenzvaJointGoal()
                        goal.positions = resp.solution.joint_state.position


                        goal = FollowJointTrajectoryGoal()
                        goal.trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
                        point = JointTrajectoryPoint()
                        point.positions = resp.solution.joint_state.position
                        #Since this is asynchronous, the time from 2 points is 0 and the action will  return immediately
                        point.time_from_start = rospy.Duration(0.1)
                        goal.trajectory.points.append(point)

                        if not simulation:
                            action_client.send_goal(goal)


                        # For Z-positional movements about this value, reduce the delta_pos controller parameter
                        # this sets the z_stall_condition flag which forces the robot to move to its commanded Z height
                        # fully before continuing to other points
                        if z_delta > 0.002:
                            z_stall_condition = True
                            self.delta_pos = 0.065

                        if simulation:
                            rospy.sleep(0.1)
                        else:
                            self.wait_for_stall(resp.solution.joint_state.position)

                        if z_stall_condition:
                            self.delta_pos = delta_copy
                            rospy.sleep(1.0)

                    except rospy.ServiceException, e:
                        rospy.loginfo("Service call failed: %s"%e)
                        return


if __name__=='__main__':
        rospy.init_node('revel_gcode_commander', anonymous=True)
	rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('gcode_processor')
        desired_file = "square_flower.nc"
        full_path = pkg_path + "/gcode/" + desired_file
        dgc = RevelGCodeCommander(full_path)
        dgc.execute_gcode(0, 0.3, -0.003)


