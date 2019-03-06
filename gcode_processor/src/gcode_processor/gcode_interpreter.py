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
A mini Python GCode parser and interpreter.

This is intended to parse and interpret only a limited subset of GCodes, and as such
is not appropriate for most CNC machines or 3D Printers.

Instead, this focuses on GCodes that utilize 2D strategies such as 'Engrave' or 'Trace'
for the purposes of drawing, weaving or other creative 2-dimensional movements

Author: Maxwell Svetlik
"""

import rospy
import rospkg
import actionlib
import rospkg
import sys
import os

class GCodeLine():
    def __init__(self, code="0", x="", y="", z="", i="", j=""):
        self.code = code
        self.x = x
        self.y = y
        self.z = z
        self.i = i
        self.j = j

    def set_code(self,code): self.code = code
    def set_x(self,x): self.x = x
    def set_y(self,y): self.y = y
    def set_z(self,z): self.z = z
    def set_i(self,i): self.i = i
    def set_j(self,j): self.j = j

    def get_formatted_data(self):
        data = [self.code, self.x, self.y, self.z, self.i, self.j]
        string = "code: {0} X:{1} Y:{2} Z:{3}".format(self.code, self.x, self.y, self.z)
        return string

class GCodeInterpreter():

        ACCEPTED_LETTERS = ['G','X','Y','Z']
        ASSUMED_COMMANDS = ['X', 'Y', 'Z']
        SUPPORTED_GCODES = ["G1", "G0", "G20", "G21"]

	def __init__(self, filename):
		self.lines = None
		self.line_index = 0
                self.last_gcode_line = GCodeLine()
		self.conversion_factor = 0.001

                if os.path.exists(filename):
		   with open(filename, 'rb') as f:
		       try:
			    self.lines = f.readlines()
		       except IOError:
			    print "Error opening GCode file."

	def get_next_line(self):
		if self.lines is not None:
		    if self.line_index < len(self.lines):
                        line = self.lines[self.line_index]
			self.line_index += 1
			return line
		return -1

        def gcode_accepted(self, next_line):
            # Filter out comments, code block names, MCodes, SCodes
            if not next_line[0] in self.ACCEPTED_LETTERS:
                return False

            if next_line[0] in self.ASSUMED_COMMANDS:
                return True

            # Filter out unsupported gcodes
            my_code = next_line.partition(' ')[0]
            my_code = my_code.replace('\n', '')
            my_code = my_code.replace('\r', '')
            if not my_code in self.SUPPORTED_GCODES:
                return False

            return True

        def get_fnext(self):
            gcodeline = GCodeLine()
            next_line = 0
            while next_line != -1:
                next_line = self.get_next_line()
                if next_line == -1:
                    continue
                if not self.gcode_accepted(next_line):
                    continue
                #Gcode line is accepted. Check and handle if its a 'preprocess' GCode
                else:
                    my_code = next_line.partition(' ')[0]
                    my_code = my_code.replace('\n', '')
                    my_code = my_code.replace('\r', '')

                    if my_code == "G20": #sets units to INCH
                        self.conversion_factor = 0.0254
                        continue
                    elif my_code == "G21": #sets units to MILIMETERS
                        self.conversion_factor = 0.001
                        continue
                    break

            if next_line == -1:
                return -1

            delimited_line = next_line.split(" ")
            my_code = next_line.partition(' ')[0]


            if my_code == "G1":
                self.last_gcode_line.code = "G1"
            elif my_code == "G0":
                self.last_gcode_line.code = "G0"
            else:
                gcodeline.set_x(self.last_gcode_line.x)
                gcodeline.set_y(self.last_gcode_line.y)
                gcodeline.set_z(self.last_gcode_line.z)

            gcodeline.set_code(self.last_gcode_line.code)

            for segment in delimited_line:
                segment = segment.replace('\n', '')

                if "X" in segment:
                    loc = segment.replace("X", '')
                    gcodeline.set_x(float(loc)*self.conversion_factor)
                if "Y" in segment:
                    loc = segment.replace("Y", '')
                    gcodeline.set_y(float(loc)*self.conversion_factor)
                if "Z" in segment:
                    loc = segment.replace("Z", '')
                    gcodeline.set_z(float(loc)*self.conversion_factor)
            return gcodeline

        def get_num_lines(self):
            return len(self.lines)

