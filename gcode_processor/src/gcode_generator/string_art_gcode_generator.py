#!/usr/bin/env python
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
This program takes in a list of nodes that describe movement for string art applications on a circular hoop, and outputs gcode.
The radius of the hoop and the number of nodes are configurable.

E.g. a list of [1, 2, 3, 4] and a node number of 4 would describe the following movement:


1   - - - - - -  2
|                |
|                |
|                |
|                |
|                |
|                |
4   - - - - - -  3


This program generates gcode to approach the nodes, which in the real world are capped pegs, ahead of the peg, and move
to the side, the back, the opposite side and again to the front of the peg. So the gcode movement described for approaching node 1
looks like the following:

             c
         /

	d    1    b

  	 \       /
	     a
	     e


Usage:
    $ ./string_art_gcode_generator.py INPUTFILE.TXT OUTPUTFILE.NC



Author: Ryan Menz, Maxwell Svetlik

"""

import math
import sys


class GenerateStringArtGCode():

    def __init__(self, n=8, hoop_rad=3, bolt_rad=1):

	self.n=8 #warning code only works for even nums
	self.hoop_rad=3 #radius of hoop in inches
	self.bolt_rad=1 #effective diameter of bolt head
	self.lists = [1,4,7,2,5,8,3,6,1,3,5,7,2,4,6,8,2,666] #lists of points, first value is 1, last value not used

    def angle(self, num):#far right peg is 0 degrees, top peg is 90 degrees, far left is 180, bottom is 270
        return (num-1)*360.0/self.n

    def xcor(self,num):#outputs x coordinate of approach point
        return self.hoop_rad*round(math.cos(math.radians(self.angle(num))),2)

    def ycor(self,num):#outputs y coordinate of approach point
        return self.hoop_rad*round(math.sin(math.radians(self.angle(num))),2)

    def xcoor1(self,num):#x coordinate of 1st waypoint
        return .7071067812*self.bolt_rad*round(math.cos(math.radians(self.angle(num)+45)),2)

    def ycoor1(self,num):#y coordinate of 1st waypoint
        return .7071067812*self.bolt_rad**round(math.sin(math.radians(self.angle(num)+45)),2)

    def xcoor2(self,num):#x coordinate of 2nd waypoint
        return self.bolt_rad**round(math.cos(math.radians(self.angle(num))),2)

    def ycoor2(self,num):#y coordinate of 2nd waypoint
        return self.bolt_rad**round(math.sin(math.radians(self.angle(num))),2)

    def xcoor3(self,num):#x coordinate of 3rd waypoint
        return .7071067812*self.bolt_rad**round(math.cos(math.radians(self.angle(num)-45)),2)

    def ycoor3(self,num):#y coordinate of 3rd waypoint
        return .7071067812*self.bolt_rad**round(math.sin(math.radians(self.angle(num)-45)),2)

    def generate(self):
        #output file
        #try:
        file = open(str(sys.argv[2]),'w')#change location for your machine

        file.write('G20 G90 G17') #G20 - inches, G90 - absolute positioning, G17 - arcs are made in xy plane
        file.write('G0 X0 Y0 Z0.5\n')
        file.write('Z0.1\n')
        file.write('Z0.0\n')
        file.write('\n')
        #print "M66"  #pauses - tie string here

        for i in range(0,len(self.lists)-1):#takes points and lists and produces gcode to go to approach points and waypoints around the peg
            file.write('(%s' %self.lists[i] + ')\n')
            file.write('G0 X%s'  %str(self.xcor(self.lists[i])) + ' Y%s\n' %str(self.ycor(self.lists[i])))
            file.write('G0 X%s'  %str(self.xcor(self.lists[i])+self.xcoor1(self.lists[i])) + ' Y%s\n' %str(self.ycor(self.lists[i])+self.ycoor1(self.lists[i])))
            file.write('G0 X%s'  %str(self.xcor(self.lists[i])+self.xcoor2(self.lists[i])) + ' Y%s\n' %str(self.ycor(self.lists[i])+self.ycoor2(self.lists[i])))
            file.write('G0 X%s'  %str(self.xcor(self.lists[i])+self.xcoor3(self.lists[i])) + ' Y%s\n' %str(self.ycor(self.lists[i])+self.ycoor3(self.lists[i])))
            file.write('G0 X%s'  %str(self.xcor(self.lists[i])) + ' Y%s\n' %str(self.ycor(self.lists[i])))
            file.write('\n')

        file.write('X0 Y0 Z0.1\n')
        file.write('Z0.5\n')
        file.close()
        #except:
        #    print "Error writing to specified output file path. Is it valid?"

if __name__=='__main__':
    gsagc = GenerateStringArtGCode()
    gsagc.generate()

