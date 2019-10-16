#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, REC Robotics Equipment Corporation GmbH, Planegg
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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

import requests 
import sys
import json

from math import sin, cos, pi
import numpy
import cvxopt
from cvxopt import matrix

import rospy
from geometry_msgs.msg import Twist

# api-endpoint 
URL = "http://127.0.0.1/"
PARAMS = {'sid':'robotino_rest_node'} 

class RobotinoDriver():
  def __init__(self):
    rospy.init_node('robotino_omnidrive', anonymous=True)
    rospy.Subscriber("/cmd_vel", Twist, self.callback)
    self.pdata = [0.0, 0.0, 0.0] # [vx, vy, vw]
    self.last_time = rospy.Time.now()
  
  def getSensorData(self, data_name):
    try:
      r = requests.get(url = URL+data_name, params = PARAMS)
      if r.status_code == requests.codes.ok:
        data = r.json() 
        rospy.loginfo(data)
      else:
        rospy.logwarn("get from %s with params %s failed", URL+data_name, PARAMS)
    except requests.exceptions.RequestException as e:
      rospy.logerr("%s", e)
      pass
    return data
  
  def modifyCmdVel(self, dist_data):
    # min_x (v-x)^2 s.t. Gx <= h
    P = matrix(numpy.eye(2))
    q = matrix(-numpy.array(self.pdata[0:2]))
    G0, h0 = [], []
    for i in range(9):
      if dist_data[i] < 0.35:
        G0.append([cos(pi/4.5*i), sin(pi/4.5*i)])
#        h0.append((dist_data[i]-0.3)*0.15/0.05)
        h0.append(0.0)
    G = matrix(numpy.array(G0))
    h = matrix(numpy.array(h0))
    sol=cvxopt.solvers.qp(P,q,G,h)
    self.pdata[0:2] = sol["x"]

  def driveMotor(self, vel_cmd):
    try:
      r = requests.post(url = URL+"data/omnidrive", params = PARAMS, data = json.dumps(vel_cmd) )
      if r.status_code != requests.codes.ok:
        rospy.logwarn("post to %s with params %s failed", URL+"data/omnidrive", PARAMS)
    except requests.exceptions.RequestException as e:
      rospy.logerr("%s", e)
      pass

  def callback(self, data):
    rospy.loginfo("%f %f %f" % (data.linear.x, data.linear.y, data.angular.z) )
    self.last_time = rospy.Time.now()
    self.pdata = [data.linear.x, data.linear.y, data.angular.z]
  
  def startDrive(self):
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
      current_time = rospy.Time.now()
      is_bumper = self.getSensorData(data_name = "data/bumper")["value"]
      dist_data = self.getSensorData(data_name = "data/distancesensorarray")
      if is_bumper is True:
        self.pdata = [0.0, 0.0, 0.0]
        rospy.logwarn("Bumper is hit!")
      elif current_time - self.last_time > rospy.Duration(0, 5.0 * 10**8): # current_time - last_time > 0.5 sec
        self.pdata = [0.0, 0.0, 0.0]
        rospy.logwarn("CmdVel is not published!")
      elif min(dist_data) < 0.35:
        self.modifyCmdVel(dist_data)
        rospy.logwarn("There are obstacles near the robotino!")
      self.driveMotor(self.pdata)
      rate.sleep()
      

if __name__ == '__main__':
  myargv = rospy.myargv(argv=sys.argv)
  if len(myargv)>1:
    URL = URL.replace("127.0.0.1",myargv[1])
  print("connecting to: ",URL)
  try:
    rd = RobotinoDriver()
    rd.startDrive()
  except rospy.ROSInterruptException:
    pass
