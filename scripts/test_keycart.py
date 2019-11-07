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

from math import cos, acos, pi
import time
import numpy as np

from tf.transformations import euler_from_quaternion

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist

import rospy

class TestKeycart():
  def __init__(self):
    rospy.init_node('test_keycart_drive', anonymous=True)
    rospy.Subscriber("/imu_data", Imu, self.ImuCallback)
    self.pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
    self.yaw = 0.0
    self.yaw_last = 0.0

  def ImuCallback(self, data):
    quat = data.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    yaw_change = yaw - self.yaw_last
    if abs(yaw_change) > pi:
      yaw_change = -np.sign(yaw_change) * (2*pi - abs(yaw_change))
    self.yaw += yaw_change
    self.yaw_last = yaw
    

  def startTurn(self, target_angle):
    max_z = 0.5
    rate = rospy.Rate(10)
    t = Twist()
    count = 0
    while (count < 10) and (not rospy.is_shutdown()):
      cmd_z = 1.5*(target_angle - self.yaw)
      if abs(cmd_z) > max_z:
        cmd_z = cmd_z * abs(max_z / cmd_z)
      t.angular.z = cmd_z
      t.linear.x = 0.0
      rospy.loginfo("Yaw (deg): %1.1f", self.yaw*180/pi)
      self.pub.publish(t)
      if abs(target_angle-self.yaw) < 0.05:
        count += 1
      else:
        count = 0
      rate.sleep()
    t.angular.z = 0.0
    self.pub.publish(t)
    return target_angle-self.yaw

  def startCirculate(self, keep_angle, pipe_distance):
    if (keep_angle==0) or (abs(keep_angle)>=pi):
      rospy.logwarn("keep_angle is not appropriate, %1.1f", keep_angle)
      return 0
    traj_r = 1.7 / cos(pi/2-keep_angle)
    if pipe_distance > abs(traj_r) * (1+cos(keep_angle)):
      target_angle = np.sign(traj_r) * pi
    else:
      target_angle = np.sign(traj_r) * (2*pi - acos(cos(keep_angle) - pipe_distance/abs(traj_r)))
    max_x = 0.3
    max_z = max_x / abs(traj_r)
    rate = rospy.Rate(10)
    t = Twist()
    count = 0
    while (count < 10) and (not rospy.is_shutdown()):
      cmd_z = 1.0*(target_angle-self.yaw)
      if abs(cmd_z) > max_z:
        cmd_z = cmd_z * abs(max_z / cmd_z)
      cmd_x = cmd_z * traj_r
      t.angular.z = cmd_z
      t.linear.x = cmd_x
      rospy.loginfo("Yaw (deg): %1.1f", self.yaw*180/pi)
      self.pub.publish(t)
      if abs(target_angle-self.yaw) < 0.05:
        count += 1
      else:
        count = 0
      rate.sleep()
    t.angular.z = 0.0
    t.linear.x = 0.0
    self.pub.publish(t)
    return target_angle-self.yaw
  
  def startStraight(self, sec):
    max_x = 0.4
    count = 0
    rate = rospy.Rate(10)
    t = Twist()
    while (count < sec*10) and (not rospy.is_shutdown()):
      t.linear.x = min(0.05 * (sec*10 - count), max_x)
      rospy.loginfo("Yaw (deg): %1.1f", self.yaw*180/pi)
      self.pub.publish(t)
      count = count + 1
      rate.sleep()
    t.linear.x = 0.0
    self.pub.publish(t)

  def startDrive(self):
    time.sleep(1)
    self.yaw = 0.0
    target_angle = 70.0 * pi / 180.0 # target_angle must be in [-pi:pi] (except 0), positive -> turn left, negative -> turn right
    pipe_distance = 2.0              # Must be positive value
    rospy.loginfo("Target angle: %1.1f", target_angle*180/pi)

    ang_err = self.startTurn(target_angle)
    if abs(ang_err) > 0.1:
      rospy.logwarn("angle error: %1.1f", ang_err*180/pi)
    rospy.loginfo("Turn finish! err: %1.1f", ang_err*180/pi)
    time.sleep(1)

    ang_err = self.startCirculate(keep_angle=target_angle, pipe_distance)
    if abs(ang_err) > 0.1:
      rospy.logwarn("angle error: %1.1f", ang_err*180/pi)
    rospy.loginfo("Circulate finish! err: %1.1f", ang_err*180/pi)
    time.sleep(1)

    ang_err = self.startTurn(target_angle=pi)
    if abs(ang_err) > 0.1:
      rospy.logwarn("angle error: %1.1f", ang_err*180/pi)
    rospy.loginfo("Turn finish! err: %1.1f", ang_err*180/pi)
    time.sleep(1)

    self.startStraight(sec=5)
    rospy.loginfo("Straight finish! Yaw: %1.1f", self.yaw*180/pi)
      

if __name__ == '__main__':
  try:
    tk = TestKeycart()
    tk.startDrive()
  except rospy.ROSInterruptException:
    pass
