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

import roslib
roslib.load_manifest('diagnostic_updater')
import diagnostic_updater
import diagnostic_msgs

import rospy

# api-endpoint 
URL = "http://127.0.0.1/"
PARAMS = {'sid':'robotino_rest_node'} 

class BatteryDiagnostics():
  def __init__(self):
    rospy.init_node('battery_diagnostics_publisher', anonymous=True)
    self.updater = diagnostic_updater.Updater()
    self.updater.setHardwareID("Kobuki")
    self.updater.add("Battery", self.check_battery)
  
  def getSensorData(self, data_name):
    try:
      r = requests.get(url = URL+data_name, params = PARAMS)
      if r.status_code == requests.codes.ok:
        data = r.json() 
        #rospy.loginfo(data)
      else:
        rospy.logwarn("get from %s with params %s failed", URL+data_name, PARAMS)
    except requests.exceptions.RequestException as e:
      rospy.logerr("%s", e)
      pass
    return data

  def check_battery(self, stat):
    data = self.getSensorData(data_name = "data/festoolcharger")
    battery_voltage = data["payload"]["voltages"][0:2]
    battery_percent = data["payload"]["capacities"][0:2]
    battery_voltage = sum(battery_voltage) / 2
    battery_percent = sum(battery_percent) / 2
    if battery_percent > 40:
      stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "OK")
    elif battery_percent > 20:
      stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "Low")
    else:
      stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "Dangerous")
    stat.add("Voltage (V)", battery_voltage)
    stat.add("Percent", battery_percent)
    return stat
  
  def startPublish(self):
    rate = rospy.Rate(1) # 1 Hz
    while not rospy.is_shutdown():
      self.updater.update()
      rate.sleep()
      

if __name__ == '__main__':
  myargv = rospy.myargv(argv=sys.argv)
  if len(myargv)>1:
    URL = URL.replace("127.0.0.1",myargv[1])
  print("connecting to: ",URL)
  try:
    bd = BatteryDiagnostics()
    bd.startPublish()
  except rospy.ROSInterruptException:
    pass
