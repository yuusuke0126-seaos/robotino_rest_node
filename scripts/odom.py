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
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

# api-endpoint 
URL = "http://127.0.0.1/data/odometry"
PARAMS = {'sid':'robotino_rest_node'} 

def talker():
  odomPub = rospy.Publisher('/odom', Odometry, queue_size=50)
  odom_broadcaster = tf.TransformBroadcaster()
  rospy.init_node('robotino_odom_publisher', anonymous=True)
  rate = rospy.Rate(10) # 10hz
  while not rospy.is_shutdown():
    try:
      r = requests.get(url = URL, params = PARAMS)
      if r.status_code == requests.codes.ok:
        data = r.json() 
        rospy.loginfo(data)
        current_time = rospy.Time.now()
        x = data[0]
        y = data[1]
        w = data[2]
        vx = data[3]
        vy = data[4]
        vw = data[5]

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, w)
        odom_broadcaster.sendTransform((x, y, 0.), odom_quat, current_time, "base_footprint", "odom")
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))
        odom.child_frame_id = "base_footprint"
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vw))
        odomPub.publish(odom)
      else:
        rospy.logwarn("get from %s with params %s failed", URL, PARAMS)
    except requests.exceptions.RequestException as e:
      rospy.logerr("%s", e)
      pass
    rate.sleep()

if __name__ == '__main__':
  myargv = rospy.myargv(argv=sys.argv)
  if len(myargv)>1:
    URL = URL.replace("127.0.0.1",myargv[1])
  print("connecting to: ",URL)
  try:
    talker()
  except rospy.ROSInterruptException:
    pass
