#!/usr/bin/env python3

###############################################################################
# Copyright 2020 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

import argparse
import atexit
import logging
import os
import sys
import time

#from common.logger import Logger
from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import cyber_time

from modules.common_msgs.localization_msgs import localization_pb2
from modules.common_msgs.localization_msgs import gps_pb2
from modules.common_msgs.transform_msgs import transform_pb2

class OdomPublisher(object):
    def __init__(self, node):
        self.localization = localization_pb2.LocalizationEstimate()
        self.gps_odom_pub = node.create_writer('/apollo/sensor/gnss/odometry', gps_pb2.Gps)
        self.tf_pub = node.create_writer('/tf', transform_pb2.TransformStampeds)
        self.local_pub = node.create_writer('/apollo/localization/pose', localization_pb2.LocalizationEstimate)
        self.sequence_num = 0
        self.terminating = False
        self.position_x = 10
        self.position_y = 10
        self.position_z = 0
        self.orientation_x = 0
        self.orientation_y = 0
        self.orientation_z = 0
        self.orientation_w = 1
        self.linear_velocity_x = 1
        self.linear_velocity_y = 0
        self.linear_velocity_z = 0

    def localization_callback(self, data):
        """
        New message received
        """
        self.localization.CopyFrom(data)
        self.position_x = self.localization.pose.position.x
        self.position_y = self.localization.pose.position.y
        self.position_z = self.localization.pose.position.z
        self.orientation_x = self.localization.pose.orientation.qx
        self.orientation_y = self.localization.pose.orientation.qy
        self.orientation_z = self.localization.pose.orientation.qz
        self.orientation_w = self.localization.pose.orientation.qw
        self.linear_velocity_x = self.localization.pose.linear_velocity.x
        self.linear_velocity_y = self.localization.pose.linear_velocity.y
        self.linear_velocity_z = self.localization.pose.linear_velocity.z

    def publish_odom(self):
        odom = gps_pb2.Gps()
        now = cyber_time.Time.now().to_sec()
        odom.header.timestamp_sec = now
        odom.header.module_name = "odometry"
        odom.header.sequence_num = self.sequence_num
        self.sequence_num = self.sequence_num + 1

        odom.localization.position.x = self.position_x
        odom.localization.position.y = self.position_y
        odom.localization.position.z = self.position_z
        odom.localization.orientation.qx = self.orientation_x
        odom.localization.orientation.qy = self.orientation_y
        odom.localization.orientation.qz = self.orientation_z
        odom.localization.orientation.qw = self.orientation_w
        odom.localization.linear_velocity.x = self.linear_velocity_x
        odom.localization.linear_velocity.y = self.linear_velocity_y
        odom.localization.linear_velocity.z = self.linear_velocity_z
        #self.logger.info("%s"%odom)
        self.gps_odom_pub.write(odom)

    def publish_tf(self):
        tf = transform_pb2.TransformStamped()
        now = cyber_time.Time.now().to_sec()
        tf.header.timestamp_sec = now
        #tf.header.module_name = "tf"
        tf.header.sequence_num = self.sequence_num
        tf.header.frame_id = "world"

        tf.child_frame_id = "localization"
        tf.transform.translation.x = self.position_x
        tf.transform.translation.y = self.position_y
        tf.transform.translation.z = self.position_z

        tf.transform.rotation.qx = self.orientation_x
        tf.transform.rotation.qy = self.orientation_y
        tf.transform.rotation.qz = self.orientation_z
        tf.transform.rotation.qw = self.orientation_w

        tf_1 = transform_pb2.TransformStampeds()
        tf_1.transforms.append(tf)

        self.tf_pub.write(tf_1)
    
    def publish_localization(self):
        local = localization_pb2.LocalizationEstimate()
        now = cyber_time.Time.now().to_sec()
        local.header.timestamp_sec = now
        local.measurement_time = now
        local.header.module_name = "localization"
        local.header.sequence_num = self.sequence_num

        local.pose.position.x = self.position_x
        local.pose.position.y = self.position_y
        local.pose.position.z = self.position_z
        local.pose.orientation.qx = self.orientation_x
        local.pose.orientation.qy = self.orientation_y
        local.pose.orientation.qz = self.orientation_z
        local.pose.orientation.qw = self.orientation_w
        local.pose.heading = 0
        local.pose.linear_velocity.x = self.linear_velocity_x
        local.pose.linear_velocity.y = self.linear_velocity_y
        local.pose.linear_velocity.z = self.linear_velocity_z

        self.local_pub.write(local)

    def shutdown(self):
        """
        shutdown rosnode
        """
        self.terminating = True
        #self.logger.info("Shutting Down...")
        time.sleep(0.2)

def main():
    """
    Main rosnode
    """
    node = cyber.Node('odom_publisher')
    odom = OdomPublisher(node)
    node.create_reader('/apollo/localization/pose', localization_pb2.LocalizationEstimate, odom.localization_callback)
    while not cyber.is_shutdown():
        now = cyber_time.Time.now().to_sec()
        odom.publish_odom()
        odom.publish_tf()
        odom.publish_localization()
        sleep_time = 0.01 - (cyber_time.Time.now().to_sec() - now)
        if sleep_time > 0:
            time.sleep(sleep_time)

if __name__ == '__main__':
    cyber.init()
    main()
    cyber.shutdown()
