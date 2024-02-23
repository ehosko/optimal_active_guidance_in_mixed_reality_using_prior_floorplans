#!/usr/bin/env python

# Python
import sys
import time
import csv
import datetime
import os
import re
import subprocess
import numpy as np

# ros
import rospy
import message_filters
from message_filters import TimeSynchronizer, Subscriber
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import SetBool
from voxblox_msgs.srv import FilePath
from nav_msgs.msg import Odometry

import tf2_ros
from geometry_msgs.msg import TransformStamped, PointStamped
# from ros_numpy import numpify

class EvalDataRosbag(object):
    def __init__(self):
        '''  Initialize ros node and read params '''
        # Parse parameters

        self.output_dir = rospy.get_param('~output_folder',"/home/michbaum/Projects/optag_EH/data/drift_logs/")
        self.part = rospy.get_param('~part',0)
    
        # Buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # self.tf_buffer.lookup_transform("world", "world", rospy.Time(), rospy.Duration(0.5))
        
        self.drifty_data_file = open(os.path.join(self.output_dir,'traj_rosbag_estimate_' + str(self.part) + '.csv'), 'wb')
        self.drifty_writer = csv.writer(self.drifty_data_file,
                                        delimiter=',',
                                        quotechar='|',
                                        quoting=csv.QUOTE_MINIMAL,
                                        lineterminator='\n')
        print("Writing to file: ", os.path.join(self.output_dir,"traj_rosbag_estimate.csv"))

        self.drifty_writer.writerow(['Time', 'xPosition', 'yPosition', 'zPosition',
                                    'xOrientation', 'yOrientation', 'zOrientation', 'wOrientation'])
        

        self.gt_data_file = open(os.path.join(self.output_dir,'traj_rosbag_gt_' + str(self.part) + '.csv'), 'wb')
        self.gt_writer = csv.writer(self.gt_data_file,
                                        delimiter=',',
                                        quotechar='|',
                                        quoting=csv.QUOTE_MINIMAL,
                                        lineterminator='\n')
        print("Writing to file: ", os.path.join(self.output_dir,"traj_rosbag_estimate.csv"))

        self.gt_writer.writerow(['Time', 'xPosition', 'yPosition', 'zPosition'])
        
        print("Subscribing to odometry topic")

    
        self.drifty_sub = rospy.Subscriber('odometry', Odometry,self.drifty_odom_callback,
                                            queue_size=10)
        self.gt_sub = rospy.Subscriber('gtPosition', PointStamped,self.gt_callback,queue_size=10)

        # self.drifty_sub = Subscriber('odometry', Odometry)

        
        # self.gt_sub = Subscriber('gtPosition', PointStamped)

        # self.ts = TimeSynchronizer([self.drifty_sub, self.gt_sub], 10)
        # self.ts.registerCallback(self.callback)

        
        # Finish
        rospy.on_shutdown(self.eval_finish)


    def eval_finish(self):

        self.drifty_data_file.close()

    
    def drifty_odom_callback(self, msg):

        time = msg.header.stamp.to_sec()

        #print("Writing")

        try:
            #transform_world_gt = self.tf_buffer.lookup_transform("world", "firefly/base_link", rospy.Time(), rospy.Duration(0.5))
            transform_world_drifty = self.tf_buffer.lookup_transform("mission", "rovioli/imu", rospy.Time(), rospy.Duration(0.5))
            

            #self.gt_writer.writerow([time,transform_world_gt.transform.translation.x, transform_world_gt.transform.translation.y, transform_world_gt.transform.translation.z,
            #                        transform_world_gt.transform.rotation.x, transform_world_gt.transform.rotation.y, transform_world_gt.transform.rotation.z, transform_world_gt.transform.rotation.w])
            self.drifty_writer.writerow([time,transform_world_drifty.transform.translation.x, transform_world_drifty.transform.translation.y, transform_world_drifty.transform.translation.z,
                                    transform_world_drifty.transform.rotation.x, transform_world_drifty.transform.rotation.y, transform_world_drifty.transform.rotation.z, transform_world_drifty.transform.rotation.w])


            self.gt_writer.writerow([time,self.gt_pos.point.x, self.gt_pos.point.y, self.gt_pos.point.z])

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("Transform not found")

    def gt_callback(self, msg):
        
        self.gt_pos = msg
        
    def callback(self,drifty ,gt):

        time = drifty.header.stamp.to_sec()

        print("Writing")

        try:
            #transform_world_gt = self.tf_buffer.lookup_transform("world", "firefly/base_link", rospy.Time(), rospy.Duration(0.5))
            transform_world_drifty = self.tf_buffer.lookup_transform("mission", "rovioli/imu", rospy.Time(), rospy.Duration(0.5))
            

            #self.gt_writer.writerow([time,transform_world_gt.transform.translation.x, transform_world_gt.transform.translation.y, transform_world_gt.transform.translation.z,
            #                        transform_world_gt.transform.rotation.x, transform_world_gt.transform.rotation.y, transform_world_gt.transform.rotation.z, transform_world_gt.transform.rotation.w])
            self.drifty_writer.writerow([time,transform_world_drifty.transform.translation.x, transform_world_drifty.transform.translation.y, transform_world_drifty.transform.translation.z,
                                    transform_world_drifty.transform.rotation.x, transform_world_drifty.transform.rotation.y, transform_world_drifty.transform.rotation.z, transform_world_drifty.transform.rotation.w])

            self.gt_writer.writerow([time,gt.point.x, gt.point.y, gt.point.z])


        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("Transform not found")

         
if __name__ == '__main__':
    rospy.init_node('eval_data_rosbag_node', anonymous=True)
    ed = EvalDataRosbag()
    rospy.spin()
