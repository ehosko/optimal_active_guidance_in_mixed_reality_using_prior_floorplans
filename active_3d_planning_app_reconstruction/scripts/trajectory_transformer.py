#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs
from trajectory_msgs.msg import MultiDOFJointTrajectory
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseStamped

class TrajectoryTransformerNode:
    def __init__(self):

        # Create a tf2_ros.Buffer and tf2_ros.TransformListener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Get params
        self.drifty_odom_frame = rospy.get_param('~drifty_odom_frame')
        self.gt_odom_frame = rospy.get_param('~simulation_odom_frame')

        # Subscribers
        self.trajectory_sub = rospy.Subscriber('input_trajectory', MultiDOFJointTrajectory, self.trajectory_callback)

        # Publishers
        self.trajectory_pub = rospy.Publisher('output_trajectory', MultiDOFJointTrajectory, queue_size=10)

    def trajectory_callback(self, trajectory_msg):
        try:
            # We need 3 transforms: world to drifty, drifty to gt and drifty to world again

            transform_world_drifty = self.tf_buffer.lookup_transform(self.drifty_odom_frame, 'world', trajectory_msg.header.stamp, rospy.Duration(1.0))
            # Wait for the transform between drifty_odom and ground_truth_odom frames to become available
            # transform_drifty_gt = self.tf_buffer.lookup_transform(self.gt_odom_frame, self.drifty_odom_frame, trajectory_msg.header.stamp, rospy.Duration(1.0))
            transform_gt_drifty = self.tf_buffer.lookup_transform(self.drifty_odom_frame, self.gt_odom_frame, trajectory_msg.header.stamp, rospy.Duration(1.0))

            transform_drifty_world = self.tf_buffer.lookup_transform('world', self.drifty_odom_frame, trajectory_msg.header.stamp, rospy.Duration(1.0))

            # Transform each waypoint in the trajectory
            transformed_trajectory = MultiDOFJointTrajectory()
            transformed_trajectory.header = trajectory_msg.header

            # TODO: Check if it really is this easy, since maybe we need to first build the relative transforms from the trajectory to the poses and then
            #       transform them to the other poses
            for point in trajectory_msg.points:
                # We have to first build a pose out of the transform to transform it
                transformed_pose = PoseStamped()
                transformed_pose.pose.position = point.transforms[0].translation
                transformed_pose.pose.orientation = point.transforms[0].rotation
                # Apply the 3 transforms
                transformed_pose = tf2_geometry_msgs.do_transform_pose(transformed_pose, transform_world_drifty)
                transformed_pose = tf2_geometry_msgs.do_transform_pose(transformed_pose, transform_gt_drifty)
                transformed_pose = tf2_geometry_msgs.do_transform_pose(transformed_pose, transform_drifty_world)
                transformed_transform = point
                transformed_transform.transforms[0].translation = transformed_pose.pose.position
                transformed_transform.transforms[0].rotation = transformed_pose.pose.orientation

                transformed_trajectory.points.append(transformed_transform)


            # Publish the transformed trajectory
            self.trajectory_pub.publish(transformed_trajectory)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Transform not available!")

if __name__ == '__main__':
    try:
        rospy.init_node('trajectory_transformer_node', anonymous=True)
        node = TrajectoryTransformerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
