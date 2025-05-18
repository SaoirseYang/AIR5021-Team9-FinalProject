#! /usr/bin/env python

import numpy as np
import rospy
import tf
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import MultiDOFJointTrajectory
from math import cos, sin, atan2


class rda_velocity_publisher:
    def __init__(self) -> None:
        # Initialize ROS node
        rospy.init_node("rda_velocity_publisher_node", anonymous=True)

        # Publisher for velocity commands
        self.vel_pub = rospy.Publisher("/rda_cmd_vel", Twist, queue_size=10)

        # Subscriber for planned trajectory
        rospy.Subscriber("/rda_planned_trajectory", MultiDOFJointTrajectory, self.planned_velocities_callback)
        self.traj_expiration_time = rospy.get_param("~traj_expiration_time", 4.0)

        # Parameters
        self.pub_rate = rospy.get_param("~pub_rate", 50.0)  # Hz
        self.interpolation_method = rospy.get_param("~interpolation_method", "linear")

        # Initialize tf listener
        self.listener = tf.TransformListener()

        # Initialize storage for trajectory
        self.trajectory_poses = []
        self.trajectory_vels = []
        self.trajectory_times = []
        self.last_traj_time = rospy.Time(0)
        self.trajectory_valid = False

        rospy.loginfo("RDA Velocity Publisher initialized")

    def planned_velocities_callback(self, msg):
        """Callback for receiving planned trajectory with velocities"""
        # Extract trajectory data
        self.trajectory_poses = []
        self.trajectory_vels = []
        self.trajectory_times = []

        start_time = rospy.Time.now()

        # Process each trajectory point
        for i, point in enumerate(msg.points):
            if not point.transforms or not point.velocities:
                rospy.logwarn(f"Invalid trajectory point at index {i}: missing transforms or velocities")
                continue

            # Extract position
            transform = point.transforms[0]
            position = [transform.translation.x, transform.translation.y]

            # Extract velocities
            velocity = point.velocities[0]
            vel_tuple = (velocity.linear.x, velocity.angular.z)

            # Calculate time
            time_from_start = point.time_from_start
            point_time = start_time + time_from_start

            # Store the data
            self.trajectory_poses.append(position)
            self.trajectory_vels.append(vel_tuple)
            self.trajectory_times.append(point_time)

        if len(self.trajectory_times) > 0:
            self.last_traj_time = start_time
            self.trajectory_valid = True
        else:
            rospy.logwarn("Received empty trajectory")

    def find_closest_point_on_trajectory(self):
        """Find the closest point on trajectory to current time and compute interpolation ratio"""
        if not self.trajectory_valid or len(self.trajectory_times) < 2:
            rospy.logwarn_throttle(1.0, "Trajectory invalid or too short for interpolation")
            return None, None

        current_time = rospy.Time.now()

        # Convert trajectory times to seconds since start for easier comparison
        time_diffs = [(t - self.last_traj_time).to_sec() for t in self.trajectory_times]
        current_diff = (current_time - self.last_traj_time).to_sec()

        # If current time is before the first point, use the first point
        if current_diff <= time_diffs[0]:
            rospy.logwarn_throttle(1.0, "Current time is before trajectory start, using first point")
            return 0, 0.0

        # If current time is after the last point, use the last point
        if current_diff >= time_diffs[-1]:
            rospy.logwarn_throttle(1.0, "Current time is after trajectory end, return None")
            return None, None
            # return len(self.trajectory_times) - 1, 0.0

        # Find the closest point by iterating through the time differences
        for i in range(len(time_diffs) - 1):
            if time_diffs[i] <= current_diff <= time_diffs[i + 1]:
                # Calculate interpolation ratio
                delta_t = time_diffs[i + 1] - time_diffs[i]
                if delta_t == 0:
                    ratio = 0.0
                else:
                    ratio = (current_diff - time_diffs[i]) / delta_t
                return i, ratio

        # Fallback in case of unexpected conditions
        rospy.logwarn("Could not find valid trajectory point for current time")
        return None, None

    def interpolate_velocity(self, closest_idx, ratio):
        """Interpolate velocity based on closest point and ratio"""
        if closest_idx is None:
            return None

        # If we're at the last point or second-to-last with ratio=1.0
        if closest_idx >= len(self.trajectory_vels) - 1 or (
                closest_idx == len(self.trajectory_vels) - 2 and ratio >= 1.0):
            return self.trajectory_vels[-1]

        # Linear interpolation between velocities
        v0_linear, v0_angular = self.trajectory_vels[closest_idx]
        v1_linear, v1_angular = self.trajectory_vels[closest_idx + 1]

        linear_vel = v0_linear + ratio * (v1_linear - v0_linear)
        angular_vel = v0_angular + ratio * (v1_angular - v0_angular)

        return (linear_vel, angular_vel)

    def convert_to_twist(self, velocities):
        """Convert velocity tuple to Twist message"""
        if velocities is None:
            return None

        linear_vel, angular_vel = velocities

        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel

        return twist

    def check_trajectory_expired(self):
        """Check if the current trajectory has expired and needs to be reset"""
        if not self.trajectory_valid:
            return True

        # If trajectory is too old, consider it expired
        current_time = rospy.Time.now()
        if (current_time - self.last_traj_time).to_sec() > self.traj_expiration_time:  # 1 second expiration
            rospy.logwarn_throttle(2, "Trajectory has expired - waiting for new one")
            self.trajectory_valid = False
            return True

        return False

    def publish_velocities(self):
        """Main loop to publish velocities at a high rate"""
        rate = rospy.Rate(self.pub_rate)

        rospy.loginfo(f"Starting velocity publisher at {self.pub_rate} Hz")

        zero_vel = Twist()

        while not rospy.is_shutdown():
            # Check if trajectory is valid
            if self.check_trajectory_expired():
                rospy.logwarn_throttle(1.0, "Publishing zero velocity - trajectory expired or not received")
                self.vel_pub.publish(zero_vel)
                rate.sleep()
                continue

            # Find where we are on the trajectory
            closest_idx, ratio = self.find_closest_point_on_trajectory()

            # If we couldn't find a good point, stop
            if closest_idx is None:
                rospy.logwarn_throttle(1.0, "Publishing zero velocity - no valid trajectory point found")
                self.vel_pub.publish(zero_vel)
                rate.sleep()
                continue

            # Interpolate velocity
            velocities = self.interpolate_velocity(closest_idx, ratio)

            # Convert to Twist and publish
            vel_msg = self.convert_to_twist(velocities)
            if vel_msg is not None:
                self.vel_pub.publish(vel_msg)
            else:
                self.vel_pub.publish(zero_vel)

            rate.sleep()


if __name__ == "__main__":
    try:
        vel_publisher = rda_velocity_publisher()
        vel_publisher.publish_velocities()
    except rospy.ROSInterruptException:
        pass
