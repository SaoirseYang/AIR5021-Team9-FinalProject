#! /usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Twist, PoseStamped, Point
from costmap_converter.msg import ObstacleArrayMsg
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
import geometry_msgs.msg
import tf.transformations
import tf
from collections import namedtuple
from RDA_planner.mpc import MPC
from math import atan2
from gctl.curve_generator import curve_generator
from sklearn.cluster import DBSCAN
from sensor_msgs.msg import LaserScan
import cv2
from math import cos, sin

robot_tuple = namedtuple(
    "robot_tuple", "G h cone_type wheelbase max_speed max_acce dynamics"
)
rda_obs_tuple = namedtuple(
    "rda_obs_tuple", "center radius vertex cone_type velocity"
)  # vertex: 2*number of vertex
import warnings
warnings.filterwarnings("ignore")  # 忽略所有警告

class rda_core:
    def __init__(self) -> None:

        # publish topics
        self.vel_pub = rospy.Publisher("/rda_cmd_vel", Twist, queue_size=10)
        self.rda_path_pub = rospy.Publisher("/rda_opt_path", Path, queue_size=10)

        self.ref_path_pub = rospy.Publisher("/rda_ref_path", Path, queue_size=10)
        self.ref_states_pub = rospy.Publisher("/rda_ref_states", Path, queue_size=10)
        self.obs_pub = rospy.Publisher("/rda_obs_markers", MarkerArray, queue_size=10)
        # self.rda_obs_pub = rospy.Publisher("/rda_converted_obs_markers", MarkerArray, queue_size=10)
        # New publisher for planned trajectory with velocities
        self.traj_pub = rospy.Publisher("/rda_planned_trajectory", MultiDOFJointTrajectory, queue_size=1)

        rospy.init_node("rda_node", anonymous=True)

        # ros parameters

        ## robot info
        robot_info = rospy.get_param(
            "~robot_info",
            {
                "vertices": None,
                "radius": None,
                "max_speed": [10, 1],
                "max_acce": [10, 0.5],
                "length": 2,
                "width": 1,
                "wheelbase": 1.5,
                "dynamics": "diff",
                "cone_type": "Rpositive",
            },
        )

        ## For rda MPC
        receding = rospy.get_param("~receding", 10)
        iter_num = rospy.get_param("~iter_num", 2)
        enable_reverse = rospy.get_param("~enable_reverse", False)
        sample_time = rospy.get_param("~sample_time", 0.1)
        process_num = rospy.get_param("~process_num", 4)
        accelerated = rospy.get_param("~accelerated", True)
        time_print = rospy.get_param("~time_print", False)
        obstacle_order = rospy.get_param("~obstacle_order", True)
        self.max_edge_num = rospy.get_param("~max_edge_num", 5)
        self.max_obstacle_num = rospy.get_param("~max_obs_num", 5)
        self.goal_index_threshold = rospy.get_param("~goal_index_threshold", 1)
        self.sample_time = sample_time  # Store this for trajectory timing

        ## Tune parameters
        iter_threshold = rospy.get_param("~iter_threshold", 0.2)
        slack_gain = rospy.get_param("~slack_gain", 8)
        max_sd = rospy.get_param("~max_sd", 1.0)
        min_sd = rospy.get_param("~min_sd", 0.1)
        ws = rospy.get_param("~ws", 1.0)
        wu = rospy.get_param("~wu", 0.5)
        ro1 = rospy.get_param("~ro1", 200)
        ro2 = rospy.get_param("~ro2", 1.0)

        # reference speed
        self.ref_speed = rospy.get_param("~ref_speed", 4.0)  # ref speed

        ## for scan
        use_scan_obstacle = rospy.get_param("~use_scan_obstacle", False)
        self.scan_eps = rospy.get_param("~scan_eps", 0.2)
        self.scan_min_samples = rospy.get_param("~scan_min_samples", 6)

        ## for reference paths
        self.waypoints = rospy.get_param("~waypoints", [])
        self.loop = rospy.get_param("~loop", False)
        self.curve_type = rospy.get_param("~curve_type", "dubins")
        self.step_size = rospy.get_param("~step_size", 0.1)
        self.min_radius = rospy.get_param("~min_radius", 1.0)

        ## for frame
        self.target_frame = rospy.get_param("~target_frame", "map")
        self.lidar_frame = rospy.get_param("~lidar_frame", "lidar_link")
        self.base_frame = rospy.get_param("~base_frame", "base_link")

        # for visualization
        self.marker_x = rospy.get_param("~marker_x", 0.05)
        self.marker_lifetime = rospy.get_param("~marker_lifetime", 0.1)

        # initialize
        self.robot_state = None
        self.obstacle_list = []
        self.cg = curve_generator()
        self.listener = tf.TransformListener()
        self.ref_path_list = (
            self.generate_ref_path_list()
        )  # generate the initial reference path
        robot_info_tuple = self.generate_robot_tuple(robot_info)

        self.rda_opt = MPC(
            robot_info_tuple,
            self.ref_path_list,
            receding,
            sample_time,
            iter_num,
            enable_reverse,
            False,
            obstacle_order,
            self.max_edge_num,
            self.max_obstacle_num,
            process_num,
            accelerated,
            time_print,
            self.goal_index_threshold,
            iter_threshold=iter_threshold,
            slack_gain=slack_gain,
            max_sd=max_sd,
            min_sd=min_sd,
            ws=ws,
            wu=wu,
            ro1=ro1,
            ro2=ro2,
        )

        rospy.Subscriber("/rda_goal", PoseStamped, self.goal_callback)
        rospy.Subscriber("/rda_sub_path", Path, self.path_callback)
        self.listener = tf.TransformListener()

        # Topic Subscribe
        if not use_scan_obstacle:
            rospy.Subscriber("/rda_obstacles", ObstacleArrayMsg, self.obstacle_callback)
        else:
            rospy.Subscriber("/scan", LaserScan, self.scan_callback)

    def publish_planned_trajectory(self, planned_states, planned_velocities):
        """
        Publishes the planned trajectory (states and velocities) for consumption by the velocity publisher
        
        Args:
            planned_states: List of state vectors [x, y, theta]
            planned_velocities: Array of velocity vectors [v, w] with shape (2, n)
        """
        if not planned_states or len(planned_states) == 0:
            rospy.logwarn_throttle(1.0, "No planned states available for trajectory")
            return

        planned_states = planned_states[1:]
        # Create a MultiDOFJointTrajectory message
        traj_msg = MultiDOFJointTrajectory()
        traj_msg.header.stamp = rospy.Time.now()
        traj_msg.header.frame_id = self.target_frame

        # Add trajectory points with positions and velocities
        for i in range(len(planned_states)):
            point = MultiDOFJointTrajectoryPoint()

            # Set the time from start
            point.time_from_start = rospy.Duration(i * self.sample_time)

            # Get the state
            state = planned_states[i]

            # Create transform for position
            transform = geometry_msgs.msg.Transform()
            transform.translation.x = state[0, 0]
            transform.translation.y = state[1, 0]
            transform.translation.z = 0.0

            # Set orientation (from yaw)
            quat = tf.transformations.quaternion_from_euler(0, 0, state[2, 0])
            transform.rotation.x = quat[0]
            transform.rotation.y = quat[1]
            transform.rotation.z = quat[2]
            transform.rotation.w = quat[3]

            point.transforms.append(transform)

            # Set velocity if available (linear and angular)
            if i < planned_velocities.shape[1]:
                twist = geometry_msgs.msg.Twist()
                twist.linear.x = planned_velocities[0, i]  # Linear velocity
                twist.angular.z = planned_velocities[1, i]  # Angular velocity
                point.velocities.append(twist)

            traj_msg.points.append(point)

        # Publish the trajectory
        self.traj_pub.publish(traj_msg)

    def control(self):

        rate = rospy.Rate(50)

        while not rospy.is_shutdown():

            self.read_robot_state()

            if self.robot_state is None:
                rospy.loginfo_throttle(1, "waiting for robot states")
                continue

            if len(self.obstacle_list) == 0:
                rospy.loginfo_throttle(1, "No obstacles, perform path tracking")
            else:
                rda_obs_markers = self.convert_to_markers(self.obstacle_list)
                self.obs_pub.publish(rda_obs_markers)

            if self.rda_opt.no_ref_path():

                rospy.loginfo_throttle(
                    1, "waiting for reference path, topic '/rda_sub_path' "
                )
                continue

            else:
                ref_path = self.convert_to_path(self.ref_path_list)
                self.ref_path_pub.publish(ref_path)

            # Run MPC controller
            start_time = rospy.get_time()

            if self.max_obstacle_num == 0:
                opt_vel, info = self.rda_opt.control(
                    self.robot_state, self.ref_speed, []
                )
            else:
                opt_vel, info = self.rda_opt.control(
                    self.robot_state, self.ref_speed, self.obstacle_list
                )

            planning_time = rospy.get_time() - start_time
            rospy.loginfo("MPC planning completed in {:.3f} seconds".format(planning_time))

            if info["arrive"]:
                if self.loop:

                    self.goal = self.rda_opt.ref_path[0]
                    start = self.rda_opt.ref_path[-1]

                    self.ref_path_list = self.cg.generate_curve(
                        self.curve_type,
                        [start, self.goal],
                        self.step_size,
                        self.min_radius,
                    )
                    print("start new loop")
                    self.rda_opt.update_ref_path(self.ref_path_list)

                else:
                    opt_vel = np.zeros((2, 1))
                    print("arrive at the goal!")

            vel = self.convert_to_twist(opt_vel)
            rda_opt_path = self.convert_to_path(info["opt_state_list"])
            ref_states = self.convert_to_path(info["ref_traj_list"])

            self.ref_states_pub.publish(ref_states)
            vel = self.convert_to_twist(opt_vel)
            self.vel_pub.publish(vel)
            # Extract planned states and velocities
            # planned_states = info["opt_state_list"]
            # planned_velocities = info["velocity_trajectory"]
            # print("planned_states", planned_states)
            # print("planned_velocities", planned_velocities)
            # Publish the planned trajectory for the velocity publisher
            # self.publish_planned_trajectory(planned_states, planned_velocities)
            # self.rda_path_pub.publish(rda_opt_path)

            # if "rda_obs_list" in info and len(info["rda_obs_list"]) > 0:
            #     rda_obs_markers = self.convert_rda_to_markers(info["rda_obs_list"])
            #     self.rda_obs_pub.publish(rda_obs_markers)

            # rate.sleep()

    def read_robot_state(self):

        try:
            (trans, rot) = self.listener.lookupTransform(
                self.target_frame, self.base_frame, rospy.Time(0)
            )
            # print(trans, rot)
            yaw = self.quat_to_yaw_list(rot)
            x, y = trans[0], trans[1]
            self.robot_state = np.array([x, y, yaw]).reshape(3, 1)

        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            rospy.loginfo_throttle(
                1,
                "waiting for tf for the transform from {} to {}".format(
                    self.base_frame, self.target_frame
                ),
            )

    def obstacle_callback(self, obstacle_array):

        temp_obs_list = []

        if self.max_obstacle_num == 0:
            rospy.loginfo_once(1, "No obstacles are considered")
            return

        for obstacles in obstacle_array.obstacles:

            vertex = obstacles.polygon.points
            vertex_num = len(vertex)

            if vertex_num == 1:
                # circle obstacle

                center = np.array([[vertex[0].x], [vertex[0].y]])
                radius = obstacles.radius

                linear_x, linear_y = (
                    obstacles.velocities.twist.linear.x,
                    obstacles.velocities.twist.linear.y,
                )
                velocity = np.array([[linear_x], [linear_y]])

                circle_obs = rda_obs_tuple(center, radius, None, "norm2", velocity)

                temp_obs_list.append(circle_obs)

            elif vertex_num == 2:
                # line obstacle
                continue

            elif vertex_num > 2:
                # polygon obstacle
                vertex_list = [np.array([[p.x], [p.y]]) for p in vertex]
                vertexes = np.hstack(vertex_list)

                linear_x, linear_y = (
                    obstacles.velocities.twist.linear.x,
                    obstacles.velocities.twist.linear.y,
                )
                velocity = np.array([[linear_x], [linear_y]])

                polygon_obs = rda_obs_tuple(None, None, vertexes, "Rpositive", velocity)

                temp_obs_list.append(polygon_obs)

        self.obstacle_list[:] = temp_obs_list[:]

    def path_callback(self, path):

        self.ref_path_list = []

        for p in path.poses:
            x = p.pose.position.x
            y = p.pose.position.y
            theta = self.quat_to_yaw(p.pose.orientation)

            points = np.array([x, y, theta]).reshape(3, 1)
            self.ref_path_list.append(points)

        if len(self.ref_path_list) == 0:
            rospy.loginfo_throttle(
                1,
                "No waypoints are converted to reference path, waiting for new waypoints",
            )
            return

        rospy.loginfo_throttle(0.1, "reference path update")
        self.rda_opt.update_ref_path(self.ref_path_list)

    def goal_callback(self, goal):

        x = goal.pose.position.x
        y = goal.pose.position.y
        theta = self.quat_to_yaw(goal.pose.orientation)

        self.goal = np.array([[x], [y], [theta]])

        print(f"set rda goal: {self.goal}")

        self.ref_path_list = self.cg.generate_curve(
            self.curve_type,
            [self.robot_state, self.goal],
            self.step_size,
            self.min_radius,
        )

        if len(self.ref_path_list) == 0:
            rospy.loginfo_throttle(
                1,
                "No waypoints are converted to reference path, waiting for new waypoints",
            )
            return

        rospy.loginfo_throttle(0.1, "reference path update")
        self.rda_opt.update_ref_path(self.ref_path_list)

    def scan_callback(self, scan_data):

        ranges = np.array(scan_data.ranges)
        angles = np.linspace(scan_data.angle_min, scan_data.angle_max, len(ranges))

        point_list = []

        for i in range(len(ranges)):
            scan_range = ranges[i]
            angle = angles[i]

            if scan_range < (scan_data.range_max - 0.01):
                point = np.array(
                    [[scan_range * np.cos(angle)], [scan_range * np.sin(angle)]]
                )
                point_list.append(point)

        if len(point_list) < 3 or self.robot_state is None:
            rospy.loginfo_throttle(1, "No obstacles are converted to polygon")
            return

        else:

            # get the transform from lidar to target frame
            try:
                trans, rot = self.listener.lookupTransform(
                    self.target_frame, self.lidar_frame, rospy.Time(0)
                )

                yaw = self.quat_to_yaw_list(rot)
                x, y = trans[0], trans[1]

                l_trans, l_R = self.get_transform(np.array([x, y, yaw]).reshape(3, 1))

            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
            ):
                rospy.loginfo_throttle(
                    1,
                    "waiting for tf for the transform from {} to {}".format(
                        self.lidar_frame, self.target_frame
                    ),
                )
                return

            # convert the points to convex polygon
            point_array = np.hstack(point_list).T
            labels = DBSCAN(
                eps=self.scan_eps, min_samples=self.scan_min_samples
            ).fit_predict(point_array)

            self.obstacle_list = []

            for label in np.unique(labels):
                if label == -1:
                    continue
                else:
                    point_array2 = point_array[labels == label]
                    rect = cv2.minAreaRect(point_array2.astype(np.float32))
                    box = cv2.boxPoints(rect)

                    vertices = box.T

                    global_vertices = l_R @ vertices + l_trans

                    self.obstacle_list.append(
                        rda_obs_tuple(None, None, global_vertices, "Rpositive", 0)
                    )

    def generate_ref_path_list(self):

        if len(self.waypoints) == 0:
            return []

        else:
            point_list = [
                np.array([[p[0]], [p[1]], [p[2]]]).astype("float64")
                for p in self.waypoints
            ]
            ref_path_list = self.cg.generate_curve(
                self.curve_type, point_list, self.step_size, self.min_radius
            )

            return ref_path_list

    def convert_rda_to_markers(self, rda_obs_list):
        """
        Convert the RDA obstacle list to RViz markers

        Args:
            rda_obs_list: List of RDA obstacles (rdaobs tuples)

        Returns:
            MarkerArray: Markers for visualization in RViz
        """
        marker_array = MarkerArray()

        for obs_index, obs in enumerate(rda_obs_list):
            marker = Marker()
            marker.header.frame_id = self.target_frame
            marker.header.stamp = rospy.get_rostime()

            # Use a different color for converted obstacles
            marker.color.a = 1.0
            marker.color.r = 1.0  # Red color to differentiate from original obstacles
            marker.color.g = 0.0
            marker.color.b = 0.0

            marker.lifetime = rospy.Duration(self.marker_lifetime)

            # Check if it's a polygon obstacle (has A matrix for inequality constraints)
            if obs.cone_type == "Rpositive" and obs.vertex is not None:
                marker.type = marker.LINE_LIST
                marker.scale.x = self.marker_x

                # Create line segments from vertices
                temp_matrix = np.hstack((obs.vertex, obs.vertex[:, 0:1]))
                for i in range(temp_matrix.shape[1] - 1):
                    vp = temp_matrix[:, i]
                    vp1 = temp_matrix[:, i + 1]

                    marker.points.append(Point(vp[0], vp[1], 0))
                    marker.points.append(Point(vp1[0], vp1[1], 0))

            # Check if it's a circle obstacle (has center and norm2 cone type)
            elif obs.cone_type == "norm2" and obs.center is not None:
                marker.type = marker.CYLINDER

                center = obs.center
                # For circles, extract radius from b[2] (the third component of b)
                radius = -float(obs.b[2]) if isinstance(obs.b, np.ndarray) else -float(obs.b[0][2])

                marker.scale.x = radius * 2
                marker.scale.y = radius * 2
                marker.scale.z = 0.2

                marker.pose.position.x = center[0, 0]
                marker.pose.position.y = center[1, 0]

            marker.id = obs_index
            marker_array.markers.append(marker)

        return marker_array

    def convert_to_markers(self, obs_list):

        marker_array = MarkerArray()

        # obs: center, radius, vertex, cone_type, velocity

        for obs_index, obs in enumerate(obs_list):
            marker = Marker()
            marker.header.frame_id = self.target_frame

            marker.header.stamp = rospy.get_rostime()

            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0

            marker.lifetime = rospy.Duration(self.marker_lifetime)

            # breakpoint()

            if obs.vertex is not None:

                marker.type = marker.LINE_LIST

                marker.scale.x = self.marker_x

                temp_matrix = np.hstack((obs.vertex, obs.vertex[:, 0:1]))
                for i in range(temp_matrix.shape[1] - 1):
                    vp = temp_matrix[:, i]
                    vp1 = temp_matrix[:, i + 1]

                    marker.points.append(Point(vp[0], vp[1], 0))
                    marker.points.append(Point(vp1[0], vp1[1], 0))

                marker.id = obs_index
                marker_array.markers.append(marker)

            else:
                marker.type = marker.CYLINDER

                center = obs.center
                marker.scale.x = obs.radius * 2
                marker.scale.y = obs.radius * 2
                marker.scale.z = 0.2

                marker.pose.position.x = center[0, 0]
                marker.pose.position.y = center[1, 0]

                marker.id = obs_index

                marker_array.markers.append(marker)

        return marker_array

    def convert_to_path(self, state_list):
        # from state list to path
        path = Path()

        path.header.seq = 0
        path.header.stamp = rospy.get_rostime()
        path.header.frame_id = self.target_frame

        for i in range(len(state_list)):
            ps = PoseStamped()

            ps.header.seq = i
            ps.header.stamp = rospy.get_rostime()
            ps.header.frame_id = self.target_frame

            ps.pose.position.x = state_list[i][0, 0]
            ps.pose.position.y = state_list[i][1, 0]
            ps.pose.orientation.w = 1

            path.poses.append(ps)

        return path

    def convert_to_twist(self, rda_vel):
        # from 2*1 vector to twist

        vel = Twist()
        vel.linear.x = rda_vel[0, 0]  # linear
        vel.angular.z = rda_vel[1, 0]  # steering

        return vel

    @staticmethod
    def quat_to_yaw(quater):

        x = quater.x
        y = quater.y
        z = quater.z
        w = quater.w

        raw = atan2(2 * (w * z + x * y), 1 - 2 * (pow(z, 2) + pow(y, 2)))

        return raw

    @staticmethod
    def quat_to_yaw_list(quater):

        x = quater[0]
        y = quater[1]
        z = quater[2]
        w = quater[3]

        raw = atan2(2 * (w * z + x * y), 1 - 2 * (pow(z, 2) + pow(y, 2)))

        return raw

    def generate_robot_tuple(self, robot_info):

        if robot_info is None:
            print("Lack of car information, please check the robot_info in config file")
            return

        if (
            robot_info.get("vertices", None) is None
            or robot_info.get("vertices", None) == "None"
        ):
            length = robot_info["length"]
            width = robot_info["width"]
            wheelbase = robot_info["wheelbase"]

            start_x = -(length - wheelbase) / 2
            start_y = -width / 2

            point0 = np.array([[start_x], [start_y]])  # left bottom point
            point1 = np.array([[start_x + length], [start_y]])
            point2 = np.array([[start_x + length], [start_y + width]])
            point3 = np.array([[start_x], [start_y + width]])

            vertex = np.hstack((point0, point1, point2, point3))

            G, h = self.generate_Gh(vertex)
        else:
            G, h = self.generate_Gh(robot_info["vertices"])

        cone_type = robot_info["cone_type"]
        max_speed = robot_info["max_speed"]
        max_acce = robot_info["max_acce"]
        dynamics = robot_info["dynamics"]

        robot_info_tuple = robot_tuple(
            G, h, cone_type, wheelbase, max_speed, max_acce, dynamics
        )

        return robot_info_tuple

    def generate_Gh(self, vertex):
        """
        vertex: 2*num
        """

        num = vertex.shape[1]

        G = np.zeros((num, 2))
        h = np.zeros((num, 1))

        for i in range(num):
            if i + 1 < num:
                pre_point = vertex[:, i]
                next_point = vertex[:, i + 1]
            else:
                pre_point = vertex[:, i]
                next_point = vertex[:, 0]

            diff = next_point - pre_point

            a = diff[1]
            b = -diff[0]
            c = a * pre_point[0] + b * pre_point[1]

            G[i, 0] = a
            G[i, 1] = b
            h[i, 0] = c

        return G, h

    def get_transform(self, state):
        """
        Get rotation and translation matrices from state.

        Args:
            state (np.array): State [x, y, theta] (3x1) or [x, y] (2x1).

        Returns:
            tuple: Translation vector and rotation matrix.
        """

        if state.shape == (2, 1):
            rot = np.array([[1, 0], [0, 1]])
            trans = state[0:2]
        else:
            rot = np.array(
                [
                    [cos(state[2, 0]), -sin(state[2, 0])],
                    [sin(state[2, 0]), cos(state[2, 0])],
                ]
            )
            trans = state[0:2]
        return trans, rot
