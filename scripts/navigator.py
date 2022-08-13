#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData, Path
from geometry_msgs.msg import Twist, Pose2D, PoseStamped, TransformStamped
from std_msgs.msg import String
import tf
import numpy as np
from numpy import linalg
from utils import wrapToPi
from planners import AStar, compute_smoothed_traj
from grids import StochOccupancyGrid2D
import scipy.interpolate
import matplotlib.pyplot as plt
from controllers import PoseController, TrajectoryTracker, HeadingController
from enum import Enum
import collections
from visualization_msgs.msg import Marker

from dynamic_reconfigure.server import Server
from asl_turtlebot.cfg import NavigatorConfig
from asl_turtlebot.msg import DetectedObject, VendorLocation, DeliveryRequest

# state machine modes, not all implemented
class Mode(Enum):
    IDLE = 0
    ALIGN = 1
    TRACK = 2
    PARK = 3
    TO_NEXT = 4
    STOP = 5
    STOP_SIGN = 6

class Navigator:
    """
    This node handles point to point turtlebot motion, avoiding obstacles.
    It is the sole node that should publish to cmd_vel
    """
    def __init__(self):
        rospy.init_node('turtlebot_navigator', anonymous=True)
        self.mode = Mode.IDLE

        # current state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # goal state
        self.x_g = None
        self.y_g = None
        self.theta_g = None

        # start state
        self.origin_x = None
        self.origin_y = None
        self.origin_theta = None

        self.th_init = 0.0
        # map parameters
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0
        self.map_origin = [0,0]
        self.map_probs = []
        self.occupancy = None
        self.occupancy_updated = False
        self.map_thresh = 0.5
        self.map_window = 6

        # plan parameters
        self.plan_resolution =  0.05
        self.plan_horizon = 15

        # time when we started following the plan
        self.current_plan_start_time = rospy.get_rostime()
        self.current_plan_duration = 0
        self.plan_start = [0.,0.]
        
        self.latest_request_time = rospy.get_rostime()
        # Robot limits
        self.v_max = rospy.get_param("~v_max", 0.2)    # maximum velocity
        self.om_max = rospy.get_param("~om_max", 0.4)   # maximum angular velocity

        self.v_des = 0.12   # desired cruising velocity
        self.theta_start_thresh = 0.05   # threshold in theta to start moving forward when path-following
        self.start_pos_thresh = 0.2     # threshold to be far enough into the plan to recompute it

        # threshold at which navigator switches from trajectory to pose control
        self.near_thresh = 0.2
        self.at_thresh = 0.02
        self.at_thresh_theta = 0.05

        # trajectory smoothing
        self.spline_alpha = 0.15
        self.traj_dt = 0.1

        # trajectory tracking controller parameters
        self.kpx = 0.5
        self.kpy = 0.5
        self.kdx = 1.5
        self.kdy = 1.5

        # heading controller parameters
        self.kp_th = 2.

        self.traj_controller = TrajectoryTracker(self.kpx, self.kpy, self.kdx, self.kdy, self.v_max, self.om_max)
        self.pose_controller = PoseController(0., 0., 0., self.v_max, self.om_max)
        self.heading_controller = HeadingController(self.kp_th, self.om_max)

        self.nav_planned_path_pub = rospy.Publisher('/planned_path', Path, queue_size=10)
        self.nav_smoothed_path_pub = rospy.Publisher('/cmd_smoothed_path', Path, queue_size=10)
        self.nav_smoothed_path_rej_pub = rospy.Publisher('/cmd_smoothed_path_rejected', Path, queue_size=10)
        self.nav_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.goal_marker_pub = rospy.Publisher('/goal_marker', Marker, queue_size=10)

        self.trans_listener = tf.TransformListener()
        self.trans_broadcaster = tf.TransformBroadcaster()

        self.cfg_srv = Server(NavigatorConfig, self.dyn_cfg_callback)

        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/map_metadata', MapMetaData, self.map_md_callback)
        rospy.Subscriber('/cmd_nav', Pose2D, self.cmd_nav_callback)

        # Project
        self.vendor_locations = {}
        self.delivery_request_locations = collections.deque()
        self.use_shortest_path = False
        # Subscribe to /vendor_location
        rospy.Subscriber('/vendor_location', VendorLocation, self.vendor_location_callback)
        # Subscribe to delivery_request
        rospy.Subscriber('/delivery_request', DeliveryRequest, self.delivery_request_callback)

        # Time when the stop begins
        self.stop_start_time = None
        self.cross_start_time = None
        # Time to stop at a vendor/stop sign
        self.stop_time = rospy.get_param("~stop_time", 3.)
        self.stop_min_dist = rospy.get_param("~stop_min_dist", 1.2)
        self.cross_time = rospy.get_param("~cross_time", 3.)

        print "finished init"
  
    def create_transform_msg(self, x, y, theta, child_frame, base_frame, time=None):
        t = TransformStamped()
        t.header.stamp = time if time else rospy.Time.now()
        t.header.frame_id = base_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0
        quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]
        return t
      
    def dyn_cfg_callback(self, config, level):
        rospy.loginfo("Reconfigure Request: k1:{k1}, k2:{k2}, k3:{k3}, spline_alpha:{spline_alpha}".format(**config))
        self.pose_controller.k1 = config["k1"]
        self.pose_controller.k2 = config["k2"]
        self.pose_controller.k3 = config["k3"]
        # self.spline_alpha = config["spline_alpha"]
        return config

    def cmd_nav_callback(self, data):
        """
        loads in goal if different from current goal, and replans
        """
        if data.x != self.x_g or data.y != self.y_g or data.theta != self.theta_g:
            self.x_g = data.x
            self.y_g = data.y
            self.theta_g = data.theta
            # visualize goal
            self.trans_broadcaster.sendTransformMessage(
                self.create_transform_msg(self.x_g, self.y_g, self.theta_g, "goal_pose", "map"))
            self.replan()

    def map_md_callback(self, msg):
        """
        receives maps meta data and stores it
        """
        self.map_width = msg.width
        self.map_height = msg.height
        self.map_resolution = msg.resolution
        self.map_origin = (msg.origin.position.x,msg.origin.position.y)

    def map_callback(self,msg):
        """
        receives new map info and updates the map
        """
        self.map_probs = msg.data
        # if we've received the map metadata and have a way to update it:
        if self.map_width>0 and self.map_height>0 and len(self.map_probs)>0:
            self.occupancy = StochOccupancyGrid2D(self.map_resolution,
                                                  self.map_width,
                                                  self.map_height,
                                                  self.map_origin[0],
                                                  self.map_origin[1],
                                                  self.map_window,
                                                  self.map_probs,
                                                  self.map_thresh)
            if self.x_g is not None and self.mode != Mode.STOP and self.mode != Mode.STOP_SIGN:
                # if we have a goal to plan to, replan
                rospy.loginfo("replanning because of new map")
                self.replan() # new map, need to replan

    def vendor_location_callback(self, msg):
        if (self.cross_start_time is None or self.has_crossed()) and msg.distance > 0 and msg.distance < self.stop_min_dist and msg.identifier == "stop_sign" and self.mode == Mode.TRACK:
            rospy.loginfo("Stop because of a stop sign!")
            self.init_stop_sign()
            return
        # not record the same vendor
        if msg.identifier in self.vendor_locations:
            return
        self.vendor_locations[msg.identifier] = msg
        rospy.loginfo("Record new vendor: %s, location: x: %s, y: %s", str(msg.identifier), str(msg.x), str(msg.y))

    def delivery_request_callback(self, msg):
        """ callback for a pose goal sent through delivery request """

        if len(self.delivery_request_locations) > 0:
            rospy.logwarn("The current delivery request has not been finished")
            return

        vendor_names = collections.deque(msg.vendors.split(","))
        if len(vendor_names) == 0:
            rospy.loginfo("Invalid delivery request!")
            return

        self.use_shortest_path = msg.use_shortest_path
        for s in vendor_names:
            if s not in self.vendor_locations:
                rospy.loginfo("No food/vendor found for " + s)
            else:
                vendor_location = self.vendor_locations[s]
                self.delivery_request_locations.append(vendor_location)
                rospy.loginfo("Request received! The vendor {} is located at ({}, {})".format(
                    s, vendor_location.x, vendor_location.y))

        if len(self.delivery_request_locations) == 0:
            rospy.loginfo("All requests are invalid!")
            return
        # Record the original states
        if self.origin_x is None:
            self.origin_x = rospy.get_param("~origin_x", self.x)
            self.origin_y = rospy.get_param("~origin_y", self.y)
            self.origin_theta = rospy.get_param("~origin_theta", 0)
        rospy.loginfo("Current x: {} Origin x: {}".format(self.x, self.origin_x))
        self.mode = Mode.TO_NEXT

    def shutdown_callback(self):
        """
        publishes zero velocities upon rospy shutdown
        """
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.nav_vel_pub.publish(cmd_vel)

    def near_goal(self):
        """
        returns whether the robot is close enough in position to the goal to
        start using the pose controller
        """
        if self.x_g is None or self.y_g is None:
            return False
        return linalg.norm(np.array([self.x-self.x_g, self.y-self.y_g])) < self.near_thresh

    def at_goal(self):
        """
        returns whether the robot has reached the goal position with enough
        accuracy to return to idle state
        """
        if self.x_g is None or self.y_g is None:
            return False
        return (linalg.norm(np.array([self.x-self.x_g, self.y-self.y_g])) < self.near_thresh and abs(wrapToPi(self.theta - self.theta_g)) < self.at_thresh_theta)

    def aligned(self):
        """
        returns whether robot is aligned with starting direction of path
        (enough to switch to tracking controller)
        """
        return (abs(wrapToPi(self.theta - self.th_init)) < self.theta_start_thresh)
        
    def close_to_plan_start(self):
        return (abs(self.x - self.plan_start[0]) < self.start_pos_thresh and abs(self.y - self.plan_start[1]) < self.start_pos_thresh)

    def snap_to_grid(self, x):
        return (self.plan_resolution*round(x[0]/self.plan_resolution), self.plan_resolution*round(x[1]/self.plan_resolution))

    def switch_mode(self, new_mode):
        rospy.loginfo("Switching from %s -> %s", self.mode, new_mode)
        self.mode = new_mode

    def publish_planned_path(self, path, publisher):
        # publish planned plan for visualization
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        for state in path:
            pose_st = PoseStamped()
            pose_st.pose.position.x = state[0]
            pose_st.pose.position.y = state[1]
            pose_st.pose.orientation.w = 1
            pose_st.header.frame_id = 'map'
            path_msg.poses.append(pose_st)
        publisher.publish(path_msg)

    def publish_smoothed_path(self, traj, publisher):
        # publish planned plan for visualization
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        for i in range(traj.shape[0]):
            pose_st = PoseStamped()
            pose_st.pose.position.x = traj[i,0]
            pose_st.pose.position.y = traj[i,1]
            pose_st.pose.orientation.w = 1
            pose_st.header.frame_id = 'map'
            path_msg.poses.append(pose_st)
        publisher.publish(path_msg)

    def publish_control(self):
        """
        Runs appropriate controller depending on the mode. Assumes all controllers
        are all properly set up / with the correct goals loaded
        """
        t = self.get_current_plan_time()

        if self.mode == Mode.PARK:
            V, om = self.pose_controller.compute_control(self.x, self.y, self.theta, t)
        elif self.mode == Mode.TRACK:
            V, om = self.traj_controller.compute_control(self.x, self.y, self.theta, t)
        elif self.mode == Mode.ALIGN:
            V, om = self.heading_controller.compute_control(self.x, self.y, self.theta, t)
        else:
            V = 0.
            om = 0.

        cmd_vel = Twist()
        cmd_vel.linear.x = V
        cmd_vel.angular.z = om
        self.nav_vel_pub.publish(cmd_vel)

    def get_current_plan_time(self):
        t = (rospy.get_rostime()-self.current_plan_start_time).to_sec()
        return max(0.0, t)  # clip negative time to 0

    def is_free(self, x):
        # Check if it's inside the bounds of the map
        state_min = self.snap_to_grid((-self.plan_horizon, -self.plan_horizon))
        state_max = self.snap_to_grid((self.plan_horizon, self.plan_horizon))
        for dim in range(len(x)):
            if x[dim] < state_min[dim] or x[dim] > state_max[dim]:
                return False
        # Check if it's inside any obstacle.
        return self.occupancy.is_free(x)

    def plan_to_next_goal_location(self):
        # back to the origin
        if len(self.delivery_request_locations) == 0:
            if self.origin_x == None or self.origin_y == None or self.origin_theta == None:
                self.switch_mode(Mode.IDLE)
                return
            self.x_g = self.origin_x
            self.y_g = self.origin_y
            self.theta_g = self.origin_theta
            rospy.loginfo("Finish all delivery requests. Go back to the original state ({}, {}, {})!".format(
                self.origin_x, self.origin_y, self.origin_theta))
            # visualize the goal
            self.trans_broadcaster.sendTransformMessage(
                self.create_transform_msg(self.x_g, self.y_g, self.theta_g, "goal_pose", "map"))
            self.replan()
        else:
            if self.use_shortest_path:
                min_dist = None
                vendor_location = None
                for loc in self.delivery_request_locations:
                    state_min = self.snap_to_grid((-self.plan_horizon, -self.plan_horizon))
                    state_max = self.snap_to_grid((self.plan_horizon, self.plan_horizon))
                    x_init = self.snap_to_grid((self.x, self.y))
                    x_goal = self.snap_to_grid((loc.x, loc.y))
                    problem = AStar(state_min,state_max,x_init,x_goal,self.occupancy,self.plan_resolution)
                    if not problem.solve():
                        continue
                    dist = problem.cost_to_arrive[problem.x_goal]
                    if min_dist == None or dist < min_dist:
                        min_dist = dist
                        vendor_location = loc
                if vendor_location == None:
                    vendor_location = self.delivery_request_locations.popleft()
                else:
                    self.delivery_request_locations.remove(vendor_location)
            else:
                vendor_location = self.delivery_request_locations.popleft()
            # The object is considered as an obstacle. The occupancy is set in the .world file (the scale is 0.2 now)
            # The goal position should not collide with the object!
            # Note: Might wanna try (+, +), (+, -), (-, +), (-, -) in case we cross the border
            
            test_dir = [(5.0, -5.0), (-5.0, 5.0), (5.0, 5.0), (-5.0, -5.0), (7.0, 0.0), (-7.0, 0.0), (0.0, 7.0), (0.0, -7.0)]
            min_dist = None
            closest = None
            for a, b in test_dir:
                x_test = vendor_location.x + a * self.plan_resolution
                y_test = vendor_location.y + b * self.plan_resolution
                if self.is_free((x_test, y_test)):
                    dist = np.sqrt((self.x - x_test) ** 2 + (self.y - y_test) ** 2)
                    if min_dist is None or dist < min_dist:
                        closest = (x_test, y_test)
                        min_dist = dist
            if closest is None:
                rospy.loginfo("ERROR: could not find free point near vendor")
                self.replan()
                return
            """
            vendor_th = vendor_location.theta
            pad_d = 0.35  # distance away from the vendor that we want to be
            closest = (vendor_location.x - pad_d * np.cos(vendor_th), vendor_location.y - pad_d * np.cos(vendor_th))
            """
            self.x_g = closest[0]
            self.y_g = closest[1]
            # Face the vendor
            self.theta_g = np.arctan2(vendor_location.y - self.y_g, vendor_location.x - self.x_g)
            rospy.loginfo("Plan to go to the vendor: {}, location: ({}, {})".format(vendor_location.identifier, self.x_g, self.y_g))
            rospy.loginfo("Replanning because of new delivery request")
            # visualize the goal
            self.trans_broadcaster.sendTransformMessage(
                self.create_transform_msg(self.x_g, self.y_g, self.theta_g, "goal_pose", "map"))
            self.replan()

    def init_stop(self):
        self.switch_mode(Mode.STOP)
        self.stop_start_time = rospy.get_rostime()

    def init_stop_sign(self):
        self.switch_mode(Mode.STOP_SIGN)
        self.stop_start_time = rospy.get_rostime()

    def init_cross(self):
        self.cross_start_time = rospy.get_rostime()
    
    def has_crossed(self):
        return rospy.get_rostime() - self.cross_start_time > rospy.Duration.from_sec(self.cross_time)

    def has_stopped(self):
        return (self.mode == Mode.STOP or self.mode == Mode.STOP_SIGN) and \
               rospy.get_rostime() - self.stop_start_time > rospy.Duration.from_sec(self.stop_time)

    def plan_with_window(self):
         # Attempt to plan a path
        state_min = self.snap_to_grid((-self.plan_horizon, -self.plan_horizon))
        state_max = self.snap_to_grid((self.plan_horizon, self.plan_horizon))
        x_init = self.snap_to_grid((self.x, self.y))
        self.plan_start = x_init
        x_goal = self.snap_to_grid((self.x_g, self.y_g))

        padding = 0
        planned_path = []
        traj_new = []
        t_new = []
        time = rospy.get_rostime()
        if time > self.latest_request_time:
            self.latest_request_time
        while padding < 5 and time >= self.latest_request_time:
            # Avoid stuck
            self.publish_control()
            rospy.loginfo("Navigator: computing navigation plan with window_size={}".format(self.map_window+padding))
            problem = AStar(state_min,state_max,x_init,x_goal,self.occupancy,self.plan_resolution, padding)
            success =  problem.solve()
            self.publish_control()
            if not success:
                rospy.loginfo("Planning failed")
                if len(planned_path) > 0:
                    rospy.loginfo("Use non-smoothed traj as backup.")
                    traj_new, t_new = compute_smoothed_traj(planned_path, self.v_des, 0, self.traj_dt)
                    return (planned_path, traj_new, t_new)
                return
            rospy.loginfo("Planning Succeeded")

            planned_path = problem.path
        
            self.publish_control()
            # Check whether path is too short
            if len(planned_path) < 4:
                rospy.loginfo("Path too short to track")
                self.switch_mode(Mode.PARK)
                return

            # Smooth and generate a trajectory
            traj_new, t_new = compute_smoothed_traj(planned_path, self.v_des, self.spline_alpha, self.traj_dt)

            self.publish_control()
            # Check if smooth path is collision free.
            is_traj_free = True
            for state in traj_new:
                if not self.is_free(state[0:2]):
                    is_traj_free = False
                    break
            if is_traj_free:
                return (planned_path, traj_new, t_new)
            self.publish_control()
            rospy.loginfo("Smoothed traj is not free. Try different window size.")
            padding +=2
        rospy.loginfo("No good solution, use non-smoothed path.")
        self.publish_control()
        traj_new, t_new = compute_smoothed_traj(planned_path, self.v_des, 0, self.traj_dt)
        self.publish_control()
        return (planned_path, traj_new, t_new)
        
    def replan(self):
        """
        loads goal into pose controller
        runs planner based on current pose
        if plan long enough to track:
            smooths resulting traj, loads it into traj_controller
            sets self.current_plan_start_time
            sets mode to ALIGN
        else:
            sets mode to PARK
        """
        # Make sure we have a map
        if not self.occupancy:
            rospy.loginfo("Navigator: replanning canceled, waiting for occupancy map.")
            self.switch_mode(Mode.IDLE)
            return

        best_effort = self.plan_with_window()
        if best_effort == None:
            if len(self.current_plan) == 0:
                # visualize a marker
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = rospy.Time()
                marker.id = 0
                marker.type = 1

                marker.pose.position.x = self.x_g
                marker.pose.position.y = self.y_g
                marker.pose.position.z = 0

                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0

                marker.scale.x = self.map_window*self.map_resolution
                marker.scale.y = self.map_window*self.map_resolution
                marker.scale.z = self.map_window*self.map_resolution

                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0

                marker.lifetime = rospy.Duration.from_sec(2)
                self.goal_marker_pub.publish(marker)
                vel_g_msg = Twist()
                self.nav_vel_pub.publish(vel_g_msg)
                self.switch_mode(Mode.IDLE)
            return
        planned_path, traj_new, t_new = best_effort

        # Check if current plan is still collision free
        is_traj_free = True
        if len(self.current_plan) > 0:
            for state in self.current_plan:
                if not self.is_free(state[0:2]):
                    is_traj_free = False
                    break
        
        # If currently tracking a trajectory, check whether new trajectory will take more time to follow
        if self.mode == Mode.TRACK and is_traj_free:
            t_remaining_curr = self.current_plan_duration - self.get_current_plan_time()

            # Estimate duration of new trajectory
            th_init_new = traj_new[0,2]
            th_err = wrapToPi(th_init_new - self.theta)
            t_init_align = abs(th_err/self.om_max)
            t_remaining_new = t_init_align + t_new[-1]

            if t_remaining_new > t_remaining_curr:
                rospy.loginfo("New plan rejected (longer duration than current plan)")
                self.publish_smoothed_path(traj_new, self.nav_smoothed_path_rej_pub)
                return

        # Otherwise follow the new plan
        rospy.loginfo("Follow new plan")
        self.publish_planned_path(planned_path, self.nav_planned_path_pub)
        self.publish_smoothed_path(traj_new, self.nav_smoothed_path_pub)

        self.pose_controller.load_goal(self.x_g, self.y_g, self.theta_g)
        self.traj_controller.load_traj(t_new, traj_new)

        self.current_plan = traj_new
        self.current_plan_start_time = rospy.get_rostime()
        self.current_plan_duration = t_new[-1]

        self.th_init = traj_new[0,2]
        self.heading_controller.load_goal(self.th_init)

        if not self.aligned():
            rospy.loginfo("Not aligned with start direction")
            self.switch_mode(Mode.ALIGN)
            return

        rospy.loginfo("Ready to track")
        self.switch_mode(Mode.TRACK)

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            # try to get state information to update self.x, self.y, self.theta
            try:
                (translation,rotation) = self.trans_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
                self.x = translation[0]
                self.y = translation[1]
                euler = tf.transformations.euler_from_quaternion(rotation)
                self.theta = euler[2]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                self.current_plan = []
                rospy.loginfo("Navigator: waiting for state info")
                self.switch_mode(Mode.IDLE)
                print e
                pass

            # STATE MACHINE LOGIC
            # some transitions handled by callbacks
            if self.mode == Mode.IDLE:
                pass
            elif self.mode == Mode.TO_NEXT:
                self.plan_to_next_goal_location()
            elif self.mode == Mode.ALIGN:
                if self.aligned():
                    self.current_plan_start_time = rospy.get_rostime()
                    self.switch_mode(Mode.TRACK)
            elif self.mode == Mode.TRACK:
                if self.near_goal():
                    self.switch_mode(Mode.PARK)
                elif not self.close_to_plan_start():
                    rospy.loginfo("replanning because far from start")
                    self.replan()
                elif (rospy.get_rostime() - self.current_plan_start_time).to_sec() > self.current_plan_duration:
                    rospy.loginfo("replanning because out of time")
                    self.replan() # we aren't near the goal but we thought we should have been, so replan
            elif self.mode == Mode.PARK:
                if self.at_goal():
                    if len(self.delivery_request_locations) == 0 and \
                            (self.origin_x is None or self.x_g == self.origin_x):
                        # Forget about the goal and origin
                        self.origin_x = None
                        self.origin_y = None
                        self.origin_theta = None
                        self.x_g = None
                        self.y_g = None
                        self.theta_g = None
                        self.switch_mode(Mode.IDLE)
                    else:
                        self.init_stop()
            elif self.mode == Mode.STOP:
                if self.has_stopped():
                    rospy.loginfo("Stop for enough time, plan to the next location!!!!!")
                    self.switch_mode(Mode.TO_NEXT)
                else:
                    self.mode = Mode.STOP
                    # self.switch_mode(Mode.STOP)
            elif self.mode == Mode.STOP_SIGN:
                if self.has_stopped():
                    rospy.loginfo("Pass the stop sign")
                    self.init_cross()
                    self.replan()
                else:
                    self.switch_mode(Mode.STOP_SIGN)

            self.publish_control()
            rate.sleep()

if __name__ == '__main__':    
    nav = Navigator()
    rospy.on_shutdown(nav.shutdown_callback)
    nav.run()
