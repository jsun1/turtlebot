#!/usr/bin/env python

import rospy
import os
from asl_turtlebot.msg import DetectedObject, DetectedObjectList, VendorLocation
from nav_msgs.msg import Odometry
import numpy as np
from scipy.stats import norm
import tf
from visualization_msgs.msg import Marker
import math

class FoodVendorLocator:

    def __init__(self):
        rospy.init_node('project_main', anonymous=True)
        self.vendor_locations = {} # vendor id to x,y location
        self.x = 0
        self.y = 0
        self.theta = 0
        self.vendor_loc_meas = {}  # vendor id to list of x y locations
        self.approved = ['apple', 'orange', 'banana', 'pizza', 'stop_sign']

        rospy.Subscriber('/detector/objects', DetectedObjectList, self.object_detected_callback, queue_size=1, buff_size=2**24)

        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.filtered_detected_objects_pub = rospy.Publisher('/project/detected_objects', DetectedObjectList, queue_size=10)
        self.vendor_location_pub = rospy.Publisher('/vendor_location', VendorLocation, queue_size=10)
        self.detect_marker_pub = {}
        for vendor in self.approved:
            self.detect_marker_pub[vendor] = rospy.Publisher('/marker_' + vendor, Marker, queue_size=10)

    def convert_to_world(self, distance, angle):
        x_world = self.x + distance * np.cos(self.theta + angle)
        y_world = self.y + distance * np.sin(self.theta + angle)
        return x_world, y_world

    def get_max_likelihood(self, identifier):
        if identifier in self.vendor_loc_meas:
            x_meas = np.array(self.vendor_loc_meas[identifier])[:, 0]
            y_meas = np.array(self.vendor_loc_meas[identifier])[:, 1]
            x_mean, x_stdev = norm.fit(x_meas)
            y_mean, y_stdev = norm.fit(y_meas)
            return (x_mean, y_mean), (x_stdev, y_stdev)
        else:
            return None

    def max_likelihood(self, x_detect, y_detect, identifier):
        # Extension: use a normal distribution to fit the x and y locations
        if identifier in self.vendor_loc_meas:
            self.vendor_loc_meas[identifier].append((x_detect, y_detect))
        else:
            self.vendor_loc_meas[identifier] = [(x_detect, y_detect)]
        return self.get_max_likelihood(identifier)

    def average_angles(self, angles):
        """Average (mean) of angles

        Return the average of an input sequence of angles. The result is between
        ``0`` and ``2 * math.pi``.
        If the average is not defined (e.g. ``average_angles([0, math.pi]))``,
        a ``ValueError`` is raised.
        """

        x = sum(math.cos(a) for a in angles)
        y = sum(math.sin(a) for a in angles)

        if x == 0 and y == 0:
            raise ValueError(
                "The angle average of the inputs is undefined: %r" % angles)

        # To get outputs from -pi to +pi, delete everything but math.atan2() here.
        return math.fmod(math.atan2(y, x) + 2 * math.pi, 2 * math.pi)

    def object_detected_callback(self, msg):
        #self.filtered_detected_objects_pub.publish(msg)
        #return
        objects = msg.objects  # stop_sign, bed, etc (list)
        ob_msgs = msg.ob_msgs
        # filtered_objects = DetectedObjectList()
        # for ob_msg in ob_msgs:
        for i in range(0, len(objects)):
            obj = objects[i]
            ob_msg = ob_msgs[i]
            confidence = ob_msg.confidence
            if (confidence > 0.5) and ob_msg.name in self.approved:
                distance = ob_msg.distance # workaround for location issue
                # Take the wrapping average of the left and right angles
                theta = self.average_angles([ob_msg.thetaleft, ob_msg.thetaright])
                # Translate this to coordinates in world frame
                x_obj, y_obj = self.convert_to_world(distance, theta)
                (x_obj, y_obj), stdev = self.max_likelihood(x_obj, y_obj, ob_msg.name)
                #rospy.loginfo("Max likelihood {}, ({}, {}) stdev {}".format(ob_msg.name, x_obj, y_obj, stdev))
                
                #publish the x, y position of the object along with its identifier
                vendor_location = VendorLocation()
                vendor_location.identifier = ob_msg.name
                vendor_location.x = x_obj
                vendor_location.y = y_obj
                vendor_location.theta = self.theta  # np.arctan2(vendor_location.y - self.y, vendor_location.x - self.x)
                vendor_location.distance = ob_msg.distance
                self.vendor_location_pub.publish(vendor_location)
                # filtered_objects.objects.append(obj)
                # filtered_objects.ob_msgs.append(ob_msg)
        # if len(filtered_objects.ob_msgs) > 0:
        # self.filtered_detected_objects_pub.publish(filtered_objects)

    def show_all_approved(self):
        for vendor in self.approved:
            if vendor in self.vendor_loc_meas:
                (x_v, y_v), _ = self.get_max_likelihood(vendor)
                #visualize a marker
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = rospy.Time()
                marker.id = 0
                marker.type = 1

                marker.pose.position.x = x_v
                marker.pose.position.y = y_v
                marker.pose.position.z = 0

                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0

                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1

                marker.color.a = 0.7
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0

                marker.lifetime = rospy.Duration.from_sec(2)
                self.detect_marker_pub[vendor].publish(marker)

    def odom_callback(self, msg):
        pose = msg.pose.pose#.position
        self.x = pose.position.x
        self.y = pose.position.y
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.theta = euler[2]
        self.show_all_approved()

    def run(self):
        rospy.spin()

if __name__=='__main__':
    locator = FoodVendorLocator()
    locator.run()
