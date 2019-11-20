#!/usr/bin/env python
from __future__ import print_function
import thread
import rospy
import rosbag
import os
import cv2
import numpy as np
import yaml
import copy

from duckietown import DTROS
from duckietown_utils.jpg import bgr_from_jpg
from sensor_msgs.msg import CompressedImage, CameraInfo
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError

# TODO Remove leftover code from camera node and make this its own node (camer_mock) and launchfile.
# TODO Publish '~camera_info' from bagfile as well if available.

class CameraNode(DTROS):
    # TODO Write node doc (copy from other node as template)

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(CameraNode, self).__init__(node_name=node_name)

        # Add the node parameters to the parameters dictionary and load their default values
        #self.parameters['~framerate'] = None
        self.parameters['~res_w'] = None
        self.parameters['~res_h'] = None
        #self.parameters['~exposure_mode'] = None
        rospy.set_param('~bag_path', '/data/bags/4-way-straight.bag')
        self.parameters['~bag_path'] = None
        rospy.set_param('~time_scaler', 1.0)
        self.parameters['~time_scaler'] = None
        self.updateParameters()
        self.refresh_parameters()

        # Setup publishers
        self.has_published = False
        self.pub_img = self.publisher("~image/compressed", CompressedImage, queue_size=1)
        self.pub_camera_info = self.publisher("~camera_info", CameraInfo, queue_size=1)
        self.pub_loop = self.publisher("~loop", Bool, queue_size=1)

        self.initialize_camera_info()
        self.updateCameraParameters()

        self.bridge = CvBridge()

        self.log("Initialized.")


    def publishBagMessage(self, bag_msg):
        timestamp = bag_msg.timestamp
        msg = bag_msg.message

        # Write current time into header
        msg.header.stamp = rospy.Time.now()

        resized_msg = self.resize_img_msg(msg)

        self.log("Publishing message. (time={})".format(timestamp))
        self.pub_img.publish(resized_msg)

        return timestamp

    def resize_img_msg(self, img_msg):
        try:
            img_original = bgr_from_jpg(img_msg.data)
        except ValueError as e:
            rospy.logerr(e)
            return None

        img_resized = cv2.resize(img_original, (self.res_w, self.res_h))

        try:
            resized_msg = self.bridge.cv2_to_compressed_imgmsg(img_resized, dst_format = "jpg")
        except CvBridgeError as e:
            rospy.logerr(e)
            return None

        resized_msg.header = img_msg.header
        return resized_msg

    def playBagMessages(self, bag_msgs):
        first_msg = next(bag_msgs)
        timestamp = first_msg.timestamp

        last_timestamp = self.publishBagMessage(first_msg)

        for bag_msg in bag_msgs:
            timestamp = bag_msg.timestamp
            img_msg = bag_msg.message

            sleep_duration = timestamp - last_timestamp
            rospy.sleep(sleep_duration * self.time_scaler)

            # Publish message from bag and CameraInfo
            last_timestamp = self.publishBagMessage(bag_msg)
            self.pub_camera_info.publish(self.current_camera_info)

            if self.parametersChanged or self.is_shutdown or rospy.is_shutdown():
                break


    def publishReset(self):
        msg = Bool()
        msg.data = True
        self.pub_loop.publish(msg)


    def startPublishing(self):
        self.log("Start publishing frames from bag '{}'.".format(self.bag_path))

        while not (self.is_shutdown or rospy.is_shutdown()):
            try:
                with rosbag.Bag(self.bag_path) as bag:
                    #messages = bag.read_messages(topics=[topic_to_process])
                    messages = bag.read_messages()
                    start_time = rospy.Time.from_sec(bag.get_start_time())

                    topics = bag.get_type_and_topic_info().topics.keys()

                    # String to find topic of form '/[duckiebot name]/camera_node/image/compressed'
                    topic_match = '/camera_node/image/compressed'

                    # TODO filter by type as well
                    topics_filtered = [t for t in topics if topic_match in t]

                    if len(topics_filtered) == 0:
                        self.log("Can't find topic containing '{}' in bag.".format(topic_match), 'err')
                        # Sleep to not spam the log
                        rospy.sleep(rospy.Duration.from_sec(1))
                        continue

                    # Send reset signal
                    self.publishReset()

                    topic_to_publish = topics_filtered[0]
                    self.log("Publishing frames from topic '{}'.".format(topic_to_publish))
                    bag_msgs = bag.read_messages(topics=[topic_to_publish])
                    self.playBagMessages(bag_msgs)
            except IOError:
                self.log("Can't open bag file at path '{}'. Check if it exists and is accessible for this node.".format(self.bag_path), 'err')
                # Sleep to not spam the log
                rospy.sleep(rospy.Duration.from_sec(1))

            if self.parametersChanged:
                self.refresh_parameters()
                self.updateCameraParameters()
                self.parametersChanged = False
                self.log("Parameters updated.")

        self.log("Publishing ended.")


    def initialize_camera_info(self):
        self.cali_file_folder = '/data/config/calibrations/camera_intrinsic/'
        self.frame_id = rospy.get_namespace().strip('/') + '/camera_optical_frame'
        self.cali_file = self.cali_file_folder + rospy.get_namespace().strip("/") + ".yaml"

        # Locate calibration yaml file or use the default otherwise
        if not os.path.isfile(self.cali_file):
            self.log("Can't find calibration file: %s.\n Using default calibration instead."
                          % self.cali_file, 'warn')
            self.cali_file = (self.cali_file_folder + "default.yaml")

        # Shutdown if no calibration file not found
        if not os.path.isfile(self.cali_file):
            rospy.signal_shutdown("Found no calibration file ... aborting")

        # Load the calibration file
        self.original_camera_info = self.loadCameraInfo(self.cali_file)
        self.original_camera_info.header.frame_id = self.frame_id
        self.current_camera_info = copy.deepcopy(self.original_camera_info)


    def updateCameraParameters(self):
        scale_width = float(self.parameters['~res_w']) / self.original_camera_info.width
        scale_height = float(self.parameters['~res_h']) / self.original_camera_info.height

        scale_matrix = np.ones(9)
        scale_matrix[0] *= scale_width
        scale_matrix[2] *= scale_width
        scale_matrix[4] *= scale_height
        scale_matrix[5] *= scale_height

        # Adjust the camera matrix resolution
        self.current_camera_info.height = self.parameters['~res_h']
        self.current_camera_info.width = self.parameters['~res_w']

        # Adjust the K matrix
        self.current_camera_info.K = np.array(self.original_camera_info.K) * scale_matrix

        # Adjust the P matrix (done by Rohit)
        scale_matrix = np.ones(12)
        scale_matrix[0] *= scale_width
        scale_matrix[2] *= scale_width
        scale_matrix[5] *= scale_height
        scale_matrix[6] *= scale_height
        self.current_camera_info.P = np.array(self.original_camera_info.P) * scale_matrix


    def loadCameraInfo(self, filename):
        """Loads the camera calibration files.
        Loads the intrinsic and extrinsic camera matrices.
        Args:
            filename (:obj:`str`): filename of calibration files.
        Returns:
            :obj:`CameraInfo`: a CameraInfo message object
        """
        stream = file(filename, 'r')
        calib_data = yaml.load(stream)
        cam_info = CameraInfo()
        cam_info.width = calib_data['image_width']
        cam_info.height = calib_data['image_height']
        cam_info.K = calib_data['camera_matrix']['data']
        cam_info.D = calib_data['distortion_coefficients']['data']
        cam_info.R = calib_data['rectification_matrix']['data']
        cam_info.P = calib_data['projection_matrix']['data']
        cam_info.distortion_model = calib_data['distortion_model']
        return cam_info


    def refresh_parameters(self):
        self.bag_path = self.parameters['~bag_path']
        self.time_scaler = self.parameters['~time_scaler']
        self.res_w = self.parameters['~res_w']
        self.res_h = self.parameters['~res_h']


if __name__ == '__main__':
    # Initialize the node
    camera_node = CameraNode(node_name='camera')
    # Start the image capturing in a separate thread
    thread.start_new_thread(camera_node.startPublishing, ())
    # Keep it spinning to keep the node alive
    rospy.spin()
