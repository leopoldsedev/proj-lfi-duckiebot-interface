#!/usr/bin/env python
from __future__ import print_function
import thread
import rospy
import rosbag

from duckietown import DTROS
from sensor_msgs.msg import CompressedImage, CameraInfo

# TODO Remove leftover code from camera node and make this its own node (camer_mock) and launchfile.
# TODO Publish '~camera_info' from bagfile as well if available.

class CameraNode(DTROS):
    # TODO Write node doc (copy from other node as template)

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(CameraNode, self).__init__(node_name=node_name)

        # Add the node parameters to the parameters dictionary and load their default values
        #self.parameters['~framerate'] = None
        #self.parameters['~res_w'] = None
        #self.parameters['~res_h'] = None
        #self.parameters['~exposure_mode'] = None
        rospy.set_param('~bag_path', '/data/bags/4-way-straight.bag')
        self.parameters['~bag_path'] = None
        rospy.set_param('~time_scaler', 1.0)
        self.parameters['~time_scaler'] = None
        self.updateParameters()

        # Setup publishers
        self.has_published = False
        self.pub_img = self.publisher("~image/compressed", CompressedImage, queue_size=1)
        self.pub_camera_info = self.publisher("~camera_info", CameraInfo, queue_size=1)

        self.bag_path = self.parameters['~bag_path']
        self.time_scaler = self.parameters['~time_scaler']
        # String to find topic of form '/[duckiebot name]/camera_node/image/compressed'
        self.topic_match = '/camera_node/image/compressed'

        self.log("Initialized.")

    def publishBagMessage(self, bag_msg):
        timestamp = bag_msg.timestamp
        msg = bag_msg.message

        self.log("Publishing message. (time={})".format(timestamp))
        self.pub_img.publish(msg)

        return timestamp

    def playBagMessages(self, bag_msgs):
        first_msg = next(bag_msgs)
        timestamp = first_msg.timestamp

        last_timestamp = self.publishBagMessage(first_msg)

        for bag_msg in bag_msgs:
            timestamp = bag_msg.timestamp
            img_msg = bag_msg.message

            sleep_duration = timestamp - last_timestamp
            rospy.sleep(sleep_duration * self.time_scaler)

            last_timestamp = self.publishBagMessage(bag_msg)

            if self.parametersChanged or self.is_shutdown or rospy.is_shutdown():
                break

    def startPublishing(self):
        self.log("Start publishing frames from bag '{}'.".format(self.bag_path))

        while not (self.is_shutdown or rospy.is_shutdown()):
            try:
                with rosbag.Bag(self.bag_path) as bag:
                    #messages = bag.read_messages(topics=[topic_to_process])
                    messages = bag.read_messages()
                    start_time = rospy.Time.from_sec(bag.get_start_time())

                    topics = bag.get_type_and_topic_info().topics.keys()
                    # TODO filter by type as well
                    topics_filtered = [t for t in topics if self.topic_match in t]

                    if len(topics_filtered) == 0:
                        self.log("Can't find topic containing '{}' in bag.".format(self.topic_match), 'err')
                        # Sleep to not spam the log
                        rospy.sleep(rospy.Duration.from_sec(1))
                        continue

                    topic_to_publish = topics_filtered[0]
                    self.log("Publishing frames from topic '{}'.".format(topic_to_publish))
                    bag_msgs = bag.read_messages(topics=[topic_to_publish])
                    self.playBagMessages(bag_msgs)
            except IOError:
                self.log("Can't open bag file at path '{}'. Check if it exists and is accessible for this node.".format(self.bag_path), 'err')
                # Sleep to not spam the log
                rospy.sleep(rospy.Duration.from_sec(1))

            if self.parametersChanged:
                self.bag_path = self.parameters['~bag_path']
                self.time_scaler = self.parameters['~time_scaler']
                self.parametersChanged = False
                self.log("Parameters updated.")

        self.log("Publishing ended.")


if __name__ == '__main__':
    # Initialize the node
    camera_node = CameraNode(node_name='camera')
    # Start the image capturing in a separate thread
    thread.start_new_thread(camera_node.startPublishing, ())
    # Keep it spinning to keep the node alive
    rospy.spin()
