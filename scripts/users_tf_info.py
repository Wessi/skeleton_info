#!/usr/bin/env python
import rospy
from tf2_msgs.msg import TFMessage
from icog_tf.msg import tfBundle
from icog_tf.msg import tfBundles
import re

class CTFBundle:
	def __init__(self):
		self.frames = {}

	# Returns True if all frames have been captured
	def isComplete(self):
		return all([frame in self.frames for frame in ["head", "neck", "torso", 
			"right_elbow", "right_foot", "right_hand", "right_hip", "right_knee", "right_shoulder",
			"left_elbow", "left_foot", "left_hand", "left_hip", "left_knee", "left_shoulder"]])

	# Generates a publishable message	
	def getMessage(self):
		self.msg = tfBundle()
		self.msg.user_id = self.user_id

		self.msg.head = self.frames["head"]
		self.msg.neck = self.frames["neck"]
		self.msg.torso = self.frames["torso"]

		self.msg.right_elbow = self.frames["right_elbow"]
		self.msg.right_foot = self.frames["right_foot"]
		self.msg.right_hand = self.frames["right_hand"]
		self.msg.right_hip = self.frames["right_hip"]
		self.msg.right_knee = self.frames["right_knee"]
		self.msg.right_shoulder = self.frames["right_shoulder"]

		self.msg.left_elbow = self.frames["left_elbow"]
		self.msg.left_foot = self.frames["left_foot"]
		self.msg.left_hand = self.frames["left_hand"]
		self.msg.left_hip = self.frames["left_hip"]
		self.msg.left_knee = self.frames["left_knee"]
		self.msg.left_shoulder = self.frames["left_shoulder"]
		
		return self.msg

def callback(data):
	timestamp = int(data.transforms[0].header.stamp.secs)
	frame_str = data.transforms[0].child_frame_id
	user_id = re.search('/tracker/(.+?)/', frame_str)
	transform_str = data.transforms[0].transform

	if user_id:
		user_id = user_id.group(1)
		
		# Add new user if it doesn't exist
		if not user_id in current_bundles:
			current_bundles[user_id] = CTFBundle()

		current_bundles[user_id].user_id = user_id
		current_bundles[user_id].timestamp = timestamp

		# Extract frame
		frame_id = re.search(user_id + '/(.+?)$', frame_str)
		if frame_id:
			frame_id = frame_id.group(1)
			# Add frame and its transform to current bundle
			current_bundles[user_id].frames [frame_id] = transform_str	

	# Remove bundles with old timestamps
	for user, bundle in current_bundles.items():
		if (timestamp - bundle.timestamp > 3):
			del current_bundles[user]

	# Publish to /skeleton_info_topic
	tf_bundles_msg = tfBundles()
	tf_bundles_msg.tf_bundles = []
	for user, bundle in current_bundles.items():
		if (bundle.isComplete()):
			tf_bundles_msg.tf_bundles.append(bundle.getMessage())
	publisher.publish(tf_bundles_msg)

current_bundles = {}
publisher = rospy.Publisher('skeleton_info_topic', tfBundles, queue_size=10)

if __name__ == '__main__':
	rospy.init_node('skeleton_info_node', anonymous=False)   
	rospy.Subscriber("tf", TFMessage, callback)
	rospy.spin()
