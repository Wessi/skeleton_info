#!/usr/bin/env python
import rospy
from tf2_msgs.msg import TFMessage
from icog_tf.msg import users
import re

def callback(data):
	timestamp = int(data.transforms[0].header.stamp.secs)
	frame_id = data.transforms[0].child_frame_id
	user_id = re.search('/tracker/(.+?)/', frame_id)
	if user_id:
		user_id = user_id.group(1)
		current_users[user_id] = timestamp

	# Remove all users with old timestamps
	for user,user_timestamp in current_users.items():
		if (timestamp - user_timestamp > 3):
			del current_users[user]

	# Publish Users
	users_msg = users()
	users_msg.user_ids = []
	for user,user_timestamp in current_users.items():
		users_msg.user_ids.append(user)
	publisher.publish(users_msg)


current_users = {}
publisher = rospy.Publisher('users_bundle_topic', users, queue_size=10)

if __name__ == '__main__':
	rospy.init_node('user_bundler_node', anonymous=True)   
	rospy.Subscriber("tf", TFMessage, callback)
	rospy.spin()
