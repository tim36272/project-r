#!/usr/bin/env python
import rospy
from dhs.msg import blob

class blob_tracker:
	def callback(self,data):
		


def listener():
	rospy.init_node('listener', anonymous=True)
	the_blobs = blob_tracker()
	rospy.Subscriber("/blobs_in", blob, the_blobs.callback)
	rospy.spin()


if __name__ == '__main__':
	listener()
