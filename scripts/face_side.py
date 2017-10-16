#!/usr/bin/python

import keras
from keras.models import load_model
import numpy as np
import h5py
import cv2 
import rospy
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8

image_row = 40
image_col = 40
model = load_model('/home/wade/catkin_ws/src/face_detection/model/face_angle_70_110_middle_model.h5')
#model = load_model('/home/wade/catkin_ws/src/Faceside/face_angle_80_100_middle_model.h5')
class FaceSide:
	def __init__(self):
		#print '~~~~~~~~1'
		self.image_sub = rospy.Subscriber("/face", Image, self.callback)
		self.side_pub = rospy.Publisher("/face_side", UInt8, queue_size=1)
		self.value = 1
		self.bridge = CvBridge()

        
        #self.image_pub = rospy.Publisher("test_image", Image)

	def callback(self,data):
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			#print '~~~~~~~~2'
			print(e)
		#self.img = self.bridge.imgmsg_to_cv2(data,"bgr8")
		self.img_gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
		self.img_standard =cv2.resize(self.img_gray,(image_row,image_col),interpolation=cv2.INTER_CUBIC)

		self.img_predict = self.img_standard.reshape(1, 1, image_row, image_col)

		self.value_data = model.predict(self.img_predict, batch_size=32, verbose=0)
		if self.value_data[0][0] > 0.9:
			self.value = 0
		elif self.value_data[0][2] >0.9:
			self.value = 2
		else:
			self.value = 1

		print self.value
		self.side_pub.publish(self.value)
		
		#print self.value_data


if __name__ == '__main__':
	face_side = FaceSide()
	rospy.init_node('face_side')
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"










