#!/usr/bin/env python

#Title: Python Subscriber for Tank Navigation
#Author: Khairul Izwan Bin Kamsani - [23-01-2020]
#Description: Tank Navigation Subcriber Nodes (Python)

#remove or add the library/libraries for ROS
import rospy
import sys
import cv2
import imutils
import argparse

#remove or add the message type
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import RegionOfInterest

class TankFaceDetector:

	def __init__(self):
		""" Initializing your ROS Node """
		rospy.init_node("face_detector_node", anonymous=True)

		rospy.on_shutdown(self.shutdown)

		""" Create the cv_bridge object """
		self.bridge = CvBridge()

		self.sub = rospy.Subscriber("/cv_camera/image_raw",Image, self.callback)

		self.pub = rospy.Publisher("/roi",RegionOfInterest,queue_size=10)

		self.faceCascade = cv2.CascadeClassifier("/home/pi/catkin_ws/src/tank_camera/library/haarcascade_frontalface_default.xml")
	
	def callback(self, data):
		""" convert ros --> opencv """
		self.convert_ros_to_opencv_img(data)
		
		""" detect face """
		self.track()

		#rospy.loginfo(self.rects)
		frameClone = self.cv_image.copy()

		""" loop over the face bounding boxes and draw them """
		for rect in self.rects:
			# cv2.rectangle(frameClone, (fX, fY), (fX + fW, fY + fH), (0, 255, 0), 2)
			cv2.rectangle(frameClone, (rect[0], rect[1]), (rect[2], rect[3]), (0, 255, 0), 2)

			roi=RegionOfInterest()
			roi.x_offset=rect[0]
			roi.y_offset=rect[1]
			roi.width=rect[2]
			roi.height=rect[3]

			self.pub.publish(roi)

		cv2.imshow("Face", frameClone)
		cv2.waitKey(1)
	
	def convert_ros_to_opencv_img(self, ros_image):
		self.cv_image = self.bridge.imgmsg_to_cv2(ros_image)

	def track(self):
		""" detect faces in the image and initialize the list of rectangles containing the faces and eyes """
		faceRects = self.faceCascade.detectMultiScale(self.cv_image,
			scaleFactor = 1.1, minNeighbors = 5, minSize = (30, 30),
			flags = cv2.CASCADE_SCALE_IMAGE)
		self.rects = []

		""" loop over the face bounding boxes """
		for (fX, fY, fW, fH) in faceRects:
			""" extract the face ROI and update the list of bounding boxes """
			faceROI = self.cv_image[fY:fY + fH, fX:fX + fW]
			self.rects.append((fX, fY, fX + fW, fY + fH))
			

		# return the rectangles representing bounding
		# boxes around the faces and eyes
		# return rects

	def shutdown(self):
		try:
			rospy.loginfo("[INFO] Tank Face Detector [OFFLINE]")
		finally:
			cv2.destroyAllWindows()

def main(args):
	tfd = TankFaceDetector()
	try:
		rospy.spin()
	except ROSInterruptException:
		rospy.loginfo("[INFO] Tank Face Detector [OFFLINE]")

	cv2.destroyAllWindows()

if __name__ == "__main__":
	rospy.loginfo("[INFO] Tank Face Detector [ONLINE]")
	main(sys.argv)
