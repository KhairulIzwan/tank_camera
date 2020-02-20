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
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import RegionOfInterest

class TankFaceDetector:

	def __init__(self):
		# Create an empty arrays for save rects value later
		self.rects = []
		
		# Initializing your ROS Node
		rospy.init_node("face_detector_node", anonymous=True)

		rospy.on_shutdown(self.shutdown)

		# Create the cv_bridge object
		self.bridge = CvBridge()

		# Create the Subsciber (image_raw)
		self.sub = rospy.Subscriber("/cv_camera/image_raw", Image, self.callback_image)
		
		# Create the Subsciber (camera_info)
		self.sub = rospy.Subscriber("/cv_camera/camera_info", CameraInfo, self.callback_camerainfo)
		
		# Create the Publisher (roi)		
		self.pub = rospy.Publisher("/roi", RegionOfInterest, queue_size=10)

		# Path to input Haar cascade for face detection
		self.faceCascade = cv2.CascadeClassifier("/home/pi/catkin_ws/src/tank_camera/library/haarcascade_frontalface_default.xml")
		
	def callback_camerainfo(self, data):
		# Get the image width and height
		self.w = data.width
		self.h = data.height
	
	def callback_image(self, data):
		# Convert ros --> opencv
		self.convert_ros_to_opencv_img(data)
		
		# Detect face
		self.track()
	
	def convert_ros_to_opencv_img(self, ros_image):
		self.cv_image = self.bridge.imgmsg_to_cv2(ros_image)
		
		# Clone the original image for displaying purpose later
		self.frameClone = self.cv_image.copy()

	def track(self):
		# Detect all faces in the input frame
		faceRects = self.faceCascade.detectMultiScale(self.cv_image,
			scaleFactor = 1.05, minNeighbors = 9, minSize = (30, 30),
			flags = cv2.CASCADE_SCALE_IMAGE)

		# check to see if a face was found
		if len(faceRects) > 0:
			# extract the bounding box coordinates of the face and
			# use the coordinates to determine the center of the
			# face
			(x, y, self.w, self.h) = faceRects[0]
			faceX = int(x + (self.w / 2.0))
			faceY = int(y + (self.h / 2.0))
			
			# Extract the bounding box and draw it
			if faceRects is not None:
				(x, y, w, h) = faceRects[0]
				cv2.rectangle(self.frameClone, (x, y), (x + w, y + h), (0, 255, 0),
				2)

				roi=RegionOfInterest()
				roi.x_offset=faceRects[0][0]
				roi.y_offset=faceRects[0][1]
				roi.width=faceRects[0][2]
				roi.height=faceRects[0][3]

				self.pub.publish(roi)
		
		# display the frame to the screen
		cv2.imshow("Face", self.frameClone)
		cv2.waitKey(1)
			
			
			
		

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
