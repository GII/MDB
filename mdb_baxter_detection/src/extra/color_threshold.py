#!/usr/bin/env python

import rospy, sys, cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage

def nothing(x):
	pass

class color_threshold:
	def __init__(self):
		rospy.init_node('color_threshold') # Init node
		self.bridge = CvBridge() # Create the cv_bridge object
		
		# Subscribers
		#self.image_sub = rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.image_callback)
		self.camera_image_sb = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.camera_image_cb)
		

	# Callbacks	
	def image_callback(self, ros_image):
		frame=self.imageconv(ros_image)
		self.core(frame)

	def camera_image_cb(self, img):
		np_arr = np.fromstring(img.data, np.uint8)
		frame = cv2.flip(cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR), 1)
		self.core(frame)

	# Other functions
	def imageconv (self, ros_image):
		# Use cv_bridge() to convert the ROS image to OpenCV format
		try:    
			imagecv = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
		except CvBridgeError, e:
			print e
		# Convert the image to a Numpy array since most cv2 functions require Numpy arrays.
		frame = np.array(imagecv, dtype=np.uint8)
		return frame
	
	def setValues(self, values):
		#Save the color threshold values as ROS parameter server parameters
		rospy.set_param(self.color+'Low', [values[0][0], values[0][1], values[0][2]])
		rospy.set_param(self.color+'High', [values[1][0], values[1][1], values[1][2]])
	

	def hsvCalibration(self,frame,color):
		#Create a window
		cv2.namedWindow(color+" Threshold", cv2.CV_WINDOW_AUTOSIZE)
		cv2.startWindowThread()

		#Set the min and max values for each variable
		iLowH = 0		#Hue Low
		iHighH = 179	#Hue High
		iLowS = 0		#Saturation Low
		iHighS = 255	#Saturation High
		iLowV = 0 		#Value Low
		iHighV = 255	#Value High

		# Create trackbars in "Control" window
		cv2.createTrackbar("Hue - Low", color+" Threshold", iLowH, 179, nothing) #Hue (0 - 179)
		cv2.createTrackbar("Hue - High", color+" Threshold", iHighH, 179, nothing)
		cv2.createTrackbar("Saturation - Low", color+" Threshold", iLowS, 255, nothing) #Saturation (0 - 255)
		cv2.createTrackbar("Saturation - High", color+" Threshold", iHighS, 255, nothing)
		cv2.createTrackbar("Value - Low", color+" Threshold", iLowV, 255, nothing)#Value (0 - 255)
		cv2.createTrackbar("Value - High", color+" Threshold", iHighV, 255, nothing)

		while ((cv2.waitKey(5) & 0xFF)!=27):
			img = frame.copy()

			#Transform the image space to HSV
			hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) 

			#Obtain the actual trackbar value for each variable
			iLowH = cv2.getTrackbarPos('Hue - Low',color+" Threshold")
			iLowS = cv2.getTrackbarPos('Saturation - Low',color+" Threshold")
			iLowV = cv2.getTrackbarPos('Value - Low',color+" Threshold")
			iHighH = cv2.getTrackbarPos('Hue - High',color+" Threshold")
			iHighS = cv2.getTrackbarPos('Saturation - High',color+" Threshold")
			iHighV = cv2.getTrackbarPos('Value - High',color+" Threshold")

			#Threshold the image
			thresholded = cv2.inRange(hsv, np.array([iLowH, iLowS, iLowV]), np.array([iHighH, iHighS, iHighV]))

			#Morfological transformations
			kernel = np.ones((5,5),np.uint8)
			opening = cv2.morphologyEx(thresholded, cv2.MORPH_OPEN, kernel) #opening (removes small objects from the foreground)
			closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel) #closing (removes small holes from the foreground)

			#Bitwise 'and' between the original image and the threshold mask
			result = cv2.bitwise_and(img,img,mask=closing)
			
			#Show the thresholded image in the window
			#small = cv2.resize(result,(0,0),fx=0.5,fy=0.5);
			cv2.imshow(color+" Threshold", result) 

		#cv2.destroyAllWindows()
		cv2.destroyWindow(color+" Threshold")

		values=[[iLowH, iLowS, iLowV],[iHighH, iHighS, iHighV]]
		print values
		return values

	def core(self, frame):
		if (rospy.has_param('calibrate_color')):
			if (rospy.get_param('calibrate_color')=="blue"):
				self.color="Blue"
			elif (rospy.get_param('calibrate_color')=="green"):
				self.color="Green"
			
			values = self.hsvCalibration(frame, self.color)
			self.setValues(values)
			rospy.delete_param('calibrate_color')

# Main
def main(args):
	color_threshold()
	rospy.spin()

if __name__ == '__main__':
	try:
		main(sys.argv)
	except RuntimeError, e:
		rospy.logerr('Something went wrong: %s' % (e.strerrror) )
	rospy.loginfo('color_threshold stopped')

