#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera2 to a topic named image_topic2
    self.image_pub2 = rospy.Publisher("image_topic2",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback2)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    # initialize a publisher to send x_z_pisitions of circles and targets to image1 node 
    self.x_z_positions_pub = rospy.Publisher("robot/x_z_pos", Float64MultiArray, queue_size = 10)


  # detect the red circle center
  def detect_red(self,image):
      # Isolate the red colour in the image as a binary image
      mask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
      kernel = np.ones((5, 5), np.uint8)

      # Obtain the moments of the binary image
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)

      # Calculate pixel coordinates for the centre of the blob
      if M['m00'] != 0:
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
      # if m00 is zero, means not find the that color of joints, set center 0 first
      else:
        cx, cy = int(0), int(0)
      return np.array([cx, cy])

  # detect the green circle center
  def detect_green(self,image):
      # Isolate the green colour in the image as a binary image
      mask = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
      kernel = np.ones((5, 5), np.uint8)

      # Obtain the moments of the binary image
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)

      # Calculate pixel coordinates for the centre of the blob
      if M['m00'] != 0:
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
      # if m00 is zero, means not find the that color of joints, set center 0 first
      else:
        cx, cy = int(0), int(0)
      return np.array([cx, cy])

  # detect the blue circle center
  def detect_blue(self,image):
      # Isolate the blue colour in the image as a binary image
      mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
      kernel = np.ones((5, 5), np.uint8)

      # Obtain the moments of the binary image
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)

      # Calculate pixel coordinates for the centre of the blob
      if M['m00'] != 0:
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
      # if m00 is zero, means not find the that color of joints, set center 0 first
      else:
        cx, cy = int(0), int(0)
      return np.array([cx, cy])

  # detect the yellow circle center
  def detect_yellow(self,image):
      # Isolate the yellow colour in the image as a binary image
      mask = cv2.inRange(image, (0, 100, 100), (0, 255, 255))
      kernel = np.ones((5, 5), np.uint8)

      # Obtain the moments of the binary image
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)

      # Calculate pixel coordinates for the centre of the blob
      if M['m00'] != 0:
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
      # if m00 is zero, means not find the that color of joints, set center 0 first
      else:
        cx, cy = int(0), int(0)
      return np.array([cx, cy])


  # Recieve data, process it, and publish
  def callback2(self,data):
    # Recieve the image
    try:
      self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Uncomment if you want to save the image
    cv2.imwrite('image2_copy.png', self.cv_image2)
    im2=cv2.imshow('window2', self.cv_image2)
    cv2.waitKey(1)

    # Publish the results
    try: 
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
    except CvBridgeError as e:
      print(e)

    # publish the x_z_position of joints to image1 node
    self.x_z_yellow = self.detect_yellow(self.cv_image2)
    self.x_z_blue = self.detect_blue(self.cv_image2)
    self.x_z_green = self.detect_green(self.cv_image2)
    self.x_z_red = self.detect_red(self.cv_image2)

    self.x_z_positions = Float64MultiArray()
    self.x_z_positions.data = np.array([self.x_z_yellow[0], self.x_z_yellow[1], self.x_z_blue[0], self.x_z_blue[1], 
                                       self.x_z_green[0], self.x_z_green[1], self.x_z_red[0], self.x_z_red[1]])

    try:
      self.x_z_positions_pub.publish(self.x_z_positions)
    except CvBridgeError as e:
      print(e)



# call the class
def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)


