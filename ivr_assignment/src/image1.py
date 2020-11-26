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
    # initialize a publisher to send images from camera1 to a topic named image_topic1
    self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()

    # initialize publisher to send joint angels to robot
    self.joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size = 10)
    self.joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size = 10)
    self.joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size = 10)
    
    # initialize time 
    self.time = rospy.get_time()

    # initialize subscriber to receive x_z data from image2 node processing
    self.x_z_positions_sub = rospy.Subscriber("/robot/x_z_pos", Float64MultiArray, self.callback2)

    # initialize publisher to public joints values
    self.joints_value_pub = rospy.Publisher("/robot/joints_value", Float64MultiArray, queue_size = 10)



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
      else:
        cx, cy = int(0), int(0)
      return np.array([cx, cy])
  
  # Calculate the conversion from pixel to meter
  def pixel2meter(self,image):
      # Obtain the centre of each coloured blob
      circle1Pos = self.detect_yellow(image)
      circle2Pos = self.detect_blue(image)
      # find the distance between two circles
      dist = np.sum((circle1Pos - circle2Pos)**2)
      return 2.5 / np.sqrt(dist)
  
  # get the x_z positions of all joints from image2
  def callback2(self,data):
    self.x_z_positions = data.data
  
  # calculate the joint values
  def calculate_joint_angles(self,image):
  # get conversion from pixel to meters
    scale = self.pixel2meter(image)
  # get y,z coordinate of joints in pixels
    y_z_yellow = self.detect_yellow(image)
    y_z_blue = self.detect_blue(image)
    y_z_green = self.detect_green(image)
    y_z_red = self.detect_red(image)
  
  # combine into 3d coordinates

    yellow_x = self.x_z_positions[0]
    yellow_y = y_z_yellow[0]

    # average the z to reduce error
    if (self.x_z_positions[1] * y_z_yellow[1]) != 0:
      yellow_z = (self.x_z_positions[1] + y_z_yellow[1]) / 2
    elif y_z_yellow[1] != 0:
      yellow_z = y_z_yellow[1]
    else:
      yellow_z = self.x_z_positions[1]

    blue_x = self.x_z_positions[2]
    blue_y = y_z_blue[0]
    if (self.x_z_positions[3] * y_z_blue[1]) != 0:
      blue_z = (self.x_z_positions[3] + y_z_blue[1]) / 2
    elif y_z_blue[1] != 0:
      blue_z = y_z_blue[1]
    else:
      blue_z = self.x_z_positions[3]
    
    # when x or y is 0, that means this joint is hidden behind others in one image, so take the coordinates of another joints as its coordinate
    if (self.x_z_positions[5] * y_z_green[1]) != 0:
      green_z = (self.x_z_positions[5] + y_z_green[1]) / 2
    elif y_z_green[1] != 0:
      green_z = y_z_green[1]
    else:
      green_z = self.x_z_positions[5]
    
    # green could be behind blue and yellow
    green_x = self.x_z_positions[4]
    if green_x == 0 and green_z > blue_z *1.05 and green_z < blue_z * 0.95:
      green_x = blue_x
    elif green_x == 0 and green_z > yellow_z *1.05 and green_z < yellow_z * 0.95:
      green_x = yellow_x

    green_y = y_z_green[0]
    if green_y == 0 and green_z > blue_z *1.05 and green_z < blue_z * 0.95:
      green_y = blue_y
    elif green_x == 0 and green_z > yellow_z *1.05 and green_z < yellow_z * 0.95:
      green_y = yellow_y

    if (self.x_z_positions[7] * y_z_red[1]) != 0:
      red_z = (self.x_z_positions[7] + y_z_red[1]) / 2
    elif y_z_red[1] != 0:
      red_z = y_z_red[1]
    else:
      red_z = self.x_z_positions[7]

    red_x = self.x_z_positions[6]
    if red_x == 0 and red_z > green_z * 1.05 and red_z < green_z * 0.95:
      red_x = green_x
    elif red_x == 0 and red_z > blue_z * 1.05 and red_z < blue_z * 0.95:
      red_x == blue_x
    elif red_x == 0 and red_z > yellow_z * 1.05 and red_z < yellow_z * 0.95:
      red_x == yellow_x

    # red could be behind yellow blue and green
    red_y = y_z_red[0]
    if red_y == 0 and red_z > green_z * 1.05 and red_z < green_z * 0.95:
      red_y == green_y
    elif red_y == 0 and red_z > blue_z * 1.05 and red_z < blue_z * 0.95:
      red_y == blue_y
    elif red_y == 0 and red_z > yellow_z * 1.05 and red_z < yellow_z * 0.95:
      red_y == yellow_x

    # transfer into meter coordinate and take joint1 position as base frame
    self.blue_pos = np.array([blue_x - yellow_x, blue_y - yellow_y, yellow_z - blue_z]) 
    self.green_pos = np.array([green_x - yellow_x, green_y - yellow_y, yellow_z - green_z]) 
    self.red_pos = np.array([red_x - yellow_x, red_y - yellow_y, yellow_z - red_z]) 
    self.yellow_pos = np.array([0,0,0])

    # calcualte joint values
    joints2_diff_z3 = np.sqrt((self.blue_pos[1] - self.green_pos[1])**2 + (self.blue_pos[2] - self.green_pos[2])**2)
    joints2_diff_z2 = np.sqrt((self.blue_pos[0] - self.green_pos[0])**2 + (self.blue_pos[2] - self.green_pos[2])**2)
    #joint2 = np.arctan2(np.abs(self.green_pos[1] - self.blue_pos[1]), joints2_diff_z2)
    #if self.green_pos[1]>self.blue_pos[1]:
    #  joint2 = -joint2
    joint2 = -np.arctan2(self.green_pos[1] - self.blue_pos[1], self.green_pos[2] - self.blue_pos[2]) 
    joint3 = np.arctan2(-(self.blue_pos[0] - self.green_pos[0]), joints2_diff_z3)
    self.joints_value = Float64MultiArray()
    self.joints_value.data = np.array([joint2,joint3])
    try:
      self.joints_value_pub.publish(self.joints_value)
    except CvBridgeError as e:
      print(e)





  

  # Recieve data from camera 1, process it, and publish
  def callback1(self,data):

    # move the three joints in sinusoidal signals Q2
    self.joint2 = Float64()
    self.joint3 = Float64()
    self.joint4 = Float64()

    self.joint2.data = 1.5# 0.5 * np.pi * np.sin(np.pi * (rospy.get_time() - self.time) / 15)
    self.joint3.data = 1.5# 0.5 * np.pi * np.sin(np.pi * (rospy.get_time() - self.time) / 18)
    self.joint4.data = 0# 0.5 * np.pi * np.sin(np.pi * (rospy.get_time() - self.time) / 20)
    try:
      self.joint2_pub.publish(self.joint2)
      self.joint3_pub.publish(self.joint3)
      self.joint4_pub.publish(self.joint4)
    except CvBridgeError as e:
      print(e)


    # Recieve the image
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    # Uncomment if you want to save the image
    cv2.imwrite('image_copy.png', self.cv_image1)

    im1=cv2.imshow('window1', self.cv_image1)
    cv2.waitKey(1)
    # Publish the results
    try: 
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
    except CvBridgeError as e:
      print(e)

    # get the joints
    self.calculate_joint_angles(self.cv_image1)

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


