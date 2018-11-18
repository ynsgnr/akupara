#!/usr/bin/env python
from __future__ import print_function

#taken from http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

#ROS stuff
import roslib
roslib.load_manifest('akupara')
import sys
import rospy
#Message Types
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Twist
#Movement Stuff
import tf
from tf import transformations
#Image Processing Stuff
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

consumes="/camera/rgb/image_raw"
produces="detection"

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher(produces,Image,queue_size=1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber(consumes,Image,self.callback)
    self.motor_command_publisher = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=100)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    #eliminate shadows
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 0, 0), (0, 255,255))
    rgbMask = cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)
    shadowless = cv2.subtract(cv_image,rgbMask)

    #detect shapes
    gray = cv2.cvtColor(shadowless, cv2.COLOR_BGR2GRAY)
    ret,thresh = cv2.threshold(gray,10,255,1)
    inverted=cv2.bitwise_not(thresh)
    blurred = cv2.dilate(inverted,np.ones((5, 5), np.uint8),iterations = 2 )
    image, contours, h = cv2.findContours(blurred,1,2)
    contouredImg=cv_image.copy()

    cmask = np.zeros(gray.shape,np.uint8)
    for cnt in contours:
        approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
        print("Detected shape has "+str(len(approx)))
        cv2.drawContours(cmask,[cnt],0,255,-1) #draw on mask
        cv2.drawContours(contouredImg,[cnt],0,cv2.mean(cv_image,cmask),-1) #draw avarage color on img


    #find door with edge detection
    edges = cv2.Canny(shadowless,100,200)
    image, contours, h = cv2.findContours(edges,1,2)
    edgeCountered=cv_image.copy()
    cmask = np.zeros(gray.shape,np.uint8)
    for cnt in contours:
        approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
        print("Detected shape has "+str(len(approx)))
        cv2.drawContours(cmask,[cnt],0,255,-1) #draw on mask
        cv2.drawContours(edgeCountered,[cnt],0,cv2.mean(cv_image,cmask),-1) #draw avarage color on img

    # h, w = edges.shape[:2]
    # maskFill = np.zeros((h+2, w+2), np.uint8)
    # cv2.floodFill(edges, maskFill, (0,0), 255);
    # edgeCountered = edgeCountered | cv2.cvtColor(edges,cv2.COLOR_GRAY2BGR)

    #image windows
    cv2.imshow("Image window", cv_image)
    cv2.imshow("Shapes window", contouredImg)
    cv2.imshow("Shapes by edge window", edgeCountered)
    cv2.imshow("Edges window", edges)
    cv2.imshow("Filtered window", shadowless)
    cv2.waitKey(3)

    #move
    motor_command=Twist()
    motor_command.linear.x=0
    motor_command.angular.z=0
    self.motor_command_publisher.publish(motor_command)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
