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

    for cnt in contours:
        approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
        print("Detected shape has "+str(len(approx))+" sides and it is:")
        if len(approx)==5:
            print ("pentagon")
            cv2.drawContours(contouredImg,[cnt],0,255,-1)
        elif len(approx)==3:
            print ("triangle")
            cv2.drawContours(contouredImg,[cnt],0,(0,255,0),-1)
        elif len(approx)==4:
            print ("square")
            cv2.drawContours(contouredImg,[cnt],0,(0,0,255),-1)
        elif len(approx) == 9:
            print ("half-circle")
            cv2.drawContours(contouredImg,[cnt],0,(255,255,0),-1)
        elif len(approx) > 15:
            print ("circle")
            cv2.drawContours(contouredImg,[cnt],0,(0,255,255),-1)


    edges = cv2.Canny(shadowless,100,200)

    cv2.imshow("Image window", cv_image)
    cv2.imshow("Shapes window", contouredImg)
    cv2.waitKey(3)

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
