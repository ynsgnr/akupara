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
import random

class image_converter:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)
    self.door_color_sub = rospy.Subscriber("door_color",String,self.color_call)
    self.motor_command_publisher = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=100)
    self.depth_sub = rospy.Subscriber("/camera/depth/image_raw",Image,self.depth_callback)
    self.depth_image=np.zeros((640,480))
    self.forwardMoves=0
    self.colors={'blue':(255,0,0),'green':(0,255,0),'pink':(255,255,0),'yellow':(0,255,255),'red':(0,0,255)}
    self.doorColor=(255,0,0)
    self.move=False
    self.stepCount=0

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
    gray = cv2.cvtColor(shadowless, cv2.COLOR_BGR2GRAY)

    doors=list()

    #find door with countours
    ret,thresh = cv2.threshold(gray,10,255,1)
    inverted=cv2.bitwise_not(thresh)
    blurred = cv2.dilate(inverted,np.ones((20, 20), np.uint8),iterations = 2 )
    #add door inputs from depth image
    image, contours, h = cv2.findContours(blurred,1,2)

    editedImage=cv_image.copy()
    for cnt in contours:
        cmask = np.zeros(gray.shape,np.uint8)
        approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
        cv2.drawContours(cmask,[cnt],0,255,cv2.FILLED) #draw on mask
        c=cv2.mean(cv_image,cmask)[:-1]
        l=len(approx)
        a=0
        v=0
        #Get central coordinates:
        x=cv2.mean(cnt[:,0,0])
        y=cv2.mean(cnt[:,0,1])
        if l<8 and c[0]>20 and c[1]>20 and c[2]>20:
            a=cv2.contourArea(cnt)
            v=1
            cv2.drawContours(editedImage,[cnt],0,(255-c[0],255-c[1],255-c[2]),cv2.FILLED) #draw avarage color on img
        doors.append([l,a,c,v,(x,y)])

    #image windows
    cv2.imshow("Shapes by shapes window", editedImage)

    #Movement Part
    if(not self.move):
        return

    #step constants
    step=0.3
    angleStep=0.3
    threshold=0.2
    colorThreshold=50
    areaThreshold=0.3
    self.stepCount+=1
    randomizer=random.uniform(0, 1)*self.stepCount
    motor_command=Twist()

    #calculate mean
    l=len(self.depth_image)
    self.depth_image[~np.isfinite(self.depth_image)]=0 #replace NaN and inf
    ml=cv2.mean(self.depth_image[:,:l/2])[0] #left side mean
    mr=cv2.mean(self.depth_image[:,l/2:])[0] #right side mean
    m=cv2.mean(self.depth_image)[0]
    if(m<threshold*2):
        ml+=0.001
        mr+=0.001
        m+=0.001 #prevent zero values
        motor_command.linear.x=(-step/(ml+mr))*randomizer
        self.motor_command_publisher.publish(motor_command)
        if(ml<mr):
            motor_command.angular.z=-angleStep/ml*randomizer
        else:
            motor_command.angular.z=angleStep/mr*randomizer
        self.motor_command_publisher.publish(motor_command)
        motor_command.angular.z=-motor_command.angular.z
        self.motor_command_publisher.publish(motor_command)
        print("Too close to the wall, follow wall")
    elif(ml<mr and ml<threshold):
        motor_command.linear.x=step
        motor_command.angular.z=angleStep*randomizer
        self.motor_command_publisher.publish(motor_command)
        motor_command.angular.z=-motor_command.angular.z
        self.motor_command_publisher.publish(motor_command)
        print("Too close to left wall, go right")
    elif(mr<threshold):
        motor_command.linear.x=step
        motor_command.angular.z=-angleStep*randomizer
        self.motor_command_publisher.publish(motor_command)
        motor_command.angular.z=-motor_command.angular.z
        self.motor_command_publisher.publish(motor_command)
        print("Too close to right wall, go left")
    else:
        #check doors, structure: [l,a,c,v,(x,y)]

        self.stepCount=0

        totalArea=len(cv_image)*len(cv_image)

        colorDiffN=9999
        print(len(doors))
        if len(doors)==2:
            print("Go throught door")
            tmp=np.zeros((1,1,3),np.uint8)
            tmp[0][0]=doors[0][2]
            normalizedDoorColor = cv2.normalize(tmp, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
            tmp=np.zeros((1,1,3),np.uint8)
            tmp[0][0]=doors[1][2]
            normalizedDoorColor2 = cv2.normalize(tmp, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
            normalizedPredefinedColor = cv2.normalize(self.doorColor, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
            if(normalizedDoorColor.shape==(1,1,3)):
                normalizedDoorColor=np.transpose(normalizedDoorColor[0])
            if(normalizedDoorColor2.shape==(1,1,3)):
                normalizedDoorColor2=np.transpose(normalizedDoorColor2[0])
            colorDiffN=np.sqrt((np.power(np.subtract(normalizedDoorColor,normalizedPredefinedColor),2)).sum())
            colorDiffN2=np.sqrt((np.power(np.subtract(normalizedDoorColor2,normalizedPredefinedColor),2)).sum())
            fieldSize=100
            if(colorDiffN<colorThreshold and colorDiffN2<colorThreshold):
                print(doors[0][4][0][0]>(len(cv_image)/2) and doors[1][4][0][0]<(len(cv_image)/2))
                print(doors[0][4][0][0]<(len(cv_image)/2) and doors[1][4][0][0]>(len(cv_image)/2))
                if (doors[0][4][0][0]>(len(cv_image)/2) and doors[1][4][0][0]<(len(cv_image)/2)) or (doors[0][4][0][0]<(len(cv_image)/2) and doors[1][4][0][0]>(len(cv_image)/2)):
                    print("Tring to not the hit corners")
                    if(ml<mr):
                        t=-angleStep/2
                    else:
                        t=angleStep/2
                    motor_command.linear.x=step
                    motor_command.angular.z=t
                    self.motor_command_publisher.publish(motor_command)
                    motor_command.angular.z=-t
                    self.motor_command_publisher.publish(motor_command)
        else:
            for door in doors:

                #Normalize Pixel RGB Values
                pixelsND=np.zeros((1,1,3),np.uint8)
                pixelsNP=np.zeros((1,1,3),np.uint8)
                pixelsND[0][0]=door[2]
                pixelsNP[0][0]=self.doorColor
                normalizedDoorColor = cv2.normalize(pixelsND, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
                normalizedPredefinedColor = cv2.normalize(pixelsNP, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
                #Then calculate color distance in RGB. LAB distance does not give good results
                colorDiffN=np.sqrt((np.power(np.subtract(np.transpose(normalizedDoorColor[0]),normalizedPredefinedColor),2)).sum())


                motor_command.linear.x=0
                x=door[4][0][0]
                if(x>2*(len(cv_image)/3)):
                    print("door is right")
                    t=angleStep
                elif(x>len(cv_image)/3):
                    print("door is middle")
                    if(ml<mr):
                        t=angleStep
                    else:
                        t=-angleStep
                else:
                    print("door is left")
                    t=-angleStep

                if(colorDiffN<colorThreshold):
                    print("Good Door")
                    motor_command.angular.z=t
                else:
                    print("Bad Door")
                    motor_command.angular.z=0
                t=0
                self.motor_command_publisher.publish(motor_command)
                motor_command.angular.z=-motor_command.angular.z
                self.motor_command_publisher.publish(motor_command)

        #seems clear move forward
        self.forwardMoves+=1
        motor_command.linear.x=step
        self.motor_command_publisher.publish(motor_command)
        print("March Forward!")

    cv2.waitKey(3)

  def color_call(self,msg_str):
      color=msg_str.data
      if color in self.colors:
          print("Going to the "+color+" door")
          self.move=True
          self.doorColor=self.colors[color]
      else:
          print("Undefined color")

  def depth_callback(self,msg_depth):
      try:
          self.depth_image = self.bridge.imgmsg_to_cv2(msg_depth, "32FC1")
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
