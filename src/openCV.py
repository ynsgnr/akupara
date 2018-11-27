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
    self.image_pub = rospy.Publisher(produces,Image,queue_size=10)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber(consumes,Image,self.callback)
    self.motor_command_publisher = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=100)
    self.depth_sub = rospy.Subscriber("/camera/depth/image_raw",Image,self.depth_callback)
    self.depth_image=np.zeros((640,480))
    self.forwardMoves=0
    self.doorColor=(0,255,255)

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

    # #detect shapes
    # ret,thresh = cv2.threshold(gray,10,255,1)
    # inverted=cv2.bitwise_not(thresh)
    # blurred = cv2.dilate(inverted,np.ones((5, 5), np.uint8),iterations = 2 )
    # image, contours, h = cv2.findContours(blurred,1,2)
    # contouredImg=cv_image.copy()
    #
    # cmask = np.zeros(gray.shape,np.uint8)
    # for cnt in contours:
    #     approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
    #     print("Detected shape has "+str(len(approx)))
    #     cv2.drawContours(cmask,[cnt],0,255,-1) #draw on mask
    #     cv2.drawContours(contouredImg,[cnt],0,cv2.mean(cv_image,cmask),-1) #draw avarage color on img
    #

    doors=list()

    #find door with edge detection
    # edges = cv2.Canny(gray,100,200)
    # edgesFilled=cv2.morphologyEx(edges,cv2.MORPH_CLOSE,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(80,80)))
    # image, contours, h = cv2.findContours(edges,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)

    #find door with countours
    ret,thresh = cv2.threshold(gray,10,255,1)
    inverted=cv2.bitwise_not(thresh)
    blurred = cv2.dilate(inverted,np.ones((5, 5), np.uint8),iterations = 2 )
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

    #print("Detected "+str(len(doors))+" shapes")
    for d in doors:
        if(d[3]>0):
            if(x>2*(len(cv_image)/3)):
                where="left"
            elif(x>len(cv_image)/3):
                where="mid"
            else:
                where="right"
            #print("Has "+str(d[0])+" lines and "+str(d[1])+" area and color "+str(d[2])+" its on the "+where)

    # h, w = edges.shape[:2]
    # maskFill = np.zeros((h+2, w+2), np.uint8)
    # cv2.floodFill(edges, maskFill, (0,0), 255);
    # editedImage = editedImage | cv2.cvtColor(edges,cv2.COLOR_GRAY2BGR)

    #image windows
    cv2.imshow("Shapes by shapes window", editedImage)
    cv2.imshow("View window", blurred)

    #Movement Part

    #step constants
    step=0.3
    angleStep=0.2
    threshold=0.3
    colorThreshold=50
    motor_command=Twist()

    #calculate mean
    l=len(self.depth_image)
    self.depth_image[~np.isfinite(self.depth_image)]=0 #replace NaN and inf
    ml=cv2.mean(self.depth_image[:,:l/2])[0] #left side mean
    mr=cv2.mean(self.depth_image[:,l/2:])[0] #right side mean
    m=cv2.mean(self.depth_image)[0]
    #print(str(ml)+"-"+str(m)+"-"+str(mr))
    if(m<threshold):
        ml+=0.001
        mr+=0.001
        m+=0.001 #prevent zero values
        motor_command.linear.x=-step
        self.motor_command_publisher.publish(motor_command)
        if(ml<mr):
            motor_command.angular.z=-angleStep/ml
        else:
            motor_command.angular.z=angleStep/mr
        self.motor_command_publisher.publish(motor_command)
        #print("Too close to the wall, follow wall")
    elif(ml<mr and ml<threshold):
        motor_command.linear.x=step
        motor_command.angular.z=angleStep
        self.motor_command_publisher.publish(motor_command)
        #print("Too close to left wall, go right")
    elif(mr<threshold):
        motor_command.linear.x=step
        motor_command.angular.z=-angleStep
        self.motor_command_publisher.publish(motor_command)
        #print("Too close to right wall, go left")
    elif(self.forwardMoves<50):
        #check doors, structure: [l,a,c,v,(x,y)]
        for door in doors:
            # pixels=np.zeros((1,2,3),np.uint8)
            # pixels[0][0]=door[2]
            # pixels[0][1]=self.doorColor
            # doorColor=cv2.cvtColor(pixels, cv2.COLOR_BGR2Lab)
            # dc=doorColor[0][0].astype(float) #door color in lab
            # tc=doorColor[0][1].astype(float) #target color in lab
            # #opencv scales lab to 0,25. rescale:
            #
            # dc[0]=dc[0]*0.39#L
            # dc[1]=dc[1]-128#a
            # dc[2]=dc[2]-128#b
            #
            # tc[0]=tc[0]*0.39#L
            # tc[1]=tc[1]-128#a
            # tc[2]=tc[2]-128#b
            # #use LAB Delta E to calculate color difference - Apperantly LAB does not work as good as RGB
            # colorDiff=np.sqrt((np.power(np.subtract(dc,tc),2)).sum())
            # colorDiffRGB=np.sqrt((np.power(np.subtract(door[2],self.doorColor),2)).sum())

            #Normalize Pixel RGB Values
            pixelsND=np.zeros((1,1,3),np.uint8)
            pixelsNP=np.zeros((1,1,3),np.uint8)
            pixelsND[0][0]=door[2]
            pixelsNP[0][0]=self.doorColor
            normalizedDoorColor = cv2.normalize(pixelsND, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
            normalizedPredefinedColor = cv2.normalize(pixelsNP, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
            #Then calculate color distance
            colorDiffN=np.sqrt((np.power(np.subtract(normalizedDoorColor,normalizedPredefinedColor),2)).sum())

            print("\n")
            x=d[4][0][0]
            print(x)
            print(len(cv_image)/3)
            if(colorDiffN<colorThreshold):
                if(x>2*(len(cv_image)/3)):
                    print("turn left")
                elif(x>len(cv_image)/3):
                    print("go to empty space")
                else:
                    print("turn right")

            # print("Color test: LAB")
            # print(dc)
            # print(tc)
            # print(colorDiff)
            # print("Color test: BGR")
            # print(door[2])
            # print(self.doorColor)
            # print(colorDiffRGB)
            # print("Color test: N")
            # print(normalizedDoorColor[0][0])
            # print(normalizedPredefinedColor[0][0])
            print(colorDiffN)

        #seems clear move forward
        self.forwardMoves+=1
        motor_command.linear.x=step
        self.motor_command_publisher.publish(motor_command)
        #print("March Forward!")
    else:
        #print("Seems like there is nothing here lets go other way")
        motor_command.angular.z=1.5
        self.motor_command_publisher.publish(motor_command)
        self.forwardMoves=0


    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

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
