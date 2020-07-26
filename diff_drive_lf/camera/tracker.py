#!/usr/bin/env python
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import sys
import numpy as np

bridge = CvBridge()

class control:
  p = 0
  i = 0
  d = 0
  last_error = 0

def image_callback(ros_image):
  #print 'got an image'
  global bridge
  #convert ros_image into an opencv-compatible image
  try:
    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
  except CvBridgeError as e:
      print(e)

  #from now can work exactly like with opencv
  cv_image = cv2.resize(cv_image, (0,0), fx=0.5,fy=0.5)
  cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

  vel = Twist()

  trshld = 127
  print ("%s.%s.%s.%s.%s.%s.%s.%s") %(cv_image[375][50],cv_image[375][150],cv_image[375][250],cv_image[375][350],cv_image[375][450],cv_image[375][550],cv_image[375][650],cv_image[375][750])
  if cv_image[399][50] < trshld:
    ls1 = 1
  else:
    ls1 = 0
  if cv_image[399][150] < trshld:
    ls2 = 1
  else:
    ls2 = 0
  if cv_image[399][250] < trshld:
    ls3 = 1
  else:
    ls3 = 0
  if cv_image[399][350] < trshld:
    ls4 = 1
  else:
    ls4 = 0
  if cv_image[399][450] < trshld:
    ls5 = 1
  else:
    ls5 = 0
  if cv_image[399][550] < trshld:
    ls6 = 1
  else:
    ls6 = 0
  if cv_image[399][650] < trshld:
    ls7 = 1
  else:
    ls7 = 0
  if cv_image[399][750] < trshld:
    ls8 = 1
  else:
    ls8 = 0

  numerator = (ls1*4)+(ls2*3)+(ls3*2)+ls4-ls5-(ls6*2)-(ls7*3)-(ls8*4)
  denominator = ls1+ls2+ls3+ls4+ls5+ls6+ls7+ls8

  if denominator == 0:
    denominator = 1

  error = numerator/denominator


  control.p = error*1
  control.i = (control.i+error) * 0.01
  control.d = (error - control.last_error) * 1.1
  control.last_error = error

  

  vel.linear.x = 0.65
  vel.angular.z = control.p + control.i + control.d

  pub = rospy.Publisher('/lf_diff_drive_controller/cmd_vel', Twist, queue_size=20)
  pub.publish(vel)

  cv2.imshow("lf_camera", cv_image)

  
  cv2.waitKey(3)

  
def main(args):
  rospy.init_node('image_converter', anonymous=True)
  image_sub = rospy.Subscriber("/lf_camera/image_raw",Image, image_callback)


  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
