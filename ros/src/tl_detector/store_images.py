#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError

import time


class image_manipulation:


  def __init__(self):

    self.store_freq = 10
    self.image_cnt = 0
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/image_color", Image, self.image_callback)


  def image_callback(self,data):
    
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)


    if ( self.image_cnt % self.store_freq == 0) :
      cv2.imshow("Image window", cv_image)
      cv2.waitKey(1)
      file_name = time.strftime("%H_%M_%S") + "_" + str(self.image_cnt) + ".jpg"
      print file_name
      cv2.imwrite(file_name, cv_image)
    self.image_cnt = self.image_cnt + 1



def main(args):
  rospy.init_node('image_manipulation', anonymous=True)
  ic = image_manipulation()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
