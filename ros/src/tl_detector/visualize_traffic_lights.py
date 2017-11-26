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

    self.image_pub = rospy.Publisher("/image_with_traffic_lights", Image, queue_size=1)

  def image_callback(self,data):
    
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    s_x = 100
    s_y = 200
    s_height = 30
    s_width = 50
    cv2.rectangle(cv_image, (s_x, s_y), (s_x+s_width, s_y+s_height), (111,222,0), 10)
    self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))




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
