#!/usr/bin/env python
# -*- coding: utf-8 -*-  

import roslib
roslib.load_manifest('cv_bridge_tutorial')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:
  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic", Image, queue_size=1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # RGB表色系からHSV表色系に変換
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # しきい値の設定（ここでは赤を抽出）
    color_min = np.array([150,100,150])
    color_max = np.array([180,255,255])

    # マスク画像を生成
    color_mask = cv2.inRange(hsv_image, color_min, color_max);
    # 画像配列のビット毎の倫理席。マスク画像だけが抽出される。
    cv_image2  = cv2.bitwise_and(cv_image, cv_image, mask = color_mask)

    # RGBからグレースケールへ変換
    gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    cv_image3  = cv2.Canny(gray_image, 15.0, 30.0);

    # 円を描画
    color  = (0, 255, 0)
    center = (100, 100)
    radius = 20
    cv2.circle(cv_image2, center, radius, color)

    # ウインドウのサイズを変更
    cv_half_image = cv2.resize(cv_image,   (0,0),fx=0.5, fy=0.5)
    cv_half_image2 = cv2.resize(cv_image2, (0,0),fx=0.5,fy=0.5);
    cv_half_image3 = cv2.resize(cv_image3, (0,0),fx=0.5,fy=0.5);

    # ウインドウ表示
    cv2.imshow("Origin Image", cv_half_image)
    cv2.imshow("Result Image", cv_half_image2)
    cv2.imshow("Edge Image",   cv_half_image3)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image2, "bgr8"))
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
