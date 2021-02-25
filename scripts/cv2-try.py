#!/usr/bin/env python

import rospy
import cv2
import sys
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from pyzbar.pyzbar import decode
from cv_bridge import CvBridge, CvBridgeError


class Camera1:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image,self.callback)
    self.function()


  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      rospy.logerr(e)

    (rows,cols,channels) = cv_image.shape
    
    image = cv_image

    gray_img = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)

    the = cv2.getTrackbarPos("threshold","Trackbar")

    _, th1 = cv2.threshold(gray_img, the, 255, cv2.THRESH_BINARY)

    qr_result = decode(th1)

    for barcode in qr_result:
        x,y,w,h = barcode.rect
        myData = barcode.data
        cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,255),5)
        cv2.putText(image,myData,(x,y-10),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,0),2)



    cv2.imshow("Binary", th1)
    cv2.imshow("final", image)

    cv2.waitKey(3)

  def function(self):
    print("!!!!!!!!!!!")
    return True

def nothing(hiii):
    pass

cv2.namedWindow("Trackbar")
cv2.createTrackbar("threshold","Trackbar",10,210,nothing)


def main(args):
  
  rospy.init_node('node_eg1_read_camera', anonymous=True)

  ic = Camera1()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    rospy.loginfo("Shutting down")
  
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
