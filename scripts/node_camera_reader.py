#!/usr/bin/env python

'''                       This Node reads the package location and
                             stops the conveyor belt at desired
                                        package location
                                                                             '''

import time
import rospy
import cv2
import rospkg
import yaml
import json
import threading
from parameter_generator import Param
from pkg_vb_sim.msg import LogicalCameraImage
from pkg_task5.msg import packageName
from pkg_task5.msg import packageIdeal
from conveStart_vacuuStart import conveyor_start
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode
from sensor_msgs.msg import Image



class MainaReader:

    # Constructor
    def __init__(self):

        rp = rospkg.RosPack()
        pkg_path = rp.get_path('pkg_task5')
        self._file_path = pkg_path + '/config/package_priority.yaml'
        self._boilercode = {'package_details': {}}
        self._bridge = CvBridge()
        self.PREASENT_PKG = "non"
        self.STOP_ONCE = 0
        self.start_once = 0
        self.start_Once = 0
        self._pkg_colour = {}
        self._gSheet_update = {}
        self._subForLimitedTime = ""
        self.package_name_pub = rospy.Publisher('/package_name', packageName, queue_size=0)
        self.package_ideal_pub = rospy.Publisher('/package_ideal', packageIdeal, queue_size=0)

    def func_callback(self, data):

        ''' This is the main conveyor belt  Start/stoping  function
                The global values are there so that the start and
                    stop command ONLY ONCE for each pakage         '''
  
        
        index = 0
        
        # This is to check is 'ur5' or any package is under camera
        if (len(data.models)):
            obj = data.models[index]
            if obj.type == "ur5":
                try:
                    index = 1
                    obj = data.models[index]
                except:
                    # This is for the conveyor to start if package is not under camera
                    if self.start_Once == 0:
                        conveyor_start.start(True)
                        print("no object found!")
                        self.start_Once = 1
                        self.STOP_ONCE = 0


            if obj.type != "ur5":
                # This block runs when a package is detected

                if len(self._pkg_colour) == 12:
                    package_model_No = obj.type

                    if package_model_No != self.PREASENT_PKG:
                        print("{} has been detected".format(package_model_No))
                        package_model_colour = packageName()
                        package_model_colour.colour = self._pkg_colour[package_model_No]
                        ##################################
                        package_model_colour.pkgLocationId = package_model_No
                        self.PREASENT_PKG = package_model_No
                        self.package_name_pub.publish(package_model_colour)

                # This block is for stopping the belt
                if self.STOP_ONCE == 0:
                    if obj.pose.position.y < 0.23:
                        conveyor_start.start(False)   
                        self.STOP_ONCE = 1
                        ideal_flag = packageIdeal()
                        ideal_flag.Ideal = True
                        self.package_ideal_pub.publish(ideal_flag)
                        self.start_once = 0
                        self.start_Once = 0

                # This is for starting the belt after been picked
                if self.start_once == 0:
                    if obj.pose.position.z > 0.25:
                        conveyor_start.start(True)
                        self.start_once = 1
                        self.STOP_ONCE = 0

        else:

            # This is for the conveyor to start if nothing is under camera
            if self.start_Once == 0:
                conveyor_start.start(True)
                print("no object found!!")
                self.start_Once = 1
                self.STOP_ONCE = 0


    def find_pkgmordel(self, x, y, colour):
        '''This function is for saveing the colour with respect to its location'''
        # position_2 = []
        column = 0 if x < 150 else (1 if x >130 and x < 400 else 2)
        row = 0 if y < 320 else (1 if y > 320 and y < 500 else (2 if y > 500 and y < 650 else 3))
        position_1 = "packagen"+str(row)+str(column)
        item = "Medicine" if colour == "red" else ("Food" if colour == "yellow" else "Clothes")
        position_2 = [int(row), int(column), str(colour), item]

        if len(self._pkg_colour) <= 12:

            self._pkg_colour[position_1] = colour
            self._gSheet_update[position_1] = position_2

        if len(self._pkg_colour) == 12:

            self._boilercode["package_details"] = self._gSheet_update
            # print self._boilercode
            t = threading.Thread(name="Update",
                                target=Param.creat_inventory_pata,
                                args=[self._boilercode])
            t.start()

            try :
                with open(self._file_path,'w') as file_open:
                    yaml.dump(self._boilercode, file_open, default_flow_style=False)
            except:
                return False

            self._subForLimitedTime.unregister()

        return position_1


    def callback(self,data):
        '''This is responsible for all the image processing of 2d camera'''
        
        try:
            cv_image = self._bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        image = cv_image 
        gray_img = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        _, th1 = cv2.threshold(gray_img, 20, 210, cv2.THRESH_BINARY)

        qr_result = decode(th1)
        for barcode in qr_result:
            x,y,w,h = barcode.rect
            myData = barcode.data
            location = self.find_pkgmordel(x, y, myData)
            # cv2.rectangle(image,(x,y),(x+w,y+h),0,0,255,4)
            # text = myData
            # cv2.putText(image,text,(x+10,y-10),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)
            # cv2.imshow("/eyrc/vb/camera_1/image", th1)
        cv2.waitKey(3)


def main():
    '''Main function'''

    # flag = False
    camera_2 = MainaReader()

    # Param.creat_inventory_pata("hiii")

    # 1. Initialize the Subscriber Node.
    rospy.init_node('camera_reader', anonymous=True)

    # 2. Subscribe to the desired topic and attach a Callback Funtion to it.
    rospy.Subscriber("/eyrc/vb/logical_camera_2", LogicalCameraImage, camera_2.func_callback)
    
    camera_2._subForLimitedTime = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image,camera_2.callback)

    # 3. spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        # time.sleep(16)
        main()
    except rospy.ROSInterruptException:
        pass
