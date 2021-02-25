#!/usr/bin/env python

"""
    This Node stops the conveyor belt at desired package location at the ur5_2 side of the conveyor bleat 
    and also publishes the color of the package which is then used by node_uer5_2_pick_place.py to
    place the package in their respective container. This is also responsible for updating
    the Inventory Spreadsheet, while doing so it also save the shelf package detailed 
    in a file named package_priority.yaml of use by node_online_manager.py. 
     
"""
import time
import rospy
import cv2
import rospkg
import yaml
import json
import threading
from parameter_generator import Param
# from parameterGenerator.parameter_generator import *
# # from parameterGenerator.parameter_generator import Param

from pkg_vb_sim.msg import LogicalCameraImage
from pkg_task5.msg import packageName
from pkg_task5.msg import packageIdeal
from conveStart_vacuuStart import conveyor_start
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode
from sensor_msgs.msg import Image


class MainaReader:

    def __init__(self):

        """
        Constructor
        """

        rp = rospkg.RosPack()
        pkg_path = rp.get_path('pkg_task5')
        self._file_path = pkg_path + '/config/package_priority.yaml'
        self._boilercode = {'package_details': {}}
        self._bridge = CvBridge()
        self.PREASENT_PKG = "non"
        self.runonce = True 
        self.STOP_ONCE = 0
        self.start_once = 0
        self.start_Once = 0
        self._pkg_colour = {}
        self._gSheet_update = {}
        self._subForLimitedTime = ""
        self.package_name_pub = rospy.Publisher('/package_name', packageName, queue_size=0)
        self.package_ideal_pub = rospy.Publisher('/package_ideal', packageIdeal, queue_size=0)

    def func_callback_logic_camera(self, data):

        """
        This is the main conveyor belt Start/stopping method
        The flags in the Constructor are there so that the start and stop command for the 
        conveyor bleat is excited ONLY ONCE for each package as we are running this method in a loop.
        This method also publishes the color of the current package under the logical camera.

        Parameters
        ----------
        data : List
            This will be the list of items and their details that the logic camera 2 detects.

        Returns
        -------
        return : returns nothing

        """

        index = 0
        
        # This is to check if 'ur5' or any package is under camera.
        if (len(data.models)):
            obj = data.models[index]
            if obj.type == "ur5":
                try:
                    index = 1
                    obj = data.models[index]
                except:
                    # This is for the conveyor to start if package is not under camera.
                    if self.start_Once == 0:
                        conveyor_start.start(True)
                        print("no object found!")
                        self.start_Once = 1
                        self.STOP_ONCE = 0

            # This block runs when a package is under the camera.
            if obj.type != "ur5":
                # This if statement is excused if all the packages has been detected on the shelf.
                if len(self._pkg_colour) == 12:
                    package_model_No = obj.type
                    # To check if the package is same one or a new package has arrived.
                    # if its a new package a Ros message containing is package color and 
                    # and package name is published which ur5_2 uses.
                    if package_model_No != self.PREASENT_PKG:
                        print("{} has been detected".format(package_model_No))
                        package_model_colour = packageName()
                        package_model_colour.colour = self._pkg_colour[package_model_No]
                        package_model_colour.pkgLocationId = package_model_No
                        self.PREASENT_PKG = package_model_No
                        # Publishing Ros messages
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

                # # This is for starting the belt after been picked
                # if self.start_once == 0:
                #     if obj.pose.position.z > 0.25:
                #         conveyor_start.start(True)
                #         self.start_once = 1
                #         self.STOP_ONCE = 0

        else:

            # This is for the conveyor to start if nothing is under camera
            if self.start_Once == 0:
                conveyor_start.start(True)
                print("no object found!!")
                self.start_Once = 1
                self.STOP_ONCE = 0


    def find_pkgmodel(self, x, y, colour):
        """
        This method is for saving the colour with respect to its location.
        With the help of two parameters the x,y the package name is obtained
        and the item is determined with the help of the parameter colour.
        once all the packages are located it is the send to updating the 
        Google spreadsheets

        Parameters
        ----------
        x : Int
            This value represents how much to the right the package is from the starting
            point of frame which is 0,0 at left,top. This gives the column no of the package.

        y: Int
            This value represents how much to the bottom is the package form the 0,0 position
            and this gives the row number of the priority.

        colour : String
            This will be the color of the package

        Returns
        -------
        return : returns nothing 

        """
        # Finding row and column of the package
        column = 0 if x < 150 else (1 if x >130 and x < 400 else 2)
        row = 0 if y < 320 else (1 if y > 320 and y < 500 else (2 if y > 500 and y < 650 else 3))

        # Here we get package name like.."packagen00"
        position_1 = "packagen"+str(row)+str(column)
        item = "Medicine" if colour == "red" else ("Food" if colour == "yellow" else "Clothes")
        position_2 = [int(row), int(column), str(colour), item]

        if len(self._pkg_colour) <= 12:
            # This list will be used for recognizing the color of the package 
            # after it gets detected  by logical camera 2
            self._pkg_colour[position_1] = colour
            # This list is sent as a parameter to updating the Google spread sheet.
            self._gSheet_update[position_1] = position_2

        if len(self._pkg_colour) == 12 and self.runonce :

            self.runonce = False
            self._boilercode["package_details"] = self._gSheet_update
    
            print("sheets")

            # Calling module parameter_generator to send action messages to update sheets.
            t = threading.Thread(name="Update",
                                target=Param.creat_inventory_pata,
                                args=[self._boilercode])
            t.start()
            
            print("sheets done")

            # Creating the a yaml file, consisting of the package details.
            try :
                with open(self._file_path,'w') as file_open:
                    yaml.dump(self._boilercode, file_open, default_flow_style=False)
            except Exception as e:
                    raise e
                

            # unsubscribing to the topic 2d camera feed
            self._subForLimitedTime.unregister()




    def callback(self,data):
        """
        This is responsible for all the image processing of 2d camera
        is this method image data is converted to Gray image then
        applied a binary thresholding so that the QR Cord easily
        detected. then this binary image is passed though a decode() 
        method to get all the barcode from the image.

        Parameters
        ----------
            data : List
                This list is the out put form the 2d camera

        Returns
            -------
            return : returns nothing

        """
        
        try:
            cv_image = self._bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        image = cv_image
        # Converting BGR image to GRAY image.
        gray_img = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        # Appalling binary thresholding to the gray image.
        _, th1 = cv2.threshold(gray_img, 39, 255, cv2.THRESH_BINARY)

        # Decoding the binary image to find QRcods in it.
        qr_result = decode(th1)
        for barcode in qr_result:
            x,y,w,h = barcode.rect
            myData = barcode.data
            self.find_pkgmodel(x, y, myData)

        cv2.waitKey(3)


def main():
    """
    Main Function
    """

    camera_2 = MainaReader()


    # 1. Initialize the ROS Node.
    rospy.init_node('camera_reader', anonymous=True)

    # 2. Subscribe to the desired topic and attach a Callback method to it.
    rospy.Subscriber("/eyrc/vb/logical_camera_2", LogicalCameraImage, camera_2.func_callback_logic_camera)
    
    camera_2._subForLimitedTime = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image,camera_2.callback)

    # 3. spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        time.sleep(35)
        main()
    except rospy.ROSInterruptException:
        pass
