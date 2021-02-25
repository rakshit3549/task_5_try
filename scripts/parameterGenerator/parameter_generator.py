"""
        This is this module that establishes communication between the ROS nodes and ROS_IOT bridge.
        This module is called by online_order_manager.py to send action messages to ROS_IOT bridge in case
        the Google spreadsheet needs to be updated.

"""
import time
import rospy
import actionlib
import json
from pkg_ros_iot_bridge.msg import msgRosIotAction
from pkg_ros_iot_bridge.msg import msgRosIotGoal

class Para_grn(object):
    """
    This is the main class of this module
    """
    def __init__(self, arg):
        """
        Construct object of SimpleActionClient
        """
        super(Para_grn, self).__init__()
        self.arg = arg
        # self.CLIENT_BRIDGE = actionlib.SimpleActionClient('/action_ros_iot', msgRosIotAction)
        rospy.loginfo("This Action server is up, we can send results!")

    def send_para(self, Id, datas):
        """
        This function is responsible for setting communication with ROS_IOT bridge.
        It send the 'data' parameter as Ros action messages to ROS_IOT bridge
        for updation on sheets.

        Parameters
        ----------
        Id : String
            This is the sheet name that need to get updated.

        datas : Dictionary
            Details of package to send through action files.

        Returns
        -------
        return : returns nothing 

        """

        goal_bridge = msgRosIotGoal(data=str(datas), sheetName=Id)
        self.CLIENT_BRIDGE.send_goal(goal_bridge)
        rospy.loginfo("Sent result to action bridge")

    def creat_inventory_pata(self, datas):
        """
        This function is called when the inventory sheets are needed to be updated.
        This function is called by node_camer_reader.py and it passes details of all packages to this function.
        After getting the data it loops through it sorts the package in an order and creates a new dictionary named "final_para"
        with additional details for the packages. The additional details are depended on the color of the package 
        and the its location. The newly generated dictionary and the sheet name is sent to self.send_para() 
        for sending the action messages to ROS_IOT bridge.

        Parameters
        ----------
        data : List
            This contains the list of all the package details like 
            package color and its location 

        Returns
        -------
        return : returns nothing
        
        """
        data = datas["package_details"]

        # Sorting the package name for sending Google spreadsheet update request in order

        sorted_keys = sorted(data.keys())
        for key in sorted_keys:
            final_para = {"qty_invet":"1"}
            for value in data.values():
                if data[key] == value:
                    if value[2] == "red":
                        prefixes = "R"
                        item = "Medicine"
                    elif value[2] == "yellow":
                        prefixes = "Y"
                        item = "Food"
                    else:
                        prefixes = "G"
                        item = "Clothes"
                    final_para["SKU"] = str(prefixes+str(value[0])+str(value[1])+str(time.strftime("%m%y")))
                    final_para["storageNo"] = str("R"+str(value[0])+" C"+str(value[1]))
                    final_para["item"] = item
                    
                    # Sending the package detailed am Sheet name to self.send_para() to get send to
                    # ROS_IOT bridge for updation the Google spread sheet.

                    self.send_para("Inventory", final_para)
                    time.sleep(3)

    def order_update(self, arm, data):
        """
        This function is called when the order details of an particular order needs to be updated.
        Depending upon the name if the arm that sent Google spreadsheet update request
            this function provide the Google spreadsheet name. After that self.send_para() is called
            to sent the sheet name and data to be send to ROS_IOT bridge.

        Parameters
        ----------
        arm : String 
            This is the name arm that sent request for Google spreadsheet to be updated
            The string would be either "ur5_1" or "ur5_2"  

        data : Dictionary
            This is the order details the need to get updated 

        Returns
        -------
        return : returns nothing

        """

        print("order_update",type(data))

        if arm == "ur5_1":
            sheetName = "OrdersDispatched"
            # final_para[]
        else:
            sheetName = "OrdersShipped"

        self.send_para(sheetName, data)

if __name__ == 'parameter_generator':

    Param = Para_grn("para")

