#! /usr/bin/env python

"""        
    This Node is responsible for controlling the ur5_1 arm which 
    place the required package on the conveyor belt.
    This node also publishes conformation for every order 
    that has been processed.
"""

import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
import rospkg
import yaml
import time
import sys
from pkg_task5.msg import pkgLocation
from pkg_task5.msg import orderStatus
from pkg_task5.msg import vacuumGripperInUse
from conveStart_vacuuStart import vacuum_start



class Ur5_1Moveit:

    def __init__(self, arg_robot_name):
        """
        Constructor
        """

        rospy.init_node('node_ur5_1', anonymous=True)
        self._robot_ns = '/'  + arg_robot_name
        self._planning_group = "manipulator"
        sys.argv.append(arg_robot_name)
        self._order_status_pub = rospy.Publisher('/orderStatus', orderStatus, queue_size=0)
        self._vacuumGripper_inUse = rospy.Publisher('/vacuumgripper_inuse', vacuumGripperInUse, queue_size=0)
        self._robot = moveit_commander.RobotCommander(robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher( self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient( self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self.start_flage = True

        self.high_priority_pkg = [] #[["Mpkg00",3001]]


        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rp = rospkg.RosPack()
        self._pkg_path = rp.get_path('pkg_task5')
        self._file_path = self._pkg_path + '/config/saved_trajectories_ur5_1/'
        rospy.loginfo('\033[94m' + " >>> Ur5_1Moveit init done." + '\033[0m')
                
        
    def go_to_predefined_pose(self, arg_pose_name):
        
        """
        This function is for moving the arm to predefined pose

        Parameters
        ----------
        arg_pose_name : String
                    This would the name for the predefined pose

        Returns
        -------
        return : returns nothing        

        """

        rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_named_target(arg_pose_name)
        plan = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')


    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
        
        """
        This function is responsible for playing saved trajectory

        Parameters
        ----------
        arg_file_path : String
                    This would be the path of saved yaml file.

        arg_file_name : String
                    This would be the name of the saved yaml file.

        Returns
        -------
        return : Boolean
            returns True if task executed successful else return false.

        """

        file_path = arg_file_path + arg_file_name
        try :
            with open(file_path, 'r') as file_open :
                loaded_plan = yaml.load(file_open)
            ret = self._group.execute(loaded_plan)
            if file_open is not None:
                file_open.close()
            return ret
        except:
            return False

    def moveit_hard_play_planned_path_from_file(self, arg_file_path, arg_file_name, arg_max_attempts):
        
        """
        This function does multiple attempts for playing saved trajectory

        Parameters
        ----------
        arg_file_path : String
                        This would be the path of saved yaml file.

        arg_file_name : String
                        This would be the name of the saved yaml file.

        arg_max_attempts : Int 
                        This is the number of it has to make if the playing saved trajectory fails.

        Returns
        -------
        return : Boolean
            returns True if task executed successful else return false.

        """

        number_attempts = 0
        flag_success = False
        while ( (number_attempts <= arg_max_attempts) and (flag_success is False) ):
            number_attempts += 1
            flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
            print("!!!!!!!!", flag_success)
            rospy.loginfo("attempts: {}".format(number_attempts) )
        return flag_success


    def error_corrector(self, pick_location, drop_location, status):
        
        """
        This function regains control, if failed to playing saved trajectory even after multiple attempts.
        In case the arm fails to playing saved trajectory this function would try bring the arm under 
        controller by replay the saved trajectory form all zero position to the end of the failed trajectory position.

        Parameters
        ----------
        pick_location : String
                This would be the name of the saved yaml file of package to be picked

        drop_location : String
                This would be the name of the saved yaml file of package to be doped

        status : Boolean
                This boolean value would be true is the package is attached to the arm
                and false is not attached

        Returns
        -------
        return : Boolean
            returns True if task executed successful else return false.
    
        """

        rospy.logwarn(" ENTERED INTO ERROR CORRECTION LOOP.")

        # This if loop is entered if the ur5_1 is connected to the package 
        # which means the pkg and ur5_1 are on at the shelf and had to be disconnected 
        # before going forward with the error correction
        if status:
            vacuum_start.start('ur5_1', False)
            self.goto_drop()
            flag_success = self.moveit_hard_play_planned_path_from_file(self._file_path, pick_location, 5)
            vacuum_start.start('ur5_1', True)
            flag_success = self.moveit_hard_play_planned_path_from_file(self._file_path, drop_location, 5)
       
        # This if loop is entered if the ur5_1 is not connected to the package
        # which means the ur5_1 over conveyor
        if status != True:
            self.goto_drop()
            flag_success = self.moveit_hard_play_planned_path_from_file(self._file_path, pick_location, 5)

        if flag_success != True:
            while not rospy.is_shutdown():
                rospy.logerr(" FAILED TO HANDLE THIS PACKAGE, PLEASE RELAUNCH THIS TASK; THANKYOU FROM UR5#1 ")
                time.sleep(3)
        else:
            rospy.loginfo('\033[92m' + " SUCCESSFULLY REGAINED CONTROL OVER THE TASK." + '\033[0m')

            return flag_success

    def publish_vacuumgripper_status(self, status):
        
        vacuumgripper_status = vacuumGripperInUse()
        vacuumgripper_status.inUse = status
        self._vacuumGripper_inUse.publish(vacuumgripper_status)


    def goto_drop(self):

        """
        This function is to move the arm to the pick location that is over the conveyor
        and also helps of the arm enters the error corrector loop which is self.error_corrector()

        Returns
        -------
        return : Boolean
            returns True if task executed successful else return false.

        """

        self.go_to_predefined_pose("allZeros")
        if self.start_flage :
            rospy.loginfo('\033[94m' + " Initiating ur5_1 ." + '\033[0m')
            self.start_flage = False
        else:
            rospy.loginfo('\033[94m' + " Re-initiating ur5_1 ." + '\033[0m')
        flag_success = self.moveit_hard_play_planned_path_from_file(self._file_path, 'zero_to_drop.yaml', 5)
        
        return flag_success


    def pick_place_pkg(self,order_id, pick_location, drop_location, location):
            
            """
            This function is for moving the arm to the box location

            Parameters
            ----------
            order_id : Int
                    This would by the order id for the current package which is in use
                    while publishing the package(order) status through
                    Ros messages to node_online_order_manager.py

            drop_location : String
                        This would be the name of the saved yaml file 
                        that has saved the trajectory of the drop location.

            pick_location : String
                        This would be the name of the saved yaml file 
                        that has saved the trajectory of the pick location.

            location : String
                        This will the package location in row,column 
                        like.."00" or "01"

            Returns
            -------
            return : returns nothing

            """

            rospy.loginfo('\033[94m' + "Picking Package "+ str(location) + '\033[0m')
            flag_success = False
            # Moving arm to pick location
            flag_success = self.moveit_hard_play_planned_path_from_file(self._file_path, pick_location, 5)
            # This if statement is executed if the self.moveit_hard_play_planned_path_from_file() returns false
            # And the arm enters the error correction loop
            if flag_success != True:
                flag_success = self.error_corrector(pick_location, drop_location, False)
            flag_success = False

            # Publishing that vacuum griper is in use.
            self.publish_vacuumgripper_status(True)
            # Starting vacuum gripper
            vacuum_start.start('ur5_1', True)
            self.publish_vacuumgripper_status(False)
            rospy.loginfo('\033[94m' + "Picking Package "+ str(location) +" Successfully Execute ." + '\033[0m')
            rospy.loginfo('\033[94m' + "Placing Package "+ str(location) + '\033[0m')
            # This is for bring the robot back to the conveyor for placing the package location
            flag_success = self.moveit_hard_play_planned_path_from_file(self._file_path, drop_location, 5)
            # This if statement is executed if the self.moveit_hard_play_planned_path_from_file() returns false
            # And the arm enters the error correction loop
            if flag_success != True:
                flag_success = self.error_corrector(pick_location, drop_location, True)

            self.publish_vacuumgripper_status(True)
            # Stopping vacuum gripper
            vacuum_start.start('ur5_1', False)
            self.publish_vacuumgripper_status(False)
            
            # This publish a message that the package has be placed on conveyor(ie. Dispatched)
            order_status = orderStatus()
            order_status.arm = 'ur5_1'
            order_status.orderId = order_id
            self._order_status_pub.publish(order_status)
            
            rospy.loginfo('\033[94m' + "Placing Package"+ str(location) +" Successfully Execute ." + '\033[0m')


    def func_callback_online_order(self, my_msg):

        """
        This function does the work of arranging the order as per their priorities.
        This callback function is called when ever a new order has been received.
        my_msg.pkg_location is a string formed by the initial letter of the order item
        followed by the package name for example for "food" item its "my_msg.pkg_location"
        would be "Fpackagen00". This formate is created and send by
        the node_online_manager. this formate can now appended to a list sort that
        list with reverse=True every a new order has been received.

        Parameters
        ----------
        my_msg.order_id : Int
                        This is the order Id of the received which will be in use for
                        publishing ROS messages after the order has been is processed.

        my_msg.pkg_location : String
                        This string is created by using first letter of the item in the order
                        and the package name. for example "Fpackagen00","Mpackagen02"
                        this formate is much simpler to sort as per priority.

        Returns
        -------
            return : returns nothing

        """
        
        order_id = my_msg.order_id
        pkg_location = my_msg.pkg_location
        rospy.loginfo('\033[94m'+str(order_id)+pkg_location+'\033[0m')
        self.high_priority_pkg.append([pkg_location,order_id])
        self.high_priority_pkg.sort(reverse=True)


    def VargiBot(self):

        """
        This is the Main function. This function run in a loop until the "Ur5_1Moveit.high_priority_pkg"
        has some length which it will get by the callback function "Ur5_1Moveit.func_callback_online_order()"
        that mean an order has been placed .
        Once the if loop is entered the initial element of this list is removed after saving it value to a local variable.
        This saved local variable contains the package location which is used for the removing the required package from 
        the shelf and it also  contains this order_id of the order which would 
        be in use while publish a message (ie. order_status)
        As the package location if in the formate "Fpackagen00" package location can be easy extracted by [-2:]

        Returns
        -------
        return : returns nothing 

        """

        # Waiting for an order to come and append to self.high_priority_pkg
        if len(self.high_priority_pkg):
            plocation = self.high_priority_pkg[0]
            location = str(plocation[0][-2:])
            order_id = plocation[1]
            self.high_priority_pkg.remove(plocation)
            self.pick_place_locator(order_id,location)
        else:
            return True


    def pick_place_locator(self, order_id, location):

        """
        This function get the location and concatenate it and creates two file name
        pick_location and drop_location. Which is then send for processing

        Parameters
        ----------
            order_id : String
                    This is the order Id of the correct order

            location : String
                    This is in formate row column example "00"or"01"...

        """

        pick_location = "drop_to_"+str(location)+".yaml"
        drop_location = str(location)+"_to_drop.yaml"
        self.pick_place_pkg(order_id, pick_location, drop_location, location)


    def __del__(self):

        """
        Destructor
        """
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5_1Moveit Deleted." + '\033[0m')

def main():

    """
    This is the main function

    """

    try:

        ur5_1 = Ur5_1Moveit('ur5_1')

        # 1. Subscribe to the desired topic and attach a Callback Function to it.
        rospy.Subscriber("/location", pkgLocation, ur5_1.func_callback_online_order)

        # 2. Bring the arm to the drop location 
        ur5_1.goto_drop()

        while not rospy.is_shutdown():
            
            ur5_1.VargiBot()

        # 3. spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

        del ur5_1

    except rospy.ROSInterruptException:
        print("Something went wrong !")


if __name__ == '__main__':
    try:
        time.sleep(55)
        main()
    except rospy.ROSInterruptException:
        pass
