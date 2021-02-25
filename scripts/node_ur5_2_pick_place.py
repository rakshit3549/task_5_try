#! /usr/bin/env python

'''                             
    This Node is responsible for controlling the ur5_2 arm.
    This node also publishes conformation for every order 
    that has been prosessed.
'''

import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
import rospkg
import yaml
import time
import sys
from pkg_task5.msg import orderStatus
from pkg_task5.msg import packageName
from pkg_task5.msg import packageIdeal
from pkg_task5.msg import vacuumGripperInUse
from conveStart_vacuuStart import vacuum_start


class Ur5_2Moveit:
    
    def __init__(self, arg_robot_name):

        """
        Constructor

        """
        # Initialize the ROS Node.
        rospy.init_node('node_ur5_2', anonymous=True)
        self._robot_ns = '/'  + arg_robot_name
        self._planning_group = "manipulator"
        self._order_status_pub = rospy.Publisher('/orderStatus', orderStatus, queue_size=0)
        self._robot = moveit_commander.RobotCommander(robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher( self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient( self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()
        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._vacuumGripper_inUse = False
        self._package_ideal = False
        self._start_flage = True
        self._package_array = []
        self._pkg_colour = {}


        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rp = rospkg.RosPack()
        self._pkg_path = rp.get_path('pkg_task5')
        self._file_path = self._pkg_path + '/config/saved_trajectories_ur5_2/'

        rospy.loginfo('\033[94m' + " >>> Ur5_2Moveit init done." + '\033[0m')


    def func_callback_by_packageIdeal(self, package_ideal):

        """
        This function callback set the vale of self._package_ideal as true when
        if package ideal in the conveyor belt.
        
        Parameters
        ----------
        package_ideal.Ideal = Boolean
                Value of this valuable is changed to True if package ideal in the conveyor belt by this function,
                and its value gets back to false after that package has been picked

        Returns
        -------
        return : returns nothing

        """

        self._package_ideal = package_ideal.Ideal

    def func_callback_by_vacuumInUse(self, vacuumInUse):

        self._vacuumGripper_inUse = vacuumInUse.inUse

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
            with open(file_path, 'r') as file_open:
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
        	rospy.loginfo("attempts: {}".format(number_attempts) )
	return flag_success


    def error_corrector(self, location, status):

        """
        This function regains control, if failed to playing saved trajectory even after multiple attempts.
        In case the arm fails to playing saved trajectory this function would try bring the arm under 
        controller by replay the saved trajectory form all zero position to the end of the failed trajectory position. 

        Parameters
        ----------
        location : String
                This would be the name of the saved yaml file.

        status : Boolean
                This boolean value would be true is the package is attached to the arm
                and false is not attached

        Returns
        -------
        return : Boolean
            returns True if task executed successful else return false.
    
        """
                            
        rospy.logwarn(" ENTERED INTO ERROR CORRECTION LOOP.")

        # This if loop is entered if the ur5_2 is connected to the package 
        # which means the pkg and ur5_2 are on the conveyor and had to be disconnected 
        # before going forward with the error correction 
        if status:
            vacuum_start.start('ur5_2', False)
            flag_success = self.goto_pick()
            vacuum_start.start('ur5_2', True)
            flag_success = self.moveit_hard_play_planned_path_from_file(self._file_path, location, 5)

        # This if loop is entered if the ur5_2 is not connected to the package
        # which means the pkg would in its respected container and ur5_2 over container
        if status != True :
            flag_success = self.goto_pick()

        if flag_success != True:
            while not rospy.is_shutdown():
                rospy.logerr(" FAILED TO HANDLE THIS PACKAGE, PLEASE RELAUNCH THIS TASK; THANKYOU FROM UR5#2 ")
                time.sleep(3)
        else:
            rospy.loginfo('\033[92m' + " SUCCESSFULLY REGAINED CONTROL OVER THE TASK." + '\033[0m')

            return flag_success


    def goto_pick(self):

        """
        This function is to move the arm to the pick location that is over the conveyor
        and also helps of the arm enters the error corrector loop which is self.error_corrector()

        Returns
        -------
        return : Boolean
            returns True if task executed successful else return false.

        """

        self.go_to_predefined_pose("allZeros")
        if self._start_flage :
            rospy.loginfo('\033[94m' + " Initiating ur5_2 ." + '\033[0m')
            self._start_flage = False
        else:
            rospy.loginfo('\033[94m' + " Re-initiating ur5_2 ." + '\033[0m')
        flag_success = self.moveit_hard_play_planned_path_from_file(self._file_path, 'zero_to_pick.yaml', 5)

        return flag_success


    def pick_place_pkg(self, drop_location, pick_location, PrasentPkgColour, pesentPackageName):

            """
            This function is for moving the arm to the box location

            Parameters
            ----------
            drop_location : String
                        This would be the name of the saved yaml file 
                        that has saved the trajectory of the drop location.

            pick_location : String
                        This would be the name of the saved yaml file 
                        that has saved the trajectory of the pick location.

            PrasentPkgColour : String
                        This will the color of the package that has to by picked and doped.

            pesentPackageName : String
                        This would be name of the package that has to by picked and doped.
                        like.."packgen00"

            Returns
            -------
            return : returns nothing

            """

            rospy.loginfo('\033[94m' + "Picking "+ PrasentPkgColour +" Package." + '\033[0m')

            # Waiting for the package to reach the ur5_2 arm and the vacuum gripped service to get free
            enteredThisLoop1 = False
            while not(self._package_ideal and self._vacuumGripper_inUse != True):
                enteredThisLoop1 = self._vacuumGripper_inUse
                # print("!!!!!!!!!!!!1")
            if enteredThisLoop1:
                time.sleep(2)
            # Starting vacuum gripper
            vacuum_start.start('ur5_2', True)
            self._package_ideal = False
            rospy.loginfo('\033[94m' + "Picking "+ PrasentPkgColour +" Package Successfully Execute ." + '\033[0m')
            rospy.loginfo('\033[94m' + "Placing "+ PrasentPkgColour +" Package ." + '\033[0m')
            flag_success = False
            # Moving arm to drop location
            flag_success = self.moveit_hard_play_planned_path_from_file(self._file_path, drop_location, 5)
            # This if statement is executed if the self.moveit_hard_play_planned_path_from_file() returns false
            # And the arm enters the error correction loop
            if flag_success != True:
                flag_success = self.error_corrector(drop_location, True)
    
            enteredThisLoop2 = False
            while self._vacuumGripper_inUse != False:
                enteredThisLoop2 = self._vacuumGripper_inUse
                print("!!!!!!!!!!!!!!!!!!!!!!!!!2")
            if enteredThisLoop2:
                time.sleep(2)
            # Stopping vacuum gripper
            vacuum_start.start('ur5_2', False)
            # This publish a message that the package has be dropped(ie. Shipped) 
            order_status = orderStatus()
            order_status.arm = 'ur5_2'
            order_status.orderId = pesentPackageName
            print("!!!order_status",order_status)
            self._order_status_pub.publish(order_status)
            
            if flag_success:
                rospy.loginfo('\033[94m' + "Placing "+ PrasentPkgColour +" Package Successfully Execute ." + '\033[0m')
            flag_success = False
            # This is for bring the robot back to pick location
            flag_success = self.moveit_hard_play_planned_path_from_file(self._file_path, pick_location, 5)
            # This if statement is executed if the self.moveit_hard_play_planned_path_from_file() returns false
            # And the arm enters the error correction loop
            if flag_success != True:
                flag_success = self.error_corrector(pick_location, False)

            rospy.loginfo('\033[94m' + "Placing Package "+ str(PrasentPkgColour) +" Successfully Execute ." + '\033[0m')


    def func_callback_by_packageName(self, package_name):

            """
            This function append the current package color and its name to a list named ur5_2Moveit._package_array
            which is sent by node_camer_reader.py.
        
            Parameters
            ----------
            package_name.colour : String
                            This would be the color of the package that is ideal on the conveyor near the ur5_2.

            package_name.pkgLocationId : String
                            This would be the name of the package that is ideal on the conveyor near the ur5_2.
                            like.."packgen00" 

            Returns
            -------
            return : returns nothing

            """

            pesentPackageColor = package_name.colour
            pesentPackageName = package_name.pkgLocationId
            print("This is from service message {}".format(pesentPackageColor))
            self._package_array.append([pesentPackageColor, pesentPackageName])

    def VargiBot(self):
        """
        This is the Main function. This function run in a loop until the "Ur5_2Moveit._package_array"
        has some length which it will get by the callback function "Ur5_2Moveit.unc_callback_by_packageName()"
        that mean there is a package waiting to be picked.
        Once the if loop is entered the initial element of this list is removed after saving it value to a local variable.
        This saved local variable contains the package color which is used for placing the package 
        to its corresponding container and also  contains this name of the package (example. "packagen00")
        which would be in use while publish a message (ie. order_status)

        Returns
        -------
        return : returns nothing 

        """

        # Waiting for an order to come and append to self._package_array
        if (len(self._package_array)):
            PrasentPkgColour = self._package_array[0][0]
            pesentPackageName = self._package_array[0][1]
            self._package_array.pop(0)
            drop_location = "pick_to_"+PrasentPkgColour+".yaml"
            pick_location = PrasentPkgColour+"_to_pick.yaml"
            self.pick_place_pkg(drop_location, pick_location, PrasentPkgColour, pesentPackageName)
            self._package_ideal = False

        else:
            return



    def __del__(self):
        """
        Destructor
        """

        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5_2Moveit Deleted." + '\033[0m')
 
def main():
    
    """
    This is the main function

    """

    ur5_2 = Ur5_2Moveit('ur5_2')

    # 1. Subscribe to the desired topic and attach a Callback Function to it.
    rospy.Subscriber("/package_name", packageName, ur5_2.func_callback_by_packageName)

    rospy.Subscriber("/package_ideal", packageIdeal, ur5_2.func_callback_by_packageIdeal)

    rospy.Subscriber("/vacuumgripper_inuse", vacuumGripperInUse, ur5_2.func_callback_by_vacuumInUse)

    # 2. Bring the arm to the pick location 
    ur5_2.goto_pick()

    while not rospy.is_shutdown():

        ur5_2.VargiBot()
    
    #3. spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    del ur5_2

if __name__ == '__main__':
    try:
    	time.sleep(45)
        main()
    except rospy.ROSInterruptException:
        pass
