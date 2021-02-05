#! /usr/bin/env python

'''                             This Node is responsible for controlling the ur5_2 arm which 
                                    place the required package on the conveyor belt                            '''

import rospy
import moveit_commander
import moveit_msgs.msg
import cv2
import actionlib
import rospkg
import yaml
import time
import sys
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode
from sensor_msgs.msg import Image
from pkg_task4.msg import packageName
from pkg_task4.msg import packageIdeal
from conveStart_vacuuStart import vacuum_start



class Ur5_2Moveit:
    
    def __init__(self, arg_robot_name):
        ''' Constructor'''

        rospy.init_node('node_ur5_2', anonymous=True)
        self._robot_ns = '/'  + arg_robot_name
        self._planning_group = "manipulator"
        
        # sys.argv.append(arg_robot_name) 
        self._robot = moveit_commander.RobotCommander(robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher( self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient( self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()
        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._bridge = CvBridge()
        self._package_ideal = False
        self._start_flage = True
        self._package_array = ["yellow"]
        self._pkg_colour = {}


        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rp = rospkg.RosPack()
        self._pkg_path = rp.get_path('pkg_task4')
        self._file_path = self._pkg_path + '/config/saved_trajectories_ur5_2/'

        rospy.loginfo('\033[94m' + " >>> Ur5_2Moveit init done." + '\033[0m')


    def go_to_predefined_pose(self, arg_pose_name):
        '''This is for moving the arm to difined pose'''

        rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_named_target(arg_pose_name)
        plan = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')


    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
        '''This function is responsible for playing saved trajectory'''

        file_path = arg_file_path + arg_file_name
        try :
            with open(file_path, 'r') as file_open:
                loaded_plan = yaml.load(file_open)
            ret = self._group.execute(loaded_plan)
            return ret
        except:
            return False


    def moveit_hard_play_planned_path_from_file(self, arg_file_path, arg_file_name, arg_max_attempts):
        '''This function allows maltipal attempts for playing saved trajectory'''

	number_attempts = 0
	flag_success = False
	while ( (number_attempts <= arg_max_attempts) and (flag_success is False) ):
		number_attempts += 1
        	flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
        	rospy.loginfo("attempts: {}".format(number_attempts) )
	return flag_success


    def error_corrector(self, *args):
        '''This magical function regains control over the task when is fails to 
                            playing saved trajectory                              '''
                            
        rospy.logwarn(" ENTERED INTO ERROR CORRECTION LOOP.")
        if len(args) == 2:
            vacuum_start.start('ur5_2', False)
        flag_success = self.goto_pick()
        if len(args) == 2:
                vacuum_start.start('ur5_2', True)
                flag_success = self.moveit_hard_play_planned_path_from_file(self._file_path, args[0], 5)

        if flag_success != True:
            while not rospy.is_shutdown():
                rospy.logerr(" FAILED TO HANDLE THIS PACKAGE, PLEASE RELAUNCH THIS TASK; THANKYOU FROM UR5#2 ")
                time.sleep(3)
        else:
            rospy.loginfo('\033[92m' + " SUCCESSFULLY REGAINED CONTROL OVER THE TASK." + '\033[0m')

            return True


    def goto_pick(self, *args):
        '''This is to move the arm to the pick location'''

        self.go_to_predefined_pose("allZeros")
        if self._start_flage :
            rospy.loginfo('\033[94m' + " Initiating ur5_2 ." + '\033[0m')
            self._start_flage = False
            # if self._start_flage != True:
            #     vacuum_start.start('ur5_2', False)
        else:
            rospy.loginfo('\033[94m' + " Reinitiating ur5_2 ." + '\033[0m')
        flag_success = self.moveit_hard_play_planned_path_from_file(self._file_path, 'zero_to_pick.yaml', 5)
        return flag_success


    def pick_place_pkg(self, drop_location, pick_location, PrasentPkgColour):
            '''This is to move the arm to the box location'''

            rospy.loginfo('\033[94m' + "Picking "+ PrasentPkgColour +" Package." + '\033[0m')
            while self._package_ideal != True:
                pass
            vacuum_start.start('ur5_2', True)
            # self._package_ideal = False
            rospy.loginfo('\033[94m' + "Picking "+ PrasentPkgColour +" Package Successfully Execute ." + '\033[0m')
            rospy.loginfo('\033[94m' + "Placing "+ PrasentPkgColour +" Package ." + '\033[0m')
            flag_success = False
            flag_success = self.moveit_hard_play_planned_path_from_file(self._file_path, drop_location, 5)
            if flag_success != True:
                flag_success = self.error_corrector(drop_location, True)
            vacuum_start.start('ur5_2', False)
            if flag_success:
                rospy.loginfo('\033[94m' + "Placing "+ PrasentPkgColour +" Package Successfully Execute ." + '\033[0m')
            flag_success = False
            flag_success = self.moveit_hard_play_planned_path_from_file(self._file_path, pick_location, 5)
            if flag_success != True:
                flag_success = self.error_corrector(pick_location)


    def VargiBot(self):
        '''Main drop function'''

        if (len(self._package_array)):
            PrasentPkgColour = self._package_array[0]

            drop_location = "pick_to_"+PrasentPkgColour+".yaml"
            pick_location = PrasentPkgColour+"_to_pick.yaml"
            self.pick_place_pkg(drop_location, pick_location, PrasentPkgColour)
            self._package_array.pop(0)
            self._package_ideal = False

        else:
            return


    def func_callback_by_packageName(self, package_name):
        '''This function update the package colour '''

        self._pesent_package = package_name.colour
        print("This is from service message {}".format(self._pesent_package))
        self._package_array.append(self._pesent_package)


    def func_callback_by_packageIdeal(self, package_ideal):
        '''This function answers if package is ideal in the conveyor belt or not'''

        self._package_ideal = package_ideal.Ideal



    def __del__(self):
        '''Destructor'''

        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5_2Moveit Deleted." + '\033[0m')
 
def main():
    
    ''' This is the main function '''

    ur5_2 = Ur5_2Moveit('ur5_2')

    rospy.Subscriber("package_name", packageName, ur5_2.func_callback_by_packageName)

    rospy.Subscriber("package_ideal", packageIdeal, ur5_2.func_callback_by_packageIdeal)

    ur5_2.goto_pick()

    while not rospy.is_shutdown():
        ur5_2.VargiBot()
    
    rospy.spin()

    del ur5_2

if __name__ == '__main__':
    try:
    	# time.sleep(17)
        main()
    except rospy.ROSInterruptException:
        pass
