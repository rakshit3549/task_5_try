#! /usr/bin/env python

'''                             This Node is responsible for controlling the ur5_1 arm
                                    which does the segregation of package                               '''

import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
import rospkg
import yaml
import time
import sys
from conveStart_vacuuStart import vacuum_start



class Ur5_1Moveit:

    def __init__(self, arg_robot_name):
        ''' Constructor'''

        rospy.init_node('node_ur5_1', anonymous=True)
        self._robot_ns = '/'  + arg_robot_name
        self._planning_group = "manipulator"
        
        sys.argv.append(arg_robot_name) 
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

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rp = rospkg.RosPack()
        self._pkg_path = rp.get_path('pkg_task4')
        self._file_path = self._pkg_path + '/config/saved_trajectories_ur5_1/'
        rospy.loginfo('\033[94m' + " >>> Ur5_1Moveit init done." + '\033[0m')

        
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
            vacuum_start.start('ur5_1', False)
        self.goto_drop()
        if len(args) != 2:
            flag_success = self.moveit_hard_play_planned_path_from_file(self._file_path, args[0], 5)
        else:
            flag_success = False

        if flag_success != True:
            while not rospy.is_shutdown():
                rospy.logerr(" FAILED TO HANDLE THIS PACKAGE, PLEASE RELAUNCH THIS TASK; THANKYOU FROM UR5#1 ")
                time.sleep(3)
        else:

            rospy.loginfo('\033[92m' + " SUCCESSFULLY REGAINED CONTROL OVER THE TASK." + '\033[0m')


    def goto_drop(self):
        '''This is to move the arm to the drop location'''

        self.go_to_predefined_pose("allZeros")

        if self.start_flage :
            rospy.loginfo('\033[94m' + " Initiating ur5_1 ." + '\033[0m')
            self.start_flage = False
        else:
            rospy.loginfo('\033[94m' + " Reinitiating ur5_1 ." + '\033[0m')
        self.moveit_hard_play_planned_path_from_file(self._file_path, 'zero_to_drop.yaml', 5)


    def pick_place_locator(self, location):
        pick_location = "drop_to_"+str(location)+".yaml"
        drop_location = str(location)+"_to_drop.yaml"
        self.pick_place_pkg(pick_location, drop_location, location)


    def pick_place_pkg(self, pick_location, drop_location, location):
            '''This is to move the arm to the location'''

            rospy.loginfo('\033[94m' + "Picking Package"+ str(location) + '\033[0m')
            flag_success = False
            flag_success = self.moveit_hard_play_planned_path_from_file(self._file_path, pick_location, 5)
            if flag_success != True:
                self.error_corrector(pick_location)
            flag_success = False
            vacuum_start.start('ur5_1', True)
            rospy.loginfo('\033[94m' + "Picking Package"+ str(location) +" Successfully Execute ." + '\033[0m')
            rospy.loginfo('\033[94m' + "Placing Package"+ str(location) + '\033[0m')
            flag_success = self.moveit_hard_play_planned_path_from_file(self._file_path, drop_location, 5)
            if flag_success != True:
                self.error_corrector(drop_location, True)
            vacuum_start.start('ur5_1', False)
            rospy.loginfo('\033[94m' + "Placing Package"+ str(location) +" Successfully Execute ." + '\033[0m')


    def __del__(self):
        '''Destructor'''
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5_1Moveit Deleted." + '\033[0m')

def main():
    ''' This is the main function '''

    try:

        ur5_1 = Ur5_1Moveit('ur5_1')

        ur5_1.goto_drop()
        
        ur5_1.pick_place_locator("00")
        ur5_1.pick_place_locator("01")
        ur5_1.pick_place_locator("02")
        ur5_1.pick_place_locator("10")
        ur5_1.pick_place_locator("11")
        ur5_1.pick_place_locator("12")
        ur5_1.pick_place_locator("20")
        ur5_1.pick_place_locator("22")
        ur5_1.pick_place_locator("30")


        del ur5_1

    except rospy.ROSInterruptException:
        print("Something went wrong !")


if __name__ == '__main__':
    try:
        #time.sleep(15)
        main()
    except rospy.ROSInterruptException:
        pass
