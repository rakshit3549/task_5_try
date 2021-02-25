#! /usr/bin/env python

'''         This controls the Vacuum Gripper        '''

import rospy
from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import vacuumGripperRequest
from pkg_vb_sim.srv import vacuumGripperResponse


class Ur5Moveit:
    '''Main class '''

    def __init__(self, robot_name):

        if robot_name ==  'ur5_1':
            self._service = '/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1'
        else:
            self._service = '/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2'

    def vacuum_gripper(self, boolean):
        '''Main function '''

        try:
            vacuum_gripper = rospy.ServiceProxy(self._service,
                                                vacuumGripper)
            service_request = vacuumGripperRequest(boolean)
            service_response = vacuum_gripper(service_request)

            return vacuumGripperResponse(service_response.result)
        except:
            print("vacuumGripper Service call failed")


def start(robot_name, boolean):
    '''start function'''
    ur5 = Ur5Moveit(robot_name)

    if boolean:
        report = ur5.vacuum_gripper(True)
        return report
    else:
        report = ur5.vacuum_gripper(False)
        return report

if __name__ == '__main__':

    start('ur5_1', True)
