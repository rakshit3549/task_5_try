#! /usr/bin/env python

'''         This controls the Vacuum Gripper        '''

import rospy
from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import vacuumGripperRequest
from pkg_vb_sim.srv import vacuumGripperResponse


class Ur5Moveit:
    '''Main class '''

    def __init__(self):

        self._service_ur5_1 = '/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1'
        self._vacuum_gripper_ur5_1 = rospy.ServiceProxy(self._service_ur5_1, vacuumGripper)

        self._service_ur5_2 = '/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2'
        self._vacuum_gripper_ur5_2 = rospy.ServiceProxy(self._service_ur5_2, vacuumGripper)

    def vacuum_gripper(self,robot_name, boolean):
        '''Main function '''

        if robot_name == 'ur5_1':
            try:
                service_request_ur5_1 = vacuumGripperRequest(boolean)
                service_response_ur5_1 = self._vacuum_gripper_ur5_1(service_request_ur5_1)

                return vacuumGripperResponse(service_response_ur5_1.result)
            except:
                print("Service call failed")

        else:
            try:
                service_request_ur5_2 = vacuumGripperRequest(boolean)
                service_response_ur5_2 = self._vacuum_gripper_ur5_1(service_request_ur5_2)

                return vacuumGripperResponse(service_response_ur5_2.result)
            except:
                print("Service call failed")


def start(robot_name, boolean):

    global ur5

    if boolean:
        report = ur5.vacuum_gripper(robot_name, True)
        return report
    else:
        report = ur5.vacuum_gripper(robot_name, False)
        return report


'''start function'''
ur5 = Ur5Moveit()

if __name__ == '__main__':
    print("hiii")