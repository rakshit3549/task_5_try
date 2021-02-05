#! /usr/bin/env python
'''         This controls the Conveyor belt         '''

import rospy
from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_vb_sim.srv import conveyorBeltPowerMsgRequest
from pkg_vb_sim.srv import conveyorBeltPowerMsgResponse

class Ur5Moveit:
    '''Main class '''
    def __init__(self):
        pass

    def startConveyor(self, set_power):
        '''Main function '''

        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        try:
            start_conveyor = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power',
                                                conveyorBeltPowerMsg)
            service_request = conveyorBeltPowerMsgRequest(set_power)
            service_response = start_conveyor(service_request)

            return conveyorBeltPowerMsgResponse(service_response.result)

        except rospy.ServiceException:
            print("Service call failed")

def start(boolean):
    '''start function'''

    ur5 = Ur5Moveit()
    if boolean:
        report = ur5.startConveyor(100)
        return report
    else:
        report = ur5.startConveyor(0)
        return report
        
if __name__ == '__main__':

    start(True)
