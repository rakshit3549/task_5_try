import time
import rospy
import actionlib
import json
from pkg_ros_iot_bridge.msg import msgRosIotAction
from pkg_ros_iot_bridge.msg import msgRosIotGoal

class Para_grn(object):
    """docstring for Para_grn"""
    def __init__(self, arg):
        super(Para_grn, self).__init__()
        self.arg = arg
        self.CLIENT_BRIDGE = actionlib.SimpleActionClient('/action_ros_iot', msgRosIotAction)
        # self.CLIENT_BRIDGE.wait_for_server()
        rospy.loginfo("This Action server is up, we can send results!")

    def send_para(self, Id, datas):
        # datas = '{"city": "Mumbai", "order_time": "2021-02-02 13:07:20", "order_id": "3001", "lon": "72.8777 E", "qty": "1", "item": "Food", "lat": "19.0760 N"}'
        goal_bridge = msgRosIotGoal(data=str(datas), sheetName=Id)
        self.CLIENT_BRIDGE.send_goal(goal_bridge)
        rospy.loginfo("Sent result to action bridge")

    def creat_pata(self, datas):
        # print(datas)
        # datas = json.loads(datas)
        print(type(datas))

        # para = {"qty": "1"}
        # datas = {'package_details': {'package_colour': {'packagen31': [3, 1, 'green'], 'packagen12': [1, 2, 'red'],
        #                                                 'packagen11': [1, 1, 'yellow'], 'packagen10': [1, 0, 'green'],
        #                                                 'packagen22': [2, 2, 'yellow'], 'packagen30': [3, 0, 'yellow'],
        #                                                 'packagen20': [2, 0, 'green'], 'packagen32': [3, 2, 'red'],
        #                                                 'packagen00': [0, 0, 'red'], 'packagen01': [0, 1, 'yellow'],
        #                                                 'packagen02': [0, 2, 'green'], 'packagen21': [2, 1, 'red']}}}

        data = datas["package_details"]
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
                    self.send_para("Inventory", final_para)
                    time.sleep(1)
                    print(final_para)




Param = Para_grn("para")
# para.creat_pata()

