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

    def creat_inventory_pata(self, datas):

        print(type(datas))
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

    def order_update(self, arm, data):
        print(data)
        print("$$$$$$$$$$$$$$$$$$$$$$$$$$S")
        print(type(data))

        if arm == "ur5_1":
            sheetName = "OrdersDispatched"
            # final_para[]
        else:
            sheetName = "OrdersShipped"

        self.send_para(sheetName, data)

Param = Para_grn("para")
# para.creat_pata()

