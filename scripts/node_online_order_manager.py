#! /usr/bin/env python

'''This Client Node sends goals to appropriate server to do task '''

import rospy
import json
import rospkg
import yaml
import time
from datetime import datetime, timedelta
from parameter_generator import Param
from pkg_task5.msg import pkgLocation
from pkg_task5.msg import orderStatus
from pkg_ros_iot_bridge.msg import msgMqttSub

class order_manager(object):
    """docstring for order_manager"""
    def __init__(self, arg):
        super(order_manager, self).__init__()
        self.arg = arg
        rp = rospkg.RosPack()
        pkg_path = rp.get_path('pkg_task5')
        self.file_path = pkg_path + '/config/package_priority.yaml'
        rospy.Subscriber("/ros_iot_bridge/mqtt/sub", msgMqttSub, self.func_callback_online_order)
        rospy.Subscriber("/orderstatus", orderStatus, self.func_callback_order_status)

        self.location = rospy.Publisher("/location", pkgLocation, queue_size=0)
        self.yaml_data = ""
        self.pkg_list = []
        self.savedOrdersForDispatch = {}
        self.savedOrdersForShipping = {}
        self.dispatOrder = []
        self.onceflage = True


    def func_callback_online_order(self, my_msg):
        print(my_msg.message)
        incomingOrder = my_msg.message
        #incomingOrder is perfect srring
        # print(type(my_msg.message))

        data = json.loads(my_msg.message)
        # print(data)
        # print(type(data))

        # hii = json.dumps(data)
        # print(hii)
        # print(type(hii))


        order_id = data["order_id"]
        item = data["item"]
        # order_time = datetime.now().replace(microsecond=0)
        # data["order_time"] = order_time
        self.savedOrdersForDispatch[order_id] = incomingOrder
        # dict of allorder and string order details

        if self.onceflage:
            self.load_yaml()
            self.onceflage = False

        package_location = pkgLocation()
        package_location.pkg_location, fullpkgName = self.get_location(item, order_id)
        package_location.order_id = str(order_id)
        self.location.publish(package_location)
        self.savedOrdersForShipping[fullpkgName] = incomingOrder

        # print(self.savedOrdersForDispatch['1001'])

    def func_callback_order_status(self, orderstatus):

        # timeString = {}
        upadteFromArm = orderstatus.arm
        order_identity = orderstatus.orderId

        # updateDateTime = str(time.strftime("%D %H:%m:%S"))
        # timeString = ', "updateDateTime":"%s"'%(updateDateTime)

        timeString = ', "updateDateTime":"%s"'%(self.get_time_str())

        if upadteFromArm == "ur5_1":
            self.dispatOrder.append(order_identity)
            order_details = self.savedOrdersForDispatch[order_identity]
            index = order_details.find('}')
            final_order_details = order_details[:index]+timeString+order_details[index:]
        else:
            order_details = self.savedOrdersForShipping[order_identity]
            jsonDict = json.loads(order_details)
            print("!!!!!!!!!!!!!!!!!!!!!!!!!!")
            print(jsonDict)
            print("!!!!!!!!!!!!!!!!!!!!!!!!!!!1")
            shipped_item = jsonDict["item"]

            estimTime = 1 if shipped_item == "Medicine" else (
                        3 if shipped_item == "Food" else 5)
            
            estimatedTime=(datetime.now()+timedelta(days = estimTime)).strftime("%Y-%m-%d")
            index = order_details.find('}')
            addEstimTimeToData = timeString+', "estimTime":"%s"'%(estimatedTime)
            final_order_details = order_details[:index]+addEstimTimeToData+order_details[index:]

        #order_details is string conta perfect order

        # orderInStr = json.dumps(order_details)


        
        # print(type(updateDateTime))
        # order_details["updateDateTime"] = updateDateTime

        

        # jsonDict = json.loads(timeString)
        # print(jsonDict)
        # order_details.update(jsonDict)
        # print(order_details)

        # final_order_details = json.loads(order_details)
        print(final_order_details)
        print(type(final_order_details))
        # final_order_details.replace("'",'"')
        # print(final_order_details)
        Param.order_update(upadteFromArm,final_order_details)

    
    def get_time_str(self):
        timestamp = int(time.time())
        value = datetime.fromtimestamp(timestamp)
        str_time = value.strftime('%Y-%m-%d %H:%M:%S')

        return str_time


    def get_location(self, items, order_id):
        data = self.yaml_data["package_details"]
        # self.pkg_list=[]
        # items = 'Food'
        for pkg in data:
            if data[pkg][3] == items :
                if pkg not in self.pkg_list:
                    self.pkg_list.append(pkg)
                    return items[0]+pkg, pkg
                    break
        
    def load_yaml(self):
        try:
            with open(self.file_path) as file:
                self.yaml_data = yaml.load(file)
                # print(self.yaml_data)
                # print(type(self.yaml_data))
        except Exception as e:
            raise e
        finally:
            if file is not None:
                file.close()

if __name__ == '__main__':

    try:
        # 1. Initialize ROS Node
        rospy.init_node('node_online_order_manager')
        Manager = order_manager("object")
        # 4. Initialize Subscriber
        
        # try:
        #     with open(file_path) as file:
        #         yaml_data = yaml.load(file)#, Loader=yaml.FullLoader)
        #         print(yaml_data)
        # except Exception as e:
        #     raise e
        

        rospy.spin()
        
    except rospy.ROSInterruptException:
        print 'Something went wrong:'