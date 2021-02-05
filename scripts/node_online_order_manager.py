#! /usr/bin/env python

'''This Client Node sends goals to appropriate server to do task '''

import rospy
import json
import rospkg
import yaml
from pkg_task5.msg import pkgLocation
from pkg_ros_iot_bridge.msg import msgMqttSub

# rp = rospkg.RosPack()
# pkg_path = rp.get_path('pkg_task5')
# file_path = pkg_path + '/config/package_priority.yaml'

Orders = {}

class order_manager(object):
    """docstring for order_manager"""
    def __init__(self, arg):
        super(order_manager, self).__init__()
        self.arg = arg
        rp = rospkg.RosPack()
        pkg_path = rp.get_path('pkg_task5')
        self.file_path = pkg_path + '/config/package_priority.yaml'
        rospy.Subscriber("/ros_iot_bridge/mqtt/sub", msgMqttSub, self.func_callback_topic_my_topic)

        self.location = rospy.Publisher("/location", pkgLocation, queue_size=0)
        self.yaml_data = ""
        self.pkg_list=[]
        self.Orders={}
        self.onceflage = True

        # self.data = _

    def func_callback_topic_my_topic(self, my_msg):
        # print(my_msg.message)
        data = json.loads(my_msg.message)
        order_id = data["order_id"]
        item = data["item"]

        self.Orders[order_id] = data
        if self.onceflage:
            self.load_yaml()
            self.onceflage = False

        package_location = pkgLocation()
        package_location.pkg_location = self.get_location(item)
        package_location.order_id = int(order_id)
        self.location.publish(package_location)

        # print(self.Orders['1001'])

    def get_location(self, items):
        data = self.yaml_data["package_details"]
        # self.pkg_list=[]
        # items = 'Food'
        for pkg in data:
            if data[pkg][3] == items :
                if pkg not in self.pkg_list:
                    self.pkg_list.append(pkg)
                    return pkg
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