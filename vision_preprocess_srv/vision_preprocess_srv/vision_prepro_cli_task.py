#! /usr/bin/env python


import sys
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from task_msgs.msg import DistanceMapMsg
from task_msgs.srv import DistanceMapSrv

class Vision_Preprocess_Client(Node):

    def __init__(self):
        super().__init__('vision_prepro')
        self.pcl_client = self.create_client(DistanceMapSrv,'vision_preprocess_service_task')

        while not self.pcl_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = DistanceMapSrv.Request()

    def send_request(self):

        self.future = self.pcl_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)
    
    while True:
        vision_prepro_cli_task = Vision_Preprocess_Client()

        response = vision_prepro_cli_task.send_request()

        vision_prepro_cli_task.get_logger().info(
            "Result of Preprocess for Point Cloud :  Got distance map? {}, error? {}, Distance map ".format(
            response.success, response.message)
            )
        if response.success == True:
            print("successfulll!!!!!!!!!!!!!!!!!!!!")

        vision_prepro_cli_task.destroy_node()
        rclpy.shutdown()


    # vision_prepro_cli = Vision_Preprocess_Client()

    # response = vision_prepro_cli.send_request()
    # vision_prepro_cli.get_logger().info(
    #     "Result of Preprocess for Point Cloud"
    # )
    # # if response.success == True:
    # #     print("successfulll!!!!!!!!!!!!!!!!!!!!")

    # vision_prepro_cli.destroy_node()
    # rclpy.shutdown()



if __name__ =='__main__':
    main()





