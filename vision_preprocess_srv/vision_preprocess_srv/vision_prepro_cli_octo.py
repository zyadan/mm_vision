#! /usr/bin/env python


import sys
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from tc5_msgs.srv import OctomapSrv

class Vision_Preprocess_Client(Node):

    def __init__(self):
        super().__init__('vision_prepro')
        self.octomap_client = self.create_client(OctomapSrv,'vision_preprocess_service_taskocto')

        while not self.octomap_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = OctomapSrv.Request()

    def send_request(self):

        self.future = self.octomap_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)
    
    while True:
        vision_prepro_cli_octo = Vision_Preprocess_Client()

        response = vision_prepro_cli_octo.send_request()

        vision_prepro_cli_octo.get_logger().info(
            "Result of Preprocess for Point Cloud :  Got octomap? {}, error? {}, octomap {}".format(
            response.success, response.message, response.omap)
            )
        if response.success == True:
            print("successfulll!!!!!!!!!!!!!!!!!!!!")

        vision_prepro_cli_octo.destroy_node()
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





