#! /usr/bin/env python


import sys
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from tc5_msgs.srv import MappingPoint
from geometry_msgs.msg import Point

class Vision_Preprocess_Cli_Control(Node):

    def __init__(self):
        super().__init__('vision_prepro')
        self.pcl_client = self.create_client(MappingPoint,'vision_preprocess_service_control')

        while not self.pcl_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = MappingPoint.Request()

    def send_request(self, ptlu, ptbr, flag = 0):
        self.req.ptlu = ptlu
        self.req.ptbr = ptbr
        self.req.flag = flag

        self.future = self.pcl_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)
    
    while True:
        ptlu = Point()
        ptbr = Point()
        ptlu.x = float(-3)
        ptlu.y = float(-1)
        ptlu.z = float(1.2)
        ptbr.x = float(-3)
        ptbr.y = float(2)
        ptbr.z = float(0.3)

        flag= MappingPoint.Request.END
        # flag=MappingPoint.Request.START
        # flag=MappingPoint.Request.INITIAL

        vision_prepro_cli_control = Vision_Preprocess_Cli_Control()


        response = vision_prepro_cli_control.send_request(ptlu, ptbr, flag)
        vision_prepro_cli_control.get_logger().info(
            "Result of Preprocess for Point Cloud : capture state {}, capture done? {}, next pose {}".format(
            response.success, response.finish, response.nextpose)
            )

        if response.success == True:
            vision_prepro_cli_control.get_logger().info("successfulll!!!!!!!!!!!!!!!!!!!!")

        vision_prepro_cli_control.destroy_node()
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





