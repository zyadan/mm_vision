import sys
import datetime
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import sensor_msgs.msg as sensor_msgs
from geometry_msgs.msg import Point
import cv2
from cv_bridge import CvBridge, CvBridgeError

import vision_preprocess_srv.utils as utils
import vision_preprocess_srv.point_cloud2 as pc2
from tc5_msgs.msg import DistanceMapSlice
import open3d as o3d
import numpy as np
import ros2_numpy

from std_srvs.srv import Trigger

img_bridge = CvBridge()

import open3d as o3d
import numpy as np
import os
import math
from pyntcloud import PyntCloud
from pandas import DataFrame
import scipy.linalg as linalg
from ctypes import *
import random
from scipy import ndimage

class VisionPrePro(Node):
    def __init__(self):

        super().__init__('vision_prepro')

        self.get_logger().info("Initialized..")

        self.cv_img = None
        self.ros_cloud = None

        # self.roi_path = '/home/zyadan/ros2_ws_pro/src/vision_preprocess/image_roi.yml'

       # self.read_yaml()
        # self.image_sub = self.create_subscription(Image, "/camera/color/image_raw", self.image_cb, 1)
        self.pointcloud_sub = self.create_subscription(PointCloud2, "/camera/depth/color/points", self.pcl_cb, 1)

        self.pc_srv = self.create_service(Trigger, 'vision_preprocess_service', self.pcl_service_cb)






    def read_yaml(self):
        fs = cv2.FileStorage(str(self.roi_path), cv2.FILE_STORAGE_READ)
        x = int(fs.getNode("x").real())
        y = int(fs.getNode("y").real())
        w = int(fs.getNode("width").real())
        h = int(fs.getNode("height").real())
        fs.release()
        print("roi_image: ", x,y,w,h)
        self.roi_image = [x,y,w,h]
        self.cv_img = np.zeros((self.roi_image[3], self.roi_image[2], 3), dtype=np.uint8)

    def pcl_cb(self, ros_cloud):
        self.ros_cloud = ros_cloud



    def pcl_service_cb(self, _, response : Trigger.Response):

        try:

            self.get_logger().info("get point cloud..")
            field_names=[field.name for field in self.ros_cloud.fields]
            print("field name ", field_names)
            ros_pointcloud = pc2.read_points(self.ros_cloud, skip_nans=True, field_names = field_names)
            pc_ros = ros2_numpy.numpify(self.ros_cloud)
            print("pc_ros----------------------", pc_ros[0][0])

            widthros = pc_ros.shape[0]
            heightros = pc_ros.shape[1]
            

            np_allpoints = np.zeros((heightros*widthros, 3),dtype = np.float64)
            np_allpoints[:,0] = np.resize(pc_ros['x'], heightros*widthros)
            np_allpoints[:,1] = np.resize(pc_ros['y'], heightros*widthros)
            np_allpoints[:,2] = np.resize(pc_ros['z'], heightros*widthros)
            print("np_allpoints-----------",np_allpoints[:,0])


            x_min, y_min, z_min = np.amin(np_allpoints, axis=0) 
            x_max, y_max, z_max = np.amax(np_allpoints, axis=0)

            resolution = (x_max - x_min)/ widthros 

            distance_map = pc_ros['z']
            origin_point_np = pc_ros[0][0]

            print("origin_point_np:  ",type(origin_point_np[0]))
            task_map = np.full((heightros, heightros), 0)

            # np.savetxt('/home/zyadan/ros2_ws_pro/src/vision_preprocess/distacne_map.txt', distance_map, delimiter=',')

            print("resolution:  ",resolution)

            origin_point = Point()
            origin_point.x = float(origin_point_np[0])
            origin_point.y = float(origin_point_np[1])
            origin_point.z = float(origin_point_np[2])

            drawing_origin_point = origin_point

            unknown_value = 0.0

            distance_map_flat = [float(v) for dm in distance_map for v in dm]
            # distance_map_flat = np.array(distance_map_flat, dtype=np.float32)
            task_map_flat = [bool(bv) for gb in task_map for bv in gb]
            # task_map_flat = np.array(task_map_flat, dtype=bool)
            print("distance_map_flat-----------------", type(distance_map_flat))
            # np.savetxt('/home/zyadan/ros2_ws_pro/src/vision_preprocess/distacne_map_flat.txt', distance_map_flat, delimiter=',')
   
            self.target_point = []
 

            self.target_point.append([resolution, widthros, heightros, origin_point, drawing_origin_point, unknown_value, distance_map_flat, task_map_flat])
           # self.target_point.append([resolution, widthros, heightros, origin_point ,unknown_value, grid_flat])
            # print("dofjdjfdjaljdfkljakfjdshoeohrelkf", self.target_point)

            self.construct_msg(self.target_point)
            del self.target_point[0]
            response.success = True
            print("message send.....")


        except Exception as e:
            response.success = False
            print("Exception!")
            print(e)
        

        return response


    def image_cb(self, ros2_image):

        self.get_logger().info("In Image callback function..")

        global img_bridge
        

        a = datetime.datetime.now()
        try:

            img = img_bridge.imgmsg_to_cv2(ros2_image, "bgr8")

            self.cv_img = np.ndarray(shape=(self.roi_image[3],self.roi_image[2], 3), dtype=np.uint8, buffer=img.data)

            cv2.imshow("frame", self.cv_img)
            cv2.waitKey(1)

        
        except CvBridgeError as e:
            print(e)[-0.273, 0.659, 0.032]

        cv2.waitKey(2000)
        HeightS, WidthS, ChannelS = self.cv_img.shape
        print("image information: ", HeightS, WidthS, ChannelS)


    
    
    def construct_msg(self, ilist):
        clist = []
        tc2req = DistanceMapSlice()
        tc2req.resolution = ilist[0][0]
        tc2req.width = ilist[0][1]
        tc2req.height = ilist[0][2]
        tc2req.origin = ilist[0][3]
        tc2req.drawing_origin = ilist[0][4]
        tc2req.unknown_value = ilist[0][5]
        tc2req.data = ilist[0][6]
        tc2req.task = ilist[0][7]

        # print("msgs          dddddddddddddddd", tc2req)
        clist.append(tc2req) 


def main(args=None):
    rclpy.init(args=args)
    vision_prepro = VisionPrePro()

    rclpy.spin(vision_prepro)
    vision_prepro.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()