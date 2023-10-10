import sys
import datetime
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import sensor_msgs.msg as sensor_msgs
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
import cv2
from cv_bridge import CvBridge, CvBridgeError

import vision_preprocess_srv.utils as utils
import vision_preprocess_srv.point_cloud2 as pc2
from tc5_msgs.msg import DistanceMapSlice
from tc5_msgs.srv import MappingPoint



import open3d as o3d
import numpy as np
import ros2_numpy

from std_srvs.srv import Trigger
from task_msgs.msg import DistanceMapMsg
from task_msgs.srv import DistanceMapSrv
from std_msgs.msg import Header
from rclpy.clock import Clock


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
import pyransac3d as pyrsc

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf_transformations
import quaternion
import math




class VisionPrePro(Node):
    def __init__(self):

        super().__init__('vision_prepro_srv')

        self.get_logger().info("Initialized..")

        self.cv_img = None
        self.ros_cloud = None
        self.iters = 0
        self.Point_cloud_paint = None
        self.Point_cloud_paint_list = list()
        # self.Point_cloud_paint_np = np.zeros((3,3))
        self.Point_cloud_paint_np = None
        self.camera2worldtf_origin = None
        self.multi_capture = False
        self.camera_poselist = list()
        self.dis2wall = 1.2
        self.threshold = 6
        self.canvas_origin_inc = Point()
        self.canvas_rd_inc = Point()

        

        # self.roi_path = '/home/zyadan/ros2_ws_pro/src/vision_preprocess/image_roi.yml'

        self.source_frame = self.declare_parameter('source_frame', 'camera_link').get_parameter_value().string_value        
        self.target_frame = self.declare_parameter('target_frame', 'map').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

       # self.read_yaml()
        # self.image_sub = self.create_subscription(Image, "/camera/color/image_raw", self.image_cb, 1)
        self.pointcloud_sub = self.create_subscription(PointCloud2, "/camera/depth/color/points", self.pcl_cb, 1)
        self.pcd_pub = self.create_publisher(PointCloud2, "/construct_pcd", 1)

        self.mapping_srv = self.create_service(MappingPoint, 'vision_preprocess_service_control', self.mapping_service_cb)
        # self.pc_srv = self.create_service(Trigger, 'vision_preprocess_service', self.distance_map_cb)
        self.pc_srv = self.create_service(DistanceMapSrv, 'vision_preprocess_service_task', self.distance_map_cb)




    def robot_way_point(self, PtLU, PtRD):
        
        PtLU_np = np.matrix([PtLU.x, PtLU.y, PtLU.z], dtype='float32')
        PtLU_np.resize((3,1))
        PtRD_np = np.matrix([PtRD.x, PtRD.y, PtRD.z], dtype='float32')
        PtRD_np.resize((3,1))

        Hfov = 80.0/180 * math.pi #radian
        Vfov = 58.0/180 * math.pi #radian
        # Hfov = 67.0/180 * math.pi #radian
        # Vfov = 50.0/180 * math.pi #radian

        dilation = 0.1
        overlap = 0.3

        Psizeinwall = math.tan(Hfov/2) * self.dis2wall * 2
        Vsizeinwall = math.tan(Vfov/2) * self.dis2wall * 2
        self.get_logger().info("Psizeinwall {}, {}".format(Psizeinwall, math.tan(Hfov/2)))

        #construct wall frame
        V_vertical = np.array([0,0,-1])
        V_2point = np.array([PtRD.x - PtLU.x, PtRD.y - PtLU.y, PtRD.z - PtLU.z])
        V_wallz = np.cross(V_2point, V_vertical)
        V_wallx = np.cross(V_vertical, V_wallz)
        rotate_matrix_G2wall, rotate_matrix_wall2G = utils.rotate_matrix_from_direccos(V_wallx, V_vertical,V_wallz)

        PtRD_inwall = np.dot(rotate_matrix_wall2G, PtRD_np)
        PtLU_inwall = np.dot(rotate_matrix_wall2G, PtLU_np)

        Pwall = PtRD_inwall[0] - PtLU_inwall[0]
        Vwall = PtRD_inwall[1] - PtLU_inwall[1]

        num_p = int((Pwall +2*dilation -overlap)//(Psizeinwall-overlap)+1)  # num_p * Psizeinwall-(num_p-1)*overlap-2*dilation>Pwall
        num_v = int((Vwall +2*dilation -overlap)//(Vsizeinwall-overlap)+1)

        new_dilation_P = ((Psizeinwall-overlap)*num_p + overlap - Pwall) / 2
        new_dilation_V = ((Vsizeinwall-overlap)*num_v + overlap - Vwall) / 2

        camera_position_np = np.zeros((num_v, num_p, 3),dtype = np.float64)

        # for i in range(num_v):
        #     for j in range(num_p):

        #         camera_waypoint_np_x = (j+0.5) *Psizeinwall - j*overlap - new_dilation_P
        #         camera_waypoint_np_y = (i+0.5) *Vsizeinwall - i*overlap - new_dilation_V
        #         camera_waypoint_np_z = -1.8
                
        #         camera_position_np[i, j, 0] = camera_waypoint_np_x
        #         camera_position_np[i, j, 1] = camera_waypoint_np_y
        #         camera_position_np[i, j, 2] = camera_waypoint_np_z

        #         camera_waypoint_np = np.matrix([camera_waypoint_np_x,camera_waypoint_np_y, camera_waypoint_np_z], dtype='float32')
        #         camera_waypoint_np.resize((3,1))
        #         camera_waypoint_world = np.dot(rotate_matrix_G2wall, camera_waypoint_np) + PtLU_np

        #         # self.get_logger().info("camera_pose.position {}".format(rotate_matrix_G2wall))

        #         camera_pose = Pose()
        #         camera_pose.position.x = float(camera_waypoint_world[0])
        #         camera_pose.position.y = float(camera_waypoint_world[1])
        #         camera_pose.position.z = float(camera_waypoint_world[2])

        #         orientation = quaternion.from_rotation_matrix(rotate_matrix_G2wall)
        #         # camera_pose.orientation.x = float(orientation.x)
        #         # camera_pose.orientation.y = float(orientation.y) 
        #         # camera_pose.orientation.z = float(orientation.z) 
        #         # camera_pose.orientation.w = float(orientation.w) 
        #         camera_pose.orientation.x = float(0)
        #         camera_pose.orientation.y = float(0) 
        #         camera_pose.orientation.z = float(0) 
        #         camera_pose.orientation.w = float(1)
        #         self.get_logger().info("camera_pose.position {}, {}, {}".format(camera_pose.position.x, camera_pose.position.y, camera_pose.position.z))

        #         self.camera_poselist.append(camera_pose)
        camera_pose = Pose()
        camera_pose.position.x = float(-1.0)
        camera_pose.position.y = float(-1.0)
        camera_pose.position.z = float(1.2)


        camera_pose.orientation.x = float(0)
        camera_pose.orientation.y = float(0) 
        camera_pose.orientation.z = float(0) 
        camera_pose.orientation.w = float(1) 
        self.get_logger().info("camera_pose.position {}, {}, {}".format(camera_pose.position.x, camera_pose.position.y, camera_pose.position.z))

        self.camera_poselist.append(camera_pose)

    

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


    def mapping_service_cb(self, request, response):

        if request.flag == MappingPoint.Request.INITIAL:
            self.robot_way_point(request.ptlu, request.ptbr)
            self.get_logger().info("get robot_way_point {} {}".format(self.camera_poselist, len(self.camera_poselist)))
            response.success = False
            self.canvas_origin_inc = request.ptlu
            self.canvas_rd_inc = request.ptbr

            if len(self.camera_poselist)>1:
                self.multi_capture = True

            response.nextpose = self.camera_poselist[0]
            self.camera_poselist.pop(0)

            return response
        else:
            if request.flag == MappingPoint.Request.START:
                first_capture = True
            else:
                first_capture = False
            self.get_logger().info("request.flag {}".format(request.flag))
            self.get_logger().info("first_capture {}".format(first_capture))
            
            response.success = self.construct_map(self.multi_capture, first_capture)

            if len(self.camera_poselist) >0:
                response.finish = False
                response.nextpose = self.camera_poselist[0]
                self.camera_poselist.pop(0)
            else:
                response.finish = True
                self.get_logger().info("Point Cloud Capturing Complete!!")


            return response
        
    def mapping_service_cb_backup(self, request, response):

        if request.flag == MappingPoint.Request.INITIAL:
            self.robot_way_point(request.ptlu, request.ptbr)
            self.get_logger().info("get robot_way_point {} {}".format(self.camera_poselist, len(self.camera_poselist)))
            response.success = False
            self.canvas_origin_inc = request.ptlu
            self.canvas_rd_inc = request.ptbr

            response.nextpose = self.camera_poselist[0]
            self.camera_poselist.pop(0)

            return response
        else:

            self.get_logger().info("request.flag {}".format(request.flag))
            response.success = self.construct_map()

            if len(self.camera_poselist) >0:
                response.finish = False
                response.nextpose = self.camera_poselist[0]
                self.camera_poselist.pop(0)
            else:
                response.finish = True

            return response

    def construct_map_backup(self):
        try:
          
            self.get_logger().info("get point cloud..")
            field_names=[field.name for field in self.ros_cloud.fields]
            print("field name ", field_names)
            ros_pointcloud = list(pc2.read_points(self.ros_cloud, skip_nans=True, field_names = field_names))
            pc_ros = ros2_numpy.numpify(self.ros_cloud)
            print("pc_ros----------------------", pc_ros.shape[0])

            o3d_cloud = utils.convertCloudFromRosToOpen3d(ros_pointcloud,field_names)  # convert to open3d

            o3d.visualization.draw_geometries([o3d_cloud])
            np_cloud = np.asarray(o3d_cloud.points)

            mask = np_cloud[:, 2] <= self.threshold+0.2
            np_cloud = np_cloud[mask]

            from_frame_rel = self.source_frame
            to_frame_rel = self.target_frame
            self.get_logger().info("get frame transformation {}, to {}".format(from_frame_rel, to_frame_rel))
            
            camera2worldtf = self.tf_buffer.lookup_transform(
                        "map", 
                        "camera_depth_optical_frame",
                        rclpy.time.Time()).transform
            
            camera2worldtf_q = np.quaternion(camera2worldtf.rotation.w, camera2worldtf.rotation.x, camera2worldtf.rotation.y, camera2worldtf.rotation.z)
            camera2worldtf_t =  np.matrix([camera2worldtf.translation.x, camera2worldtf.translation.y, camera2worldtf.translation.z],
                                            dtype='float32')

            np_cloud = np_cloud.transpose()       
            camera2worldtf_tM = np.tile(camera2worldtf_t.transpose(), np_cloud.shape[1])
            self.get_logger().info("camera2worldtf_tM size {}".format(camera2worldtf_tM))
            
            rotated_point = np.dot( quaternion.as_rotation_matrix(camera2worldtf_q) , np_cloud)
            Point_cloud_paint_np  = rotated_point + camera2worldtf_tM


            self.Point_cloud_paint_np = np.concatenate((self.Point_cloud_paint_np,Point_cloud_paint_np),axis=1)
            self.Point_cloud_paint_np = np.array(self.Point_cloud_paint_np)
    

            self.Point_cloud_paint_np = np.delete(self.Point_cloud_paint_np, [0, 1,2], axis=1)
            Pttemp = o3d.geometry.PointCloud()
            Pttemp.points = o3d.utility.Vector3dVector(np.array(self.Point_cloud_paint_np.transpose()))
            self.Point_cloud_paint = Pttemp

            # o3d.io.write_point_cloud("/home/zyadan/ros2_ws_pro/src/3.pcd", Pttemp)

            # o3d.visualization.draw_geometries([Pttemp])
            success = True

         

        except Exception as e:
            success = False
            print("Exception!")
            print(e)
        

        return success
        

    def construct_map(self, multi_capture=True, first_capture=True):
        try:
          
            self.get_logger().info("get point cloud..")
            field_names=[field.name for field in self.ros_cloud.fields]
            print("field name ", field_names)
            ros_pointcloud = list(pc2.read_points(self.ros_cloud, skip_nans=True, field_names = field_names))
            pc_ros = ros2_numpy.numpify(self.ros_cloud)
            print("pc_ros----------------------", pc_ros.shape[0])

            o3d_cloud = utils.convertCloudFromRosToOpen3d(ros_pointcloud,field_names)  # convert to open3d
            np_cloud = np.asarray(o3d_cloud.points)

            mask = np_cloud[:, 2] <= self.threshold+0.2
            np_cloud = np_cloud[mask]


            # o3d.visualization.draw_geometries([o3d_cloud])

         
            if multi_capture == True:

                from_frame_rel = self.source_frame
                to_frame_rel = self.target_frame
                self.get_logger().info("get frame transformation {}, to {}".format(from_frame_rel, to_frame_rel))
                # self.get_logger().info("iiiiiiiiiiii {}".format(i))
                
                if first_capture==True:
                    camera2worldtf = self.tf_buffer.lookup_transform(
                        "map", 
                        "camera_depth_optical_frame",
                        rclpy.time.Time()).transform
            
                    camera2worldtf_q = np.quaternion(camera2worldtf.rotation.w, camera2worldtf.rotation.x, camera2worldtf.rotation.y, camera2worldtf.rotation.z)
                    camera2worldtf_t =  np.matrix([camera2worldtf.translation.x, camera2worldtf.translation.y, camera2worldtf.translation.z],
                                                    dtype='float32')

                    np_cloud = np_cloud.transpose()       
                    camera2worldtf_tM = np.tile(camera2worldtf_t.transpose(), np_cloud.shape[1])
                    self.get_logger().info("camera2worldtf_tM size {}".format(camera2worldtf_tM))
                    
                    rotated_point = np.dot( quaternion.as_rotation_matrix(camera2worldtf_q) , np_cloud)
                    Point_cloud_paint_np  = rotated_point + camera2worldtf_tM

                    self.Point_cloud_paint_np = Point_cloud_paint_np
                    
                    Pttemp = o3d.geometry.PointCloud()
                    Pttemp.points = o3d.utility.Vector3dVector(np.array(self.Point_cloud_paint_np.transpose()))
                    # self.Point_cloud_paint = Pttemp

                    planes = []
                    wall_normal = []
                    plane_num = 0
                    wall_pcd = o3d.geometry.PointCloud()
                    self.get_logger().info("hereA11111111!!!!!!!!!!!!!!!")
                    while len(Pttemp.points) > 20000 and plane_num<6:

                        self.get_logger().info("hereA12222!!!!!!!!!!!!!!!")

                        plane_model, inliers = Pttemp.segment_plane(distance_threshold=0.005, ransac_n=3, num_iterations=200)
                        [a, b, c, d] = plane_model
                        # print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
                        
                        inlier_cloud = Pttemp.select_by_index(inliers)
                        outlier_cloud = Pttemp.select_by_index(inliers, invert=True)
                        planes.append(inlier_cloud)
                        wall_normal.append(plane_model[:3])
                        plane_num +=1
                        
                        # for left 
                        Pttemp = outlier_cloud
                    self.get_logger().info("hereA1333333333333!!!!!!!!!!!!!!!")
                    angle_threshold = np.pi / 3 
                    vertical_direction = np.array([0, 0, 1])

                    for i in range(len(planes)):
                        cos_theta = np.abs(np.dot(wall_normal[i], vertical_direction))

                        if cos_theta < np.cos(angle_threshold):
                            wall_pcd_np = np.concatenate([np.asarray(wall_pcd.points), np.asarray(planes[i].points)], axis=0)
                            wall_pcd.points = o3d.utility.Vector3dVector(wall_pcd_np)
                            leaf_size = 0.01
                            point_cloud_o3d_filter = o3d.geometry.PointCloud() 
                            point_cloud_o3d_filterori = o3d.geometry.PointCloud() 
                            origin_point, distance_map, _= utils.point2voxel(np.array(wall_pcd.points), leaf_size)
                            height = distance_map.shape[0]
                            width =distance_map.shape[1]
                            grid_points_out = []
                            grid_points_ori = []
                            for i in range(height):
                                for j in range(width):

                                    if distance_map[i,j] ==0:
                                        point_temp = [0, 0, 0]
                                        point_temp[0] = np.sum(distance_map[:,j])/np.count_nonzero(distance_map[:,j])
                                        point_temp[1] = origin_point.y + j * leaf_size 
                                        point_temp[2] = origin_point.z + i * leaf_size
                                        grid_points_out.append(point_temp)
                                    else:
                                        point_temp = [0, 0, 0]
                                        point_temp[0] = distance_map[i,j]
                                        point_temp[1] = origin_point.y + j * leaf_size 
                                        point_temp[2] = origin_point.z + i * leaf_size
                                        grid_points_ori.append(point_temp)

                            filteredgrid_pointsori = np.array(grid_points_ori, dtype=np.float64)
                            filteredgrid_points = np.array(grid_points_out, dtype=np.float64)
                            point_cloud_o3d_filter.points = o3d.utility.Vector3dVector(filteredgrid_points)
                            point_cloud_o3d_filterori.points = o3d.utility.Vector3dVector(filteredgrid_pointsori)
                            wall_pcd_1 = point_cloud_o3d_filterori + point_cloud_o3d_filter

                    wall_pcd_filtered, _ = wall_pcd_1.remove_statistical_outlier(nb_neighbors=20, std_ratio=4.0)
                    self.Point_cloud_paint = wall_pcd_filtered
                    # o3d.visualization.draw_geometries([wall_pcd])
                    # o3d.visualization.draw_geometries([wall_pcd_filtered])
                    success = True
                else:
                    camera2worldtf = self.tf_buffer.lookup_transform(
                        "map", 
                        "camera_depth_optical_frame",
                        rclpy.time.Time()).transform
            
                    camera2worldtf_q = np.quaternion(camera2worldtf.rotation.w, camera2worldtf.rotation.x, camera2worldtf.rotation.y, camera2worldtf.rotation.z)
                    camera2worldtf_t =  np.matrix([camera2worldtf.translation.x, camera2worldtf.translation.y, camera2worldtf.translation.z],
                                                    dtype='float32')

                    np_cloud = np_cloud.transpose()       
                    camera2worldtf_tM = np.tile(camera2worldtf_t.transpose(), np_cloud.shape[1])
                    self.get_logger().info("camera2worldtf_tM size {}".format(camera2worldtf_tM))
                    
                    rotated_point = np.dot( quaternion.as_rotation_matrix(camera2worldtf_q) , np_cloud)
                    Point_cloud_paint_np  = rotated_point + camera2worldtf_tM

                    Pttemp = o3d.geometry.PointCloud()
                    Pttemp.points = o3d.utility.Vector3dVector(np.array(Point_cloud_paint_np.transpose()))


                    planes = []
                    wall_normal = []
                    plane_num = 0
                    wall_pcd = o3d.geometry.PointCloud()
                    self.get_logger().info("hereA11111111!!!!!!!!!!!!!!!")
                    while len(Pttemp.points) > 20000 and plane_num<6:

                        self.get_logger().info("hereA12222!!!!!!!!!!!!!!!")

                        plane_model, inliers = Pttemp.segment_plane(distance_threshold=0.005, ransac_n=3, num_iterations=200)
                        [a, b, c, d] = plane_model
                        # print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
                        
                        inlier_cloud = Pttemp.select_by_index(inliers)
                        outlier_cloud = Pttemp.select_by_index(inliers, invert=True)
                        planes.append(inlier_cloud)
                        wall_normal.append(plane_model[:3])
                        plane_num +=1
                        
                        # for left 
                        Pttemp = outlier_cloud
                    self.get_logger().info("hereA1333333333333!!!!!!!!!!!!!!!")
                    angle_threshold = np.pi / 3 
                    vertical_direction = np.array([0, 0, 1])

                    for i in range(len(planes)):
                        cos_theta = np.abs(np.dot(wall_normal[i], vertical_direction))

                        if cos_theta < np.cos(angle_threshold):
                            wall_pcd_np = np.concatenate([np.asarray(wall_pcd.points), np.asarray(planes[i].points)], axis=0)
                            wall_pcd.points = o3d.utility.Vector3dVector(wall_pcd_np)

                            leaf_size = 0.01
                            point_cloud_o3d_filter = o3d.geometry.PointCloud() 
                            point_cloud_o3d_filterori = o3d.geometry.PointCloud() 
                            origin_point, distance_map, _= utils.point2voxel(np.array(wall_pcd.points), leaf_size)
                            height = distance_map.shape[0]
                            width =distance_map.shape[1]
                            grid_points_out = []
                            grid_points_ori = []
                            for i in range(height):
                                for j in range(width):

                                    if distance_map[i,j] ==0:
                                        point_temp = [0, 0, 0]
                                        point_temp[0] = np.sum(distance_map[:,j])/np.count_nonzero(distance_map[:,j])
                                        point_temp[1] = origin_point.y + j * leaf_size 
                                        point_temp[2] = origin_point.z + i * leaf_size
                                        grid_points_out.append(point_temp)
                                    else:
                                        point_temp = [0, 0, 0]
                                        point_temp[0] = distance_map[i,j]
                                        point_temp[1] = origin_point.y + j * leaf_size 
                                        point_temp[2] = origin_point.z + i * leaf_size
                                        grid_points_ori.append(point_temp)

                            filteredgrid_pointsori = np.array(grid_points_ori, dtype=np.float64)
                            filteredgrid_points = np.array(grid_points_out, dtype=np.float64)
                            point_cloud_o3d_filter.points = o3d.utility.Vector3dVector(filteredgrid_points)
                            point_cloud_o3d_filterori.points = o3d.utility.Vector3dVector(filteredgrid_pointsori)
                            wall_pcd_1 = point_cloud_o3d_filterori + point_cloud_o3d_filter

                    wall_pcd_filtered, _ = wall_pcd_1.remove_statistical_outlier(nb_neighbors=20, std_ratio=4.0)
                    # o3d.visualization.draw_geometries([wall_pcd])
                    # o3d.visualization.draw_geometries([wall_pcd_filtered])


                    # threshold = 0.02
                    # trans_init = np.asarray([[1,0,0,0],
                    #                         [0,1,0,0],
                    #                         [0,0,1,0],
                    #                         [0,0,0,1]])
                    icp_output = o3d.pipelines.registration.registration_icp(wall_pcd_filtered, self.Point_cloud_paint, max_correspondence_distance=0.05)

                    # print(icp_output.transformation)

                    wall_pcd_filtered.transform(icp_output.transformation)
                    # o3d.visualization.draw_geometries([wall_pcd_filtered, self.Point_cloud_paint])

                    Point_cloud_paint_np = np.concatenate([np.asarray(self.Point_cloud_paint.points), np.asarray(wall_pcd_filtered.points)], axis=0)
                    
                    Pttemp1 = o3d.geometry.PointCloud()
                    Pttemp1.points = o3d.utility.Vector3dVector(np.array(Point_cloud_paint_np))
                    self.Point_cloud_paint = Pttemp1

                    # o3d.io.write_point_cloud("/home/zyadan/ros2_ws_pro/src/3.pcd", Pttemp1)

                    # o3d.visualization.draw_geometries([Pttemp1])
                    success = True

            else:

                camera2worldtf = self.tf_buffer.lookup_transform(
                    "map", 
                    "camera_depth_optical_frame",
                    rclpy.time.Time()).transform
        
                camera2worldtf_q = np.quaternion(camera2worldtf.rotation.w, camera2worldtf.rotation.x, camera2worldtf.rotation.y, camera2worldtf.rotation.z)
                camera2worldtf_t =  np.matrix([camera2worldtf.translation.x, camera2worldtf.translation.y, camera2worldtf.translation.z],
                                                dtype='float32')

                np_cloud = np_cloud.transpose()       
                camera2worldtf_tM = np.tile(camera2worldtf_t.transpose(), np_cloud.shape[1])
                self.get_logger().info("camera2worldtf_tM size {}".format(camera2worldtf_tM))
                
                rotated_point = np.dot( quaternion.as_rotation_matrix(camera2worldtf_q) , np_cloud)
                Point_cloud_paint_np  = rotated_point + camera2worldtf_tM

                self.Point_cloud_paint_np = Point_cloud_paint_np
                
                Pttemp = o3d.geometry.PointCloud()
                Pttemp.points = o3d.utility.Vector3dVector(np.array(self.Point_cloud_paint_np.transpose()))
                # self.Point_cloud_paint = Pttemp

                planes = []
                wall_normal = []
                plane_num = 0
                wall_pcd = o3d.geometry.PointCloud()
                self.get_logger().info("hereA11111111!!!!!!!!!!!!!!!")
                while len(Pttemp.points) > 20000 and plane_num<6:

                    self.get_logger().info("hereA12222!!!!!!!!!!!!!!!")

                    plane_model, inliers = Pttemp.segment_plane(distance_threshold=0.005, ransac_n=3, num_iterations=200)
                    [a, b, c, d] = plane_model
                    # print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
                    
                    inlier_cloud = Pttemp.select_by_index(inliers)
                    outlier_cloud = Pttemp.select_by_index(inliers, invert=True)
                    planes.append(inlier_cloud)
                    wall_normal.append(plane_model[:3])
                    plane_num +=1
                    
                    # for left 
                    Pttemp = outlier_cloud
                self.get_logger().info("hereA1333333333333!!!!!!!!!!!!!!!")
                angle_threshold = np.pi / 3 
                vertical_direction = np.array([0, 0, 1])

                for i in range(len(planes)):
                    cos_theta = np.abs(np.dot(wall_normal[i], vertical_direction))

                    if cos_theta < np.cos(angle_threshold):
                        wall_pcd_np = np.concatenate([np.asarray(wall_pcd.points), np.asarray(planes[i].points)], axis=0)
                        wall_pcd.points = o3d.utility.Vector3dVector(wall_pcd_np)
                        leaf_size = 0.01
                        point_cloud_o3d_filter = o3d.geometry.PointCloud() 
                        point_cloud_o3d_filterori = o3d.geometry.PointCloud() 
                        origin_point, distance_map, _= utils.point2voxel(np.array(wall_pcd.points), leaf_size)
                        height = distance_map.shape[0]
                        width =distance_map.shape[1]
                        grid_points_out = []
                        grid_points_ori = []
                        for i in range(height):
                            for j in range(width):

                                if distance_map[i,j] ==0:
                                    point_temp = [0, 0, 0]
                                    point_temp[0] = np.sum(distance_map[:,j])/np.count_nonzero(distance_map[:,j])
                                    point_temp[1] = origin_point.y + j * leaf_size 
                                    point_temp[2] = origin_point.z + i * leaf_size
                                    grid_points_out.append(point_temp)
                                else:
                                    point_temp = [0, 0, 0]
                                    point_temp[0] = distance_map[i,j]
                                    point_temp[1] = origin_point.y + j * leaf_size 
                                    point_temp[2] = origin_point.z + i * leaf_size
                                    grid_points_ori.append(point_temp)

                        filteredgrid_pointsori = np.array(grid_points_ori, dtype=np.float64)
                        filteredgrid_points = np.array(grid_points_out, dtype=np.float64)
                        point_cloud_o3d_filter.points = o3d.utility.Vector3dVector(filteredgrid_points)
                        point_cloud_o3d_filterori.points = o3d.utility.Vector3dVector(filteredgrid_pointsori)
                        wall_pcd_1 = point_cloud_o3d_filterori + point_cloud_o3d_filter

                wall_pcd_filtered, _ = wall_pcd_1.remove_statistical_outlier(nb_neighbors=20, std_ratio=4.0)
                self.Point_cloud_paint = wall_pcd_filtered
                # o3d.visualization.draw_geometries([self.Point_cloud_paint])
                success = True



        except Exception as e:
            success = False
            print("Exception!")
            print(e)
        

        return success
        
        
    
 
        

    def distance_map_cb(self, _, response):
        self.get_logger().info("In to distance_map_cb.")
        try:

            #Get the plane 

            # o3d.visualization.draw_geometries([self.Point_cloud_paint])
            # Observepcd = o3d.io.read_point_cloud("/home/zyadan/ros2_ws_pro/src/pcdori.pcd")


            # [a, b, c, d] = plane_model
            # print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

            Observepcd = self.Point_cloud_paint

            o3d.visualization.draw_geometries([Observepcd])

            # o3d.io.write_point_cloud("/home/ros/robot_ws/src/Mobile_Manipulator/mm_vision/distance_map.ply", Observepcd)

            pc_msg = utils.open3d_to_ros(Observepcd, frame_id="map")

            self.pcd_pub.publish(pc_msg)

            leaf_size = 0.02

            resolution = leaf_size

            origin_point, distance_map, grid_points_out = utils.point2distancemap(np.array(Observepcd.points), leaf_size)


            # grid_points_pt = o3d.geometry.PointCloud()
            # grid_points_pt.points = o3d.utility.Vector3dVector(np.array(grid_points_out))``
            # o3d.visualization.draw_geometries([Observepcd])
 

            # from mpl_toolkits.mplot3d import Axes3D
            # import matplotlib.pyplot as plt
            # fig = plt.figure()
            # ax = fig.add_subplot(111, projection='3d')
            # x, y, z = zip(*grid_points_out)
            # ax.scatter(x, y, z)
            # plt.show()
       
            height = distance_map.shape[0]
            width =distance_map.shape[1]
            print("resolution:  ",height,width)

            # print("origin_point_np:  ",type(origin_point_np[0]))
            # task_map = np.full((height, height), 0)


            print("resolution:  ",resolution)

            # origin_point = Point()
            # # origin_point.x = float(-1.0)
            # # origin_point.y = float(-2.0)
            # # origin_point.z = float(1.4)
            # origin_point.x = float(0.0)
            # origin_point.y = float(-1.5)
            # origin_point.z = float(1.4)




            # canvas_origin = Point()
            # canvas_origin.x = float((self.canvas_origin_inc.y - origin_point.y) // leaf_size)
            # canvas_origin.y = float((origin_point.z - self.canvas_origin_inc.z) // leaf_size)
            # canvas_origin.z = float(0)
            # canvas_width = int(abs((self.canvas_rd_inc.y - self.canvas_origin_inc.y ))// leaf_size +1)
            # canvas_height = int(abs((self.canvas_origin_inc.z - self.canvas_rd_inc.z )) // leaf_size +1)
            canvas_origin = Point()
            canvas_origin.x = float(10.0)
            canvas_origin.y = float(10.0)
            canvas_origin.z = float(0)
            canvas_width = int(width-14)
            canvas_height = int(height-14)
            self.get_logger().info("dis origin, {},".format(origin_point))
            self.get_logger().info("canvas_origin, {}, {}, {}".format(canvas_origin, canvas_width, canvas_height))
            self.get_logger().info("distance map size, {}, {}".format(height, width))

            # distance_map_op = distance_map[canvas_origin.x:canvas_width, canvas_origin.y:canvas_height]

            unknown_value = 0.0

            distance_map_flat = [float(v) for dm in distance_map for v in dm]
            # distance_map_flat = np.array(distance_map_flat, dtype=np.float32)
            # task_map_flat = [bool(bv) for gb in task_map for bv in gb]
            # task_map_flat = np.array(task_map_flat, dtype=bool)
   
            self.target_point = []
            dismapheader = Header()
            dismapheader.frame_id = self.target_frame
 
            # np.savetxt('/home/gary/mm_ws/src/Mobile_Manipulator/mm_vision/origin_point.txt', origin_point, delimiter=',')
            # np.savetxt('/home/gary/mm_ws/src/Mobile_Manipulator/mm_vision/canvas_origin.txt', canvas_origin, delimiter=',')
            # np.savetxt('/home/ros/robot_ws/src/Mobile_Manipulator/mm_vision/distance_map.txt', np.array(distance_map), delimiter=',')
            # np.savetxt('/home/ros/robot_ws/src/Mobile_Manipulator/mm_vision/distance_map_flat.txt', np.array(distance_map_flat), delimiter=',')


            self.target_point.append([dismapheader, resolution, width, height, origin_point, canvas_width, canvas_height, canvas_origin, unknown_value, distance_map_flat])
             # self.target_point.append([resolution, widthros, heightros, origin_point ,unknown_value, grid_flat])


            response.dmap = self.construct_msg(self.target_point)
            del self.target_point[0]
            response.success = True
            print("message sent.....")
            return response


        except Exception as e:
            response.success = False
            response.message = str(e)
            print("Exception!")
            print(e)
            return response
   
   
    def distance_map_cb_backup(self, _, response):

        try:

            #Get the plane 

            # o3d.visualization.draw_geometries([self.Point_cloud_paint])
            # o3d.io.write_point_cloud("/home/zyadan/ros2_ws_pro/src/pcdori.pcd", self.Point_cloud_paint)
            # Observepcd = o3d.io.read_point_cloud("/home/zyadan/ros2_ws_pro/src/pcdori.pcd")


            # [a, b, c, d] = plane_model
            # print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

            Observepcd = self.Point_cloud_paint

            planes = []
            wall_normal = []

            wall_pcd = o3d.geometry.PointCloud()
            while len(Observepcd.points) > 2500:

                plane_model, inliers = Observepcd.segment_plane(distance_threshold=0.005, ransac_n=3, num_iterations=200)
                [a, b, c, d] = plane_model
                # print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
                
                inlier_cloud = Observepcd.select_by_index(inliers)
                outlier_cloud = Observepcd.select_by_index(inliers, invert=True)
                planes.append(inlier_cloud)
                wall_normal.append(plane_model[:3])
                
                # for left 
                Observepcd = outlier_cloud

            angle_threshold = np.pi / 3 
            vertical_direction = np.array([0, 1, 0])

            for i in range(len(planes)):
                cos_theta = np.abs(np.dot(wall_normal[i], vertical_direction))

                if cos_theta < np.cos(angle_threshold):
                    wall_pcd_np = np.concatenate([np.asarray(wall_pcd.points), np.asarray(planes[i].points)], axis=0)
                    wall_pcd.points = o3d.utility.Vector3dVector(wall_pcd_np)

            wall_pcd_filtered, _ = wall_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=4.0)
            # o3d.visualization.draw_geometries([wall_pcd_filtered])



            np_cloud = np.asarray(wall_pcd_filtered)
            leaf_size = 0.05
        


            # x_min, y_min, z_min = np.amin(np_cloud, axis=0) 
            # x_max, y_max, z_max = np.amax(np_cloud, axis=0)

            resolution = leaf_size

            origin_point_np, distance_map = utils.point2distancemap(np.array(wall_pcd_filtered.points), leaf_size)
       
            height = distance_map.shape[0]
            width =distance_map.shape[1]
            print("resolution:  ",height,width)

            print("origin_point_np:  ",type(origin_point_np[0]))
            # task_map = np.full((height, height), 0)

            # np.savetxt('/home/zyadan/ros2_ws_pro/src/distance_map.txt', distance_map, delimiter=',')

            print("resolution:  ",resolution)

            origin_point = Point()
            origin_point.x = float(origin_point_np[0])
            origin_point.y = float(origin_point_np[1])
            origin_point.z = float(origin_point_np[2])

            #canvas_origin = origin_point
            canvas_origin = Point()
            canvas_origin.x = float((self.canvas_origin_inc[0] - origin_point.x) // leaf_size)
            print("here11111--------", canvas_origin.x )
            canvas_origin.y = float((self.canvas_origin_inc[1] - origin_point.y) // leaf_size)
            print("here22222--------", canvas_origin.y)
            canvas_origin.z = float(0)
            print("here-3333-------")
            canvas_width = int((self.canvas_rd_inc[0] - self.canvas_origin_inc[0] ) // leaf_size +1)
            canvas_height = int((self.canvas_rd_inc[1] - self.canvas_origin_inc[1] ) // leaf_size +1)

            print("canvas size-------", canvas_height, canvas_width)

            # distance_map_op = distance_map[canvas_origin.x:canvas_width, canvas_origin.y:canvas_height]

            unknown_value = 0.0

            distance_map_flat = [float(v) for dm in distance_map for v in dm]
            # distance_map_flat = np.array(distance_map_flat, dtype=np.float32)
            # task_map_flat = [bool(bv) for gb in task_map for bv in gb]
            # task_map_flat = np.array(task_map_flat, dtype=bool)
            print("distance_map_flat-----------------", type(distance_map_flat))
            # np.savetxt('/home/zyadan/ros2_ws_pro/src/distacne_map_flat.txt', distance_map_flat, delimiter=',')
   
            self.target_point = []
            dismapheader = Header()
            dismapheader.frame_id = self.source_frame
 

            self.target_point.append([dismapheader, resolution, width, height, origin_point, canvas_width, canvas_height, canvas_origin, unknown_value, distance_map_flat])
           # self.target_point.append([resolution, widthros, heightros, origin_point ,unknown_value, grid_flat])
            # print("dofjdjfdjaljdfkljakfjdshoeohrelkf", self.target_point)

            response.dmap = self.construct_msg(self.target_point)
            del self.target_point[0]
            response.success = True
            print("message send.....")
            return response


        except Exception as e:
            response.success = False
            response.message = str(e)
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
        # tc2req = DistanceMapSlice()
        tc2req = DistanceMapMsg()
        tc2req.header = ilist[0][0]
        tc2req.resolution = ilist[0][1]
        tc2req.width = ilist[0][2]
        tc2req.height = ilist[0][3]
        tc2req.origin = ilist[0][4]
        tc2req.canvas_width = ilist[0][5]
        tc2req.canvas_height = ilist[0][6]
        tc2req.canvas_origin = ilist[0][7]
        tc2req.unknown_value = ilist[0][8]
        tc2req.data = ilist[0][9]


        # print("msgs          dddddddddddddddd", tc2req)
        clist.append(tc2req)
        return tc2req


def main(args=None):
    rclpy.init(args=args)
    vision_prepro = VisionPrePro()

    rclpy.spin(vision_prepro)
    vision_prepro.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
