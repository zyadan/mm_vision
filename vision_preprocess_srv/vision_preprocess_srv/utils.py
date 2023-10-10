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
from geometry_msgs.msg import Point
from scipy.spatial import KDTree
import std_msgs
from rclpy.clock import Clock
import sys
import array
from pyntcloud import PyntCloud
from sensor_msgs.msg import PointCloud2,PointField


#from mayavi import mlab # for

import matplotlib.pyplot as plt

def pick_points(pcd):
    print("")
    print(
        "1) Please pick at least three correspondences using [shift + left click]"
    )
    print("   Press [shift + right click] to undo point picking")
    print("2) After picking points, press 'Q' to close the window")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # user picks points
    vis.destroy_window()
    print("")
    picked_points = vis.get_picked_points()

    return picked_points


# convert to grid with multiple points in a grid
def voxel_conv(point_cloud, leaf_size, Grid_value = "use_centroid"):
    grid_points = []
    # boundary
    x_min, y_min, z_min = np.amin(point_cloud, axis=0) 
    x_max, y_max, z_max = np.amax(point_cloud, axis=0)
 
    # dimension of voxel grid
    Dx = int((x_max - x_min)//leaf_size + 1)
    Dy = int((y_max - y_min)//leaf_size + 1)
    Dz = int((z_max - z_min)//leaf_size + 1)

    
 
    # index of voxel
    h = list()  
    hxyz = list() 
    grid_bi = np.zeros((Dx, Dy, Dz))
    for i in range(len(point_cloud)):
        hx = int((point_cloud[i][0] - x_min)//leaf_size)
        hy = int((point_cloud[i][1] - y_min)//leaf_size)
        hz = int((point_cloud[i][2] - z_min)//leaf_size)
        
        h.append(hx + hy*Dx + hz*Dx*Dy)
        hxyz.append([hx, hy, hz])
        grid_bi[hx,hy,hz]=1

    h = np.array(h)
 
    h_indice = np.argsort(h) # index range small-large 
    h_sorted = h[h_indice]
    


    # for h_index in np.unique(h):
    #     print("dx, dy, dz:", h, h_index)
    #     points = point_cloud[h==h_index]
    #     if Grid_value == "use_centroid":
    #         # use centroid for each voxel
    #         grid_points.append(np.mean(points, axis = 0))
    #     elif Grid_value == "random":
    #         # select point in each voxel randomly
    #         grid_points.append(random.choice(points))


    # grid_points = np.array(grid_points, dtype=np.float64)

    return grid_bi #return list
# convert to grid with multiple points in a grid


def PointFindOrigin(point_cloud, leaf_size):
    grid_points = []
    # boundary
    x_min, y_min, z_min = np.amin(point_cloud, axis=0) 
    x_max, y_max, z_max = np.amax(point_cloud, axis=0)
 
    # dimension of distance map
    Dx = int((x_max - x_min)//leaf_size + 1)
    Dy = int((y_max - y_min)//leaf_size + 1)
    Dz = int((z_max - z_min)//leaf_size + 1)

    
 
    # index of map grid
    h = list()  
    distance_map = np.zeros((Dy, Dx))
    for i in range(len(point_cloud)):
        hx = int((point_cloud[i][0] - x_min)//leaf_size)
        hy = int((point_cloud[i][1] - y_min)//leaf_size)
       
        h.append(hx + hy*Dx)
        

    h = np.array(h)
 
    h_indice = np.argsort(h) # index range small-large 
    h_sorted = h[h_indice]
    grid_points_temp = []
    origin_point = point_cloud[h_indice[0]]

    return origin_point



def point2distancemapbackup(point_cloud, leaf_size):
    grid_points = []
    # boundary
    x_min, y_min, z_min = np.amin(point_cloud, axis=0) 
    x_max, y_max, z_max = np.amax(point_cloud, axis=0)
 
    # dimension of distance map
    Dx = int((x_max - x_min)//leaf_size + 1)
    Dy = int((y_max - y_min)//leaf_size + 1)
    Dz = int((z_max - z_min)//leaf_size + 1)

    print("zmax and zmin", z_max, z_min, Dz)
    print("ymax and ymin", y_max, y_min, Dy)
    
 
    # index of map grid
    h = list()  
    distance_map = np.zeros((Dz, Dy))
    for i in range(len(point_cloud)):
        hx = int((point_cloud[i][1] - y_min)//leaf_size)
        hy = int((z_max - point_cloud[i][2] )//leaf_size)
       
        h.append(hx + hy*Dy)
        

    h = np.array(h)
 
    h_indice = np.argsort(h) # index range small-large 
    h_sorted = h[h_indice]
    grid_points_temp = []
    # origin_point = point_cloud[h_indice[0]]
    origin_point = Point()
    origin_point.x = point_cloud[h_indice[0]][0]
    origin_point.y = y_min
    origin_point.z = z_max

    np.append(h_sorted, -1)
    for i in range(len(h_sorted)-1):   # 0~9999
        indexinpc = h_indice[i]

        if h_sorted[i] == h_sorted[i + 1]:
            grid_points_temp.append(point_cloud[indexinpc])
            
        else:
            grid_points_temp.append(point_cloud[indexinpc])
            xtemp_min, _, _ = np.absolute(np.amax(grid_points_temp, axis=0))
            x_index = h_sorted[i] % Dy
            y_index = h_sorted[i] // Dy

            
            distance_map[y_index, x_index]= xtemp_min
            grid_points.append(grid_points_temp)
            grid_points_temp = []

    # grid_points = np.array(grid_points, dtype=np.float64)

    return origin_point, distance_map #return list

def open3d_to_ros(open3d_cloud, stamp=None, frame_id=None):

    xyz = np.asarray(open3d_cloud.points)
    print("xyx.shape", xyz.shape)
    # rgb = np.asarray(open3d_cloud.colors) #if have color

    # # make sure rgb in (0-255)
    # rgb = (rgb * 255).astype(np.uint8)

    # # make xyzrgb numpy
    # xyzrgb = np.concatenate((xyz, rgb), axis=1)
    clock = Clock()

    cloud_msg = PointCloud2()
    if stamp is not None:
        cloud_msg.header.stamp = stamp
    if frame_id is not None:
        cloud_msg.header.frame_id = frame_id
    
    # cloud_msg.header.stamp = clock.now().to_msg()
    # cloud_msg.header.frame_id = 'camera_link'
    cloud_msg.height = 1
    cloud_msg.width = xyz.shape[0]
    cloud_msg.is_bigendian = False
    cloud_msg.point_step = 12  # 3 float(4byte) per point: x, y, z, if with color---16
    cloud_msg.row_step = cloud_msg.point_step * xyz.shape[0]
    cloud_msg.is_dense = False
    cloud_msg.fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]
    cloud_msg.data = xyz.astype(np.float32).tobytes()
    
    return cloud_msg

def point2distancemap(point_cloud, leaf_size):
    grid_points = []
    grid_points_out = []
    # boundary
    x_min, y_min, z_min = np.amin(point_cloud, axis=0) 
    x_max, y_max, z_max = np.amax(point_cloud, axis=0)
 
    # dimension of distance map
    Dx = int((x_max - x_min)//leaf_size + 1)
    Dy = int((y_max - y_min)//leaf_size + 1)
    Dz = int((z_max - z_min)//leaf_size + 1)

    print("xmax and xmin", x_max, x_min, Dx)
    print("ymax and ymin", y_max, y_min, Dy)
    print("zmax and zmin", z_max, z_min, Dz)

    x_middle = (x_max + x_min)/2
    
 
    # index of map grid
    h = list()  
    distance_map = np.zeros((Dz, Dy))
    for i in range(len(point_cloud)):
        hx = int((point_cloud[i][1] - y_min)//leaf_size)
        hy = int((z_max - point_cloud[i][2] )//leaf_size)
       
        h.append(hx + hy*Dy)
        

    h = np.array(h)
 
    h_indice = np.argsort(h) # index range small-large 
    h_sorted = h[h_indice]
    grid_points_temp = []
    # origin_point = point_cloud[h_indice[0]]
    origin_point = Point()
    origin_point.x = point_cloud[h_indice[0]][0]
    origin_point.y = y_min
    origin_point.z = z_max

    np.append(h_sorted, -1)
    for i in range(len(h_sorted)-1):   # 0~9999
        indexinpc = h_indice[i]
        point_temp = [0, 0, 0]

        if h_sorted[i] == h_sorted[i + 1]:
            grid_points_temp.append(point_cloud[indexinpc])
            
        else:
            grid_points_temp.append(point_cloud[indexinpc])
            xtemp_min, _, _ = np.absolute(np.amax(grid_points_temp, axis=0))
            x_index = h_sorted[i] % Dy
            y_index = h_sorted[i] // Dy

            distance_map[y_index, x_index]= xtemp_min
            grid_points.append(grid_points_temp)
            grid_points_temp = []

            point_temp[0] = x_index * leaf_size + origin_point.y
            point_temp[1] = origin_point.z - y_index * leaf_size
            point_temp[2] = xtemp_min
            grid_points_out.append(point_temp)


    # grid_points = np.array(grid_points, dtype=np.float64)

    return origin_point, distance_map, grid_points_out #return list

def point2voxel(point_cloud, leaf_size):
    grid_points = []
    grid_points_out = []
    filtered_points = []
    # boundary
    x_min, y_min, z_min = np.amin(point_cloud, axis=0) 
    x_max, y_max, z_max = np.amax(point_cloud, axis=0)
 
    # dimension of distance map
    Dx = int((x_max - x_min)//leaf_size + 1)
    Dy = int((y_max - y_min)//leaf_size + 1)
    Dz = int((z_max - z_min)//leaf_size + 1)

    print("zmax and zmin", z_max, z_min, Dz)
    
 
    # index of map grid
    h = list()  
    distance_map = np.zeros((Dz, Dy))
    for i in range(len(point_cloud)):
        hx = int((point_cloud[i][1] - y_min)//leaf_size)
        hy = int((point_cloud[i][2] - z_min)//leaf_size)
       
        h.append(hx + hy*Dy)
        

    h = np.array(h)
 
    h_indice = np.argsort(h) # index range small-large 
    h_sorted = h[h_indice]
    grid_points_temp = []
    # origin_point = point_cloud[h_indice[0]]
    origin_point = Point()
    origin_point.x = point_cloud[h_indice[0]][0]
    origin_point.y = y_min
    origin_point.z = z_min
    count = 0 
    np.append(h_sorted, -1)
    for i in range(len(h_sorted)-1):   # 0~9999
        indexinpc = h_indice[i]
        point_temp = [0, 0, 0]

        if h_sorted[i] == h_sorted[i + 1]:
            grid_points_temp.append(point_cloud[indexinpc])
            continue
            
        else:
            grid_points_temp.append(point_cloud[indexinpc])
            xtemp_min, _, _ = np.absolute(np.amax(grid_points_temp, axis=0))
            x_index = h_sorted[i] % Dy
            y_index = h_sorted[i] // Dy


            point_idx = h_indice[count: i+1]
            random_points =  random.choice(point_cloud[point_idx])
            filtered_points.append(random_points)
            count = i

            distance_map[y_index, x_index]= xtemp_min
            grid_points.append(grid_points_temp)
            grid_points_temp = []
            
            point_temp[0] = x_index * leaf_size + origin_point.y
            point_temp[1] = origin_point.z - y_index * leaf_size
            point_temp[2] = xtemp_min
            grid_points_out.append(point_temp)


    filtered_points = np.array(filtered_points, dtype=np.float64)

    return origin_point, distance_map, filtered_points #return list


# convert to grid with multiple points in a grid
def point2distancemap_backup(point_cloud, leaf_size):
    grid_points = []
    # boundary
    x_min, y_min, z_min = np.amin(point_cloud, axis=0) 
    x_max, y_max, z_max = np.amax(point_cloud, axis=0)
 
    # dimension of distance map
    Dx = int((x_max - x_min)//leaf_size + 1)
    Dy = int((y_max - y_min)//leaf_size + 1)
    Dz = int((z_max - z_min)//leaf_size + 1)

    
 
    # index of map grid
    h = list()  
    distance_map = np.zeros((Dy, Dx))
    for i in range(len(point_cloud)):
        hx = int((point_cloud[i][0] - x_min)//leaf_size)
        hy = int((point_cloud[i][1] - y_min)//leaf_size)
       
        h.append(hx + hy*Dx)
        

    h = np.array(h)
 
    h_indice = np.argsort(h) # index range small-large 
    h_sorted = h[h_indice]
    grid_points_temp = []
    origin_point = point_cloud[h_indice[0]]


    np.append(h_sorted, -1)
    for i in range(len(h_sorted)-1):   # 0~9999
        indexinpc = h_indice[i]

        if h_sorted[i] == h_sorted[i + 1]:
            grid_points_temp.append(point_cloud[indexinpc])
            
        else:
            grid_points_temp.append(point_cloud[indexinpc])
            _, _, ztemp_min = np.amin(grid_points_temp, axis=0)
            x_index = h_sorted[i] % Dx
            y_index = h_sorted[i] // Dx

            
            distance_map[y_index, x_index]= ztemp_min
            grid_points.append(grid_points_temp)
            grid_points_temp = []

    # grid_points = np.array(grid_points, dtype=np.float64)

    return origin_point, distance_map #return list


# euler rotation
def rotate_mat(axis, radian):
    rot_matrix = linalg.expm(np.cross(np.eye(3), axis / linalg.norm(axis) * radian))
    return rot_matrix



def rotate_matrix_from_direccos(V_x, V_y, V_z):
    norm_Vx = np.linalg.norm(V_x)
    norm_Vy = np.linalg.norm(V_y)
    norm_Vz = np.linalg.norm(V_z)

    rotate_matW2L = np.array([[V_x[0]/norm_Vx,V_y[0]/norm_Vy, V_z[0]/norm_Vz], 
                             [V_x[1]/norm_Vx,V_y[1]/norm_Vy, V_z[1]/norm_Vz],
                             [V_x[2]/norm_Vx,V_y[2]/norm_Vy, V_z[2]/norm_Vz]])
    
    rotate_matL2W = np.array([[V_x[0]/norm_Vx,V_x[1]/norm_Vx, V_x[2]/norm_Vx], 
                             [V_y[0]/norm_Vy,V_y[1]/norm_Vy, V_y[2]/norm_Vy],
                             [V_z[0]/norm_Vz,V_z[1]/norm_Vz, V_z[2]/norm_Vz]])
    
    return rotate_matW2L, rotate_matL2W



convert_rgbUint32_to_tuple = lambda rgb_uint32: ((rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff))
convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value))

def convertCloudFromRosToOpen3d(cloud_data,field_names):
        # Check empty
        open3d_cloud = o3d.geometry.PointCloud()  
        if len(cloud_data)==0:
            print("Converting an empty cloud")
            return None

        # Set open3d_cloud
        if "rgb" in field_names:
            IDX_RGB_IN_FIELD=3 # x, y, z, rgb
            
            # Get xyz
            xyz = [(x,y,z) for x,y,z,rgb in cloud_data ] 

            # Check whether int or float
            if type(cloud_data[0][IDX_RGB_IN_FIELD])==int: # if float (from pcl::toROSMsg)
                rgb = [convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
            else:
                rgb = [convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]

            # combine
            open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))
            open3d_cloud.colors = o3d.utility.Vector3dVector(np.array(rgb)/255.0)
        else:
            xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
            open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))

        # return
        return open3d_cloud


def Gridmap(gridsize, gridnum, gridcenter, observeP):
    gridcenter = np.zeros((gridnum, gridnum),dtype = np.float32)
    for i in range(gridnum):
        for j in range(gridnum):
            gridcenter[i, j] = (-(gridnum/2 *gridsize - gridsize/2) + gridsize*i, -(gridnum/2 *gridsize - gridsize/2) + gridsize*j)

# mappings between PointField types and numpy types
type_mappings = [(PointField.INT8, np.dtype('int8')),
                 (PointField.UINT8, np.dtype('uint8')),
                 (PointField.INT16, np.dtype('int16')),
                 (PointField.UINT16, np.dtype('uint16')),
                 (PointField.INT32, np.dtype('int32')),
                 (PointField.UINT32, np.dtype('uint32')),
                 (PointField.FLOAT32, np.dtype('float32')),
                 (PointField.FLOAT64, np.dtype('float64'))]
pftype_to_nptype = dict(type_mappings)
nptype_to_pftype = dict((nptype, pftype) for pftype, nptype in type_mappings)


def dtype_to_fields(dtype):
    '''Convert a numpy record datatype into a list of PointFields.
    '''
    fields = []
    print("dtype.names", dtype.names)
    for field_name in dtype.names:
        np_field_type, field_offset = dtype.fields[field_name]
        print("field_offset",field_offset,np_field_type)
        pf = PointField()
        pf.name = field_name
        if np_field_type.subdtype:
            item_dtype, shape = np_field_type.subdtype
            pf.count = int(np.prod(shape))
            np_field_type = item_dtype
        else:
            pf.count = 1

        pf.datatype = nptype_to_pftype[np_field_type]
        pf.offset = field_offset
        fields.append(pf)
    return fields


def single_dtype_to_fields(dtype, field_name='field'):
    '''Convert a numpy datatype into a list of PointFields.
    '''
    fields = []

    pf = PointField()
    pf.name = field_name
    pf.datatype = nptype_to_pftype[dtype]
    pf.offset = 0
    pf.count = 1
    # fields.append(pf)
    return [pf]

def o3d_to_pointcloud2(cloud_o3d, stamp=None, frame_id=None):
    '''Converts open3d point cloud to a sensor_msgs.msg.PointCloud2.
    '''
    # make it 2d (even if height will be 1)

    cloud_arr_raw = np.array(cloud_o3d.points)
    fields = ['x', 'y', 'z']
    cloud_arr = np.zeros(cloud_arr_raw.shape[0], dtype=[(field, np.float32) for field in fields])
    # cloud_arr = np.empty(cloud_arr_raw.shape[0], dtype={'names': fields, 'formats': [cloud_arr_raw.dtype]*len(fields)})
    for i, field in enumerate(fields):
        cloud_arr[field] = cloud_arr_raw[:, i]

    
    # cloud_arr = np.atleast_2d(cloud_arr)
    
    
    cloud_msg = PointCloud2()

    if stamp is not None:
        cloud_msg.header.stamp = stamp
    if frame_id is not None:
        cloud_msg.header.frame_id = frame_id
    cloud_msg.height = cloud_arr.shape[0]
    cloud_msg.width = len(fields)
    cloud_msg.fields = dtype_to_fields(cloud_arr.dtype)
    cloud_msg.is_bigendian = sys.byteorder != 'little'
    cloud_msg.point_step = cloud_arr.dtype.itemsize
    cloud_msg.row_step = cloud_msg.point_step*len(fields)
    # cloud_msg.is_dense = False
    cloud_msg.is_dense = \
      all([np.isfinite(
            cloud_arr[fname]).all() for fname in cloud_arr.dtype.names])
    # The PointCloud2.data setter will create an array.array object for you if you don't
    # provide it one directly. This causes very slow performance because it iterates
    # over each byte in python.
    # Here we create an array.array object using a memoryview, limiting copying and
    # increasing performance.
    memory_view = memoryview(cloud_arr)
    if memory_view.nbytes > 0:
        array_bytes = memory_view.cast("B")
    else:
        # Casting raises a TypeError if the array has no elements
        array_bytes = b""
    as_array = array.array("B")
    as_array.frombytes(array_bytes)
    
    cloud_msg.data = as_array
    return cloud_msg

# from sensor_msgs.msg import PointCloud2, PointField
import std_msgs
from rclpy.clock import Clock


def open3d_to_ros(open3d_cloud, stamp=None, frame_id=None):

    xyz = np.asarray(open3d_cloud.points)
    print("xyx.shape", xyz.shape)
    # rgb = np.asarray(open3d_cloud.colors) #if have color

    # # make sure rgb in (0-255)
    # rgb = (rgb * 255).astype(np.uint8)

    # # make xyzrgb numpy
    # xyzrgb = np.concatenate((xyz, rgb), axis=1)
    clock = Clock()

    cloud_msg = PointCloud2()
    if stamp is not None:
        cloud_msg.header.stamp = stamp
    if frame_id is not None:
        cloud_msg.header.frame_id = frame_id
    
    # cloud_msg.header.stamp = clock.now().to_msg()
    # cloud_msg.header.frame_id = 'camera_link'
    cloud_msg.height = 1
    cloud_msg.width = xyz.shape[0]
    cloud_msg.is_bigendian = False
    cloud_msg.point_step = 12  # 3 float(4byte) per point: x, y, z, if with color---16
    cloud_msg.row_step = cloud_msg.point_step * xyz.shape[0]
    cloud_msg.is_dense = False
    cloud_msg.fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]
    cloud_msg.data = xyz.astype(np.float32).tobytes()
    
    return cloud_msg


if __name__ == '__main__':

    #source_dir = "../11_Oct/cam1/pcd/1633939443632570.pcd"
    source_dir = "../"
    pcd_source = o3d.io.read_point_cloud(os.path.join(source_dir,"mesh.ply"))
    pcd_grid = o3d.io.read_point_cloud(os.path.join(source_dir,"mesh.ply"))
    o3d.visualization.draw_geometries([pcd_source])
    #pcd_source.paint_uniform_color([1,0.706,0]) #yellow
    #o3d.visualization.draw_geometries([pcd_source])
    np_source = np.asarray(pcd_source.points)
    x = np_source[:,0]
    y = np_source[:,1]
    z = np_source[:,2]
    print("minmaxx  minmaxy minmaxz is {} , {} , {}, {} , {} , {}".format(np_source[:,0].min(),np_source[:,0].max(),np_source[:,1].min(),np_source[:,1].max(),np_source[:,2].min(),np_source[:,2].max()))

    np_source = np.asarray(pcd_source.points)
    # picked_points = pick_points(pcd_source)

    # leftupper = pcd_source.points[picked_points[0]]
    # rightbottom = pcd_source.points[picked_points[1]]
    # print("leftupper rightbottom:  ", leftupper,rightbottom) 
    # # after that, replace cloudroi[0] and cloudroi[1] to leftupper and rightbottom which showing in the teminal

    # bounding_ploy = np.array([
    #                 [leftupper[0],leftupper[1], 0],
    #                 [rightbottom[0], leftupper[1], 0],
    #                 [rightbottom[0], rightbottom[1], 0],
    #                 [leftupper[0], rightbottom[1], 0]
    #                 ], dtype = np.float32).reshape([-1, 3]).astype("float64")

    # bounding_polygon = np.array(bounding_ploy, dtype = np.float64)
    # vol = o3d.visualization.SelectionPolygonVolume()

    # #The Z-axis is used to define the height of the selected region
    # vol.orthogonal_axis = "Z"
    # vol.axis_max = np_source[:,2].max()
    # vol.axis_min =np_source[:,2].min()

    # vol.bounding_polygon = o3d.utility.Vector3dVector(bounding_polygon)
    # pcd_wall = vol.crop_point_cloud(pcd_source)
    # print("crop roi point cloud ....")
    # o3d.visualization.draw_geometries([pcd_wall])
    # xyz_load = np.asarray(pcd_wall.points)



    
    leaf_size = 0.05

    grid_bi = point2voxel(pcd_source.points, leaf_size)

    np.savetxt('/home/zyadan/catkin_ws_ros2pro/src/image_preprocess/grid_bi.txt', grid_bi, delimiter=',')
    # pcd_grid.points = o3d.utility.Vector3dVector(filtered_cloud)
    # 显示滤波后的点云
    # o3d.visualization.draw_geometries([pcd_grid])
    # cnt_downsampled = len(pcd_grid.points)
    # print("point count: ", " -> ", cnt_downsampled)


    print("Downsample the point cloud with a voxel of 0.05")
    print(grid_bi)
   # downpcd = pcd_source.voxel_down_sample(voxel_size=0.05)
    downpcd=o3d.geometry.voxel_down_sample(pcd_source ,voxel_size=0.05)
    # o3d.visualization.draw_geometries([downpcd])



    print("Downsample the point cloud with a voxel of 0.05")    

    obmap = ndimage.morphology.distance_transform_edt(grid_bi)
    print("obmap:....", obmap) 
    # np.savetxt('/home/zyadan/catkin_ws_ros2pro/src/image_preprocess/np_points.txt', grid_bi)




