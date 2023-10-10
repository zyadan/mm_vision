import math

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf_transformations
import numpy as np
import quaternion


class FrameListener(Node):

    def __init__(self):
        super().__init__('vision_prepro')

        # Declare and acquire `target_frame` parameter
        self.source_frame = self.declare_parameter(
          'source_frame', 'shoulder_link').get_parameter_value().string_value
        
        self.target_frame = self.declare_parameter(
          'target_frame', 'base_link').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.on_timer)
    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = self.source_frame
        to_frame_rel = self.target_frame
        self.get_logger().info("frame:-------{}, {}".format(from_frame_rel, to_frame_rel))

        try:
            t_total = self.tf_buffer.lookup_transform(
                "base_link",
                "camera_link",
                rclpy.time.Time()).transform
            
            t = self.tf_buffer.lookup_transform(
                "shoulder_link",
                "camera_link",
                rclpy.time.Time()).transform
            

            t2 = self.tf_buffer.lookup_transform(
                'base_link',
                "shoulder_link",
                rclpy.time.Time()).transform
            
            t_total_q = [t_total.rotation.x, t_total.rotation.y, t_total.rotation.z, t_total.rotation.w]
            t_total_t = [t_total.translation.x,t_total.translation.y, t_total.translation.z]

            t_q = [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]
            t_qtest = [t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z]
            t_t = [t.translation.x,t.translation.y, t.translation.z]

            t2_q = [t2.rotation.x, t2.rotation.y, t2.rotation.z, t2.rotation.w]
            t2_qtest = [t2.rotation.w, t2.rotation.x, t2.rotation.y, t2.rotation.z]
            t2_t = [t2.translation.x,t2.translation.y, t2.translation.z]
            

            QQ = tf_transformations.quaternion_multiply(t2_q,t_q)
            self.get_logger().info("QQ----------------------:{}".format(QQ))
            QQtest = tf_transformations.quaternion_multiply(t2_qtest,t_qtest)
            self.get_logger().info("QQ----------------------:{}".format(QQtest))
            self.get_logger().info("Q_total----------------------:{}".format(t_total_q))
                
            
            euler= tf_transformations.euler_from_quaternion([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])
            self.get_logger().info("get transformation-translation: {},{},{}".format(t.translation.x,t.translation.y, t.translation.z))
            self.get_logger().info("get transformation-rolation:{}".format(euler))

            point = np.matrix([1,1,0], dtype='float32')
            point.resize((3,1))
            translation1 = np.matrix(t_t, dtype='float32')
            translation1.resize((3,1))
            translation2 = np.matrix(t2_t, dtype='float32')
            translation2.resize((3,1))
            translation_total = np.matrix(t_total_t, dtype='float32')
            translation_total.resize((3,1))  

            point_t = np.matrix([0,1,0,1], dtype='float32')
            point_t.resize((4,1))

            quat1 = np.quaternion(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z)
            quat2 = np.quaternion(t2.rotation.w, t2.rotation.x, t2.rotation.y, t2.rotation.z)
            quat_total = np.quaternion(t_total.rotation.w, t_total.rotation.x, t_total.rotation.y, t_total.rotation.z)

            # rotated_point_t = np.matmul(tf_transformations.quaternion_matrix([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]), point_t)
            rotated_point1 =  quaternion.as_rotation_matrix(quat1) * point
            # transform_point_1time = rotated_point1 + translation1
            rotated_point2 =  quaternion.as_rotation_matrix(quat2) * rotated_point1
            rotated_point_total =  quaternion.as_rotation_matrix(quat_total) * point

            transform_point_2time  = rotated_point2 + translation2 + quaternion.as_rotation_matrix(quat2) * translation1
            transform_point_total  = rotated_point_total + translation_total 


            # self.get_logger().info("quaternion.as_rotation_matrix(quat):{}".format(quaternion.as_rotation_matrix(quat)))
            # self.get_logger().info("quaternion.as_rotation_matrix(quat):{}".format(quaternion.as_rotation_matrix(quat)))
            self.get_logger().info("tf_transformations.quaternion_matrix:{}".format(tf_transformations.quaternion_matrix([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])))
            self.get_logger().info("rotated_point:{}".format(rotated_point2))
            self.get_logger().info("rotated_point_total:{}".format(rotated_point_total))
            self.get_logger().info("transform_point:{}".format(transform_point_2time))
            self.get_logger().info("transform_point_total:{}".format(transform_point_total))




        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return




def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()