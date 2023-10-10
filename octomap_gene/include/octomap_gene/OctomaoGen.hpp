
#ifndef PROJECT_OCTOMAPGEN_H
#define PROJECT_OCTOMAPGEN_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/srv/bounding_box_query.hpp>
#include <octomap_msgs/srv/get_octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTreeKey.h>

// #include <octomap_ros/conversions.h>
// #include <octomap_msgs/conversions.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <tc5_msgs/srv/point_cloud2_to_octomap.hpp>


class OctomapNode: public rclcpp::Node{
public:
    OctomapNode();

protected:

private:
    void pointCloudToOctoMap(const std::shared_ptr<tc5_msgs::srv::PointCloud2ToOctomap::Request> req, 
                            std::shared_ptr<tc5_msgs::srv::PointCloud2ToOctomap::Response> res);


    rclcpp::Service<tc5_msgs::srv::PointCloud2ToOctomap>::SharedPtr OctomapService_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr OctomapPublisher_;
    // octomap::OcTree m_octree;

};

#endif