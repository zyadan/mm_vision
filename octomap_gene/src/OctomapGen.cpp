#include "octomap_gene/OctomaoGen.hpp"


OctomapNode::OctomapNode() : Node("octomap_server")
{

    RCLCPP_INFO(this->get_logger(), "OctomapNode has been started.");
    OctomapService_ = this->create_service<tc5_msgs::srv::PointCloud2ToOctomap>("PointCloud2ToOctomap_service", std::bind(&OctomapNode::pointCloudToOctoMap, this, std::placeholders::_1, std::placeholders::_2));
    OctomapPublisher_ = this->create_publisher<octomap_msgs::msg::Octomap>("octomap", 10);

}



// pointCloud转成octomap
void OctomapNode::pointCloudToOctoMap(const std::shared_ptr<tc5_msgs::srv::PointCloud2ToOctomap::Request> req, 
                                      std::shared_ptr<tc5_msgs::srv::PointCloud2ToOctomap::Response> res)
{

    
    // octomap::Pointcloud cloud;
    // octomap_ros::pointCloud2ToOctomap(req->wall_point, cloud);
    // octomap::pose6d sensor_origin(0, 0, 0, 0, 0, 0);
    // m_octree.insertPointCloud(cloud, sensor_origin);

    // octomap_ros::fullMapToMsg(m_octree, res->octomap);
    

    for (int i = 0; i < req->wall_point.fields.size(); i++){
      RCLCPP_INFO(this->get_logger(),"Field name: %s", req->wall_point.fields[i].name.c_str());
      }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl_conversions::toPCL(req->wall_point, cloud);
    pcl::fromROSMsg(req->wall_point, *cloud);
    
    // create octomap
    auto tree = std::make_shared<octomap::OcTree>(0.1); // set resolution of the octomap
    
    for (const auto& point : cloud->points)
    {
      tree->updateNode(octomap::point3d(point.x, point.y, point.z), true);
    }
    
    tree->updateInnerOccupancy(); // update inner node occupancy
  // convert octomap to octomap_msgs::msg::Octomap
    // octomap_msgs::binaryMapToMsg(tree, res->octomap);

    // tree->writeBinary("/home/ros/robot_ws/src/Mobile_Manipulator/mm_vision/octomap.bt");
    octomap_msgs::msg::Octomap octomap_msg;
    octomap_msgs::fullMapToMsg(*tree, octomap_msg);

    // octomap_msg.header.stamp = this->get_clock()->now(); // or use the stamp from the input point cloud
    // octomap_msg.header.frame_id = "odom"; // set to appropriate frame_id
    // octomap_msg.id = "OcTree";
    // octomap_msg.binary = true;
    // octomap_msg.resolution = tree->getResolution();
    octomap_msg.header.stamp = req->wall_point.header.stamp; // or use the stamp from the input point cloud
    // octomap_msg.header.frame_id = "camera_link";
    octomap_msg.header.frame_id = req->wall_point.header.frame_id.c_str(); // set to appropriate frame_id
    octomap_msg.id = "OcTree";
    octomap_msg.binary = false;
    octomap_msg.resolution = tree->getResolution();
    // RCLCPP_INFO(this->get_logger(), "here333333333333333.");
    // std::stringstream datastream;
    // tree->writeBinary(datastream);
    // octomap_msg.data = std::vector<int8_t>(datastream.str().begin(), datastream.str().end());
    // RCLCPP_INFO(this->get_logger(), "here2222222222.");

    res->octomap = octomap_msg;
    OctomapPublisher_->publish(octomap_msg);
    RCLCPP_INFO(this->get_logger(), "OctomapNode msg send.");
    RCLCPP_INFO(this->get_logger(), "octomap_msg.header.frame_id; %s", octomap_msg.header.frame_id.c_str());
    // res->success = true;


      // octomap_msgs::msg::Octomap octomap_msg;
      // octomap_msg.header.stamp = this->get_clock()->now(); // or use the stamp from the input point cloud
      // octomap_msg.header.frame_id = "map"; // set to appropriate frame_id
      // octomap_msg.id = "OcTree";
      // octomap_msg.binary = true;
      // // octomap_msg.resolution = tree->getResolution();
      // res->octomap = octomap_msg;


}



