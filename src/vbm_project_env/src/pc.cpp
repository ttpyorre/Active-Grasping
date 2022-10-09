#include <functional>
#include <memory>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/impl/search.hpp>



int count=0;
using std::placeholders::_1;

class PcSubscriber : public rclcpp::Node
{
  public:
    PcSubscriber()
    : Node("pc_subscriber")
    {
    
      
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/realsense/points", 10, std::bind(&PcSubscriber::topic_callback, this, _1));
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output_cloud", 10);
                
    }

  private:
    void topic_callback(const sensor_msgs::msg::PointCloud2 & msg) const
    {  
      float height=msg.height;
      float width=msg.width;
      RCLCPP_INFO(this->get_logger(), "height='%lf'\n", height);
      RCLCPP_INFO(this->get_logger(), "width='%lf'\n", width);
      
      pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PassThrough<pcl::PointXYZ> pass;
      pcl::fromROSMsg(msg, *input_cloud);
      
      //cloud->points.resize (cloud->width * cloud->height);
      pass.setInputCloud (input_cloud);
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (0.0,1);
    
      pass.filter (*filtered);

      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
     
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setDistanceThreshold (0.01);
      
      seg.setInputCloud (filtered);
      seg.segment (*inliers, *coefficients);
      
     if (inliers->indices.size () == 0)
     {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
       
     }
     
     pcl::ExtractIndices<pcl::PointXYZ> extract;
     extract.setInputCloud (filtered);
     extract.setIndices (inliers);
     extract.setNegative (true);
     extract.filter (*plane_cloud);

      auto pc_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
      //pcl::toROSMsg(*filtered, *pc_msg);
      pcl::toROSMsg(*plane_cloud, *pc_msg);
      //pcl::toROSMsg(*input_cloud, *pc_msg);
      pc_msg->header.frame_id = "world";
      
      publisher_->publish(*pc_msg);
      if(count<1)
      {pcl::io::savePCDFileASCII ("test_pcd.pcd",*filtered);
      }
      count+=1;
     }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PcSubscriber>());
  rclcpp::shutdown();
  return 0;
}
