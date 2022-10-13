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

int count = 0,indexi,indexj;
double min = 1000;
// Going from degrees to radians
#define deg2rad 3.14/180

// The angle we use around 0deg, as well as 180deg, to get the grasps we view as good.
#define smallanglecalc 30

// Calculating the angles in radians.
#define smallangle smallanglecalc*deg2rad
#define largeangle1 (180 - smallanglecalc)*deg2rad
#define largeangle2 (180 + smallanglecalc)*deg2rad

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
      
      //RCLCPP_INFO(this->get_logger(), "height='%lf'\n", height);
      //RCLCPP_INFO(this->get_logger(), "width='%lf'\n", width);
      
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PassThrough<pcl::PointXYZRGB> pass;
      pcl::fromROSMsg(msg, *input_cloud);
      
      //cloud->points.resize (cloud->width * cloud->height);
      pass.setInputCloud (input_cloud);
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (0.0, 1);
    
      pass.filter (*filtered);

      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
     
      pcl::SACSegmentation<pcl::PointXYZRGB> seg;
      
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setDistanceThreshold (0.01);
      
      seg.setInputCloud (filtered);
      seg.segment (*inliers, *coefficients);
      
     if (inliers->indices.size () == 0)
     {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
       
     }
     
     pcl::ExtractIndices<pcl::PointXYZRGB> extract;
     extract.setInputCloud (filtered);
     extract.setIndices (inliers);
     extract.setNegative (true);
     extract.filter (*plane_cloud);
     
   
     pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
     ne.setInputCloud (plane_cloud);

     pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
     ne.setSearchMethod (tree);

  
     pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
     
     //plane_cloud contains the oject alone
     
     float centroid_x  = 0, centroid_y = 0, centroid_z = 0;
     float cloud_size = 0;
     for (auto& point: *plane_cloud)
     { cloud_size++;
       centroid_x += point.x;
       centroid_y += point.y;
       centroid_z += point.z;
     }
      
      
     centroid_x = centroid_x / cloud_size;
     centroid_y = centroid_y / cloud_size;
     centroid_z = centroid_z / cloud_size;

    RCLCPP_INFO(this->get_logger(), "Centroid of the grasp. \n");
     RCLCPP_INFO(this->get_logger(), "x1='%lf' y1='%lf' z1='%lf' \n",centroid_x,centroid_y,centroid_z);

     // Setting viewpoint as centroid of object
     ne.setViewPoint(centroid_x, centroid_y, centroid_z);
     ne.setRadiusSearch(0.03);

     ne.compute(*cloud_normals);
     
    
    double nx1, ny1, nz1; 
    double nx2, ny2, nz2;   // Normal vectors 
    
    double rx, ry, rz;      //Distance vector between the two points  
    
    double x1, y1, z1;
    double x2, y2, z2;      // Corresponding points of normals
    
    double theta1, theta2;
    double dot, mag, mag_r;
    
    for (auto i = 0; i < cloud_normals->size(); i++)
    {
        
    for (auto j = i+1; j < cloud_normals->size(); j++)
    {
      //RCLCPP_INFO(this->get_logger(), "normal;_x'%lf'\n",cloud_normals->points[j].normal_x);
        
      //getting the normal vectors
      nx1 = cloud_normals->points[i].normal_x;
      ny1 = cloud_normals->points[i].normal_y;
      nz1 = cloud_normals->points[i].normal_z;
        
      nx2 = cloud_normals->points[j].normal_x;
      ny2 = cloud_normals->points[j].normal_y;
      nz2 = cloud_normals->points[j].normal_z;
        
      // corresponding points
      x1 = (*plane_cloud)[i].x;
      y1 = (*plane_cloud)[i].y;
      z1 = (*plane_cloud)[i].z;
        
      x2 = (*plane_cloud)[j].x;
      y2 = (*plane_cloud)[j].y;
      z2 = (*plane_cloud)[j].z;
        
      //*** Do angle computations here ***//
       
      rx = x1 - x2;
      ry = y1 - y2;
      rz = z1 - z2;
      mag_r = sqrt(rx*rx + ry*ry + rz*rz);
        
      // Dot_products
      // Getting Theta1  
      dot = nx1*rx + ny1*ry + nz1*rz;
      mag = sqrt(nx1*nx1 + ny1*ny1 + nz1*nz1);
      mag = mag*mag_r;
      
      theta1 = acos(dot/mag);
      
      // Then Theta2
      dot = nx2*rx + ny2*ry + nz2*rz;
      mag = sqrt(nx2*nx2 + ny2*ny2 + nz2*nz2);
      mag = mag*mag_r;
      
      theta2 = acos(dot/mag);
        
      double dist1, dist2;
      if ((abs(theta1) < smallangle || (theta1 > largeangle1  && theta1 < largeangle2 )) && (abs(theta2) < smallangle  || (theta2 > largeangle1  && theta2 < largeangle2 )) && theta1 != theta2)
      {
        //RCLCPP_INFO(this->get_logger(), "Good Grasp !! \n");
        //RCLCPP_INFO(this->get_logger(), "Grasp Points: \n");
        // Giving the deg2rad value, makes it so that we don't actually output all values.
        RCLCPP_INFO(this->get_logger(), "x1='%lf' y1='%lf' z1='%lf' \n",x1,y1,z1);
        RCLCPP_INFO(this->get_logger(), "x2='%lf' y2='%lf' z2='%lf' \n",x2,y2,z2);
        
        // Let's change the color of these points to magenta. r = 255, g = 0, b = 255
        /**
        (*plane_cloud)[i].r = 255;
        (*plane_cloud)[i].g = 0;
        (*plane_cloud)[i].b = 255;

        (*plane_cloud)[j].r = 255;
        (*plane_cloud)[j].g = 0;
        (*plane_cloud)[j].b = 255;
        **/

        // Calculating distance of the grasp points from the centroid.  
        dist1 = sqrt((centroid_x - x1)*(centroid_x - x1)  +  (centroid_y - y1)*(centroid_y - y1)  +  (centroid_z - z1)*(centroid_z - z1));
        dist2 = sqrt((centroid_x - x2)*(centroid_x - x2)  +  (centroid_y - y2)*(centroid_y - y2)  +  (centroid_z - z2)*(centroid_z - z2));
          
        // Choosing points with minimum distance to center as the most optimal grasp.
        if (dist1 + dist2 <= min)
        {
          min = dist1 + dist2;
          indexi = i;
          indexj = j;
         
        }  
      }    
    }

    }
    
    RCLCPP_INFO(this->get_logger(), "Best Grasp Points !! \n");
   
    RCLCPP_INFO(this->get_logger(), "x1='%lf' y1='%lf' z1='%lf' \n",(*plane_cloud)[indexi].x,(*plane_cloud)[indexi].y,(*plane_cloud)[indexi].z);
    RCLCPP_INFO(this->get_logger(), "x2='%lf' y2='%lf' z2='%lf' \n",(*plane_cloud)[indexj].x,(*plane_cloud)[indexj].y,(*plane_cloud)[indexj].z);
    
        (*plane_cloud)[indexi].r = 255;
        (*plane_cloud)[indexi].g = 0;
        (*plane_cloud)[indexi].b = 255;

        (*plane_cloud)[indexj].r = 255;
        (*plane_cloud)[indexj].g = 0;
        (*plane_cloud)[indexj].b = 255;
      
    // We publish the point cloud.
    auto pc_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    //pcl::toROSMsg(*filtered, *pc_msg);
    pcl::toROSMsg(*plane_cloud, *pc_msg);
    //pcl::toROSMsg(*input_cloud, *pc_msg);
    pc_msg->header.frame_id = "world";
      
    publisher_->publish(*pc_msg);
     
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    // Let's publish the point cloud.
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PcSubscriber>());
  rclcpp::shutdown();
  return 0;
}
