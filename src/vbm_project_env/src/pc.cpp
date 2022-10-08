#include <functional>
#include <memory>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "rclcpp/rclcpp.hpp"
                                     
#include "std_msgs/msg/float32_multi_array.hpp"
using std::placeholders::_1;

class PcSubscriber : public rclcpp::Node
{
  public:
    PcSubscriber()
    : Node("pc_subscriber")
    {
    
      
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/realsense/points", 10, std::bind(&PcSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::PointCloud2 & msg) const
    {  
      float height=msg.height;
      float width=msg.width;
      RCLCPP_INFO(this->get_logger(), "height='%lf'\n", height);
      RCLCPP_INFO(this->get_logger(), "width='%lf'\n", width);
      
      
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PcSubscriber>());
  rclcpp::shutdown();
  return 0;
}
