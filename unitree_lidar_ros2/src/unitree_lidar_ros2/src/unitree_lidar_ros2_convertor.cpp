#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include "unitree_lidar_sdk_pcl.h"

#include <pcl/point_types.h>

struct OldPointType
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY
    std::uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(OldPointType,
                                  (float, x, x)
                                          (float, y, y)
                                          (float, z, z)
                                          (float, intensity, intensity)
                                          (std::uint16_t, ring, ring)
                                          (float, time, time)
)


class UnitreeLidarConvertor : public rclcpp::Node
{
public:
    UnitreeLidarConvertor(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("unitree_lidar_convertor", options)
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/unilidar/cloud", rclcpp::SensorDataQoS(),
                std::bind(&UnitreeLidarConvertor::pointcloud_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/unitree/pointcloud_raw_ex", rclcpp::SensorDataQoS());
    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<OldPointType>::Ptr input_cloud(new pcl::PointCloud<OldPointType>());
        pcl::PointCloud cloudIn = *input_cloud;
        pcl::fromROSMsg(*msg, cloudIn);

        // Process the point cloud
        // ...
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        cloudOut->clear();
        PointType pt;
        for (size_t i = 0; i < cloudIn.points.size(); i ++){
            pt.x = cloudIn.points[i].x;
            pt.y = cloudIn.points[i].y;
            pt.z = cloudIn.points[i].z;
            pt.intensity = cloudIn.points[i].intensity;
            pt.time = cloudIn.points[i].time;
            pt.ring = cloudIn.points[i].ring;
            pt.azimuth = atan2(cloudIn.points[i].y, cloudIn.points[i].x);
            pt.distance = hypot(cloudIn.points[i].x, cloudIn.points[i].y, cloudIn.points[i].z);
            cloudOut->push_back(pt);
        }

        // Publish the processed cloud
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*cloudOut, output_msg);
        output_msg.header.stamp = this->get_clock()->now();
        output_msg.header.frame_id = "base_link";
        publisher_->publish(output_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UnitreeLidarConvertor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(UnitreeLidarConvertor)