#include <rclcpp/rclcpp.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class ElevationMapProcessor : public rclcpp::Node {
public:
  explicit ElevationMapProcessor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("elevation_map_processor", options)
  {
    // Declare parameters
    declare_parameter<double>("roi_x_min", -1.0);
    declare_parameter<double>("roi_x_max",  1.0);
    declare_parameter<double>("roi_y_min", -1.0);
    declare_parameter<double>("roi_y_max",  1.0);
    declare_parameter<double>("sampling_resolution", 0.05);

    // Read parameters
    roi_x_min_           = get_parameter("roi_x_min").as_double();
    roi_x_max_           = get_parameter("roi_x_max").as_double();
    roi_y_min_           = get_parameter("roi_y_min").as_double();
    roi_y_max_           = get_parameter("roi_y_max").as_double();
    sampling_resolution_ = get_parameter("sampling_resolution").as_double();

    // TF2 buffer & listener
    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscribers & Publishers
    elevation_map_sub_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
      "/elevation_map", rclcpp::QoS(1),
      std::bind(&ElevationMapProcessor::elevationMapCallback, this, std::placeholders::_1)
    );

    elevation_map_dense_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
      "/elevation_map_dense", rclcpp::QoS(1)
    );

    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/dense_map_roi_pointcloud", rclcpp::QoS(1)
    );
  }

private:
  void elevationMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
  {
    try {
      // 1) Convert to GridMap
      grid_map::GridMap map;
      if (!grid_map::GridMapRosConverter::fromMessage(*msg, map)) {
        RCLCPP_ERROR(get_logger(), "Failed to convert GridMap message to object.");
        return;
      }

      // 2) Replace NaN with 0.0
      grid_map::GridMap dense_map = map;
      for (const auto & layer : dense_map.getLayers()) {
        for (grid_map::GridMapIterator it(dense_map); !it.isPastEnd(); ++it) {
          if (!dense_map.isValid(*it, layer)) {
            dense_map.at(layer, *it) = 0.0;
          }
        }
      }

      // 3) Publish dense map
      dense_map.setFrameId(map.getFrameId());
      auto dense_msg_ptr = grid_map::GridMapRosConverter::toMessage(dense_map);
      if (dense_msg_ptr) {
        dense_msg_ptr->header.stamp = msg->header.stamp;
        elevation_map_dense_pub_->publish(*dense_msg_ptr);
        RCLCPP_INFO(get_logger(), "Published elevation_map_dense");
      }

      // 4) Lookup robot pose (map -> base_link)
      auto transform_stamped = tf_buffer_->lookupTransform(
        "map", "base_link",
        msg->header.stamp,
        rclcpp::Duration::from_seconds(0.1)
      );
      double trans_x = transform_stamped.transform.translation.x;
      double trans_y = transform_stamped.transform.translation.y;
      double yaw     = tf2::getYaw(transform_stamped.transform.rotation);

      // 5) Generate robot-centric grid within ROI
      using PointT      = pcl::PointXYZ;
      using PointCloudT = pcl::PointCloud<PointT>;

      auto roi_cloud = std::make_shared<PointCloudT>();
      roi_cloud->header.frame_id = "base_footprint";

      for (double x_local = roi_x_min_; x_local <= roi_x_max_; x_local += sampling_resolution_) {
        for (double y_local = roi_y_min_; y_local <= roi_y_max_; y_local += sampling_resolution_) {
          // Transform to map frame
          double cos_y = std::cos(yaw);
          double sin_y = std::sin(yaw);
          double x_map = cos_y * x_local - sin_y * y_local + trans_x;
          double y_map = sin_y * x_local + cos_y * y_local + trans_y;

          // Check inside map
          grid_map::Index idx;
          if (!dense_map.getIndex({x_map, y_map}, idx) ||
              !dense_map.isValid(idx, "elevation"))
          {
            continue;
          }

          // Create point
          PointT p;
          p.x = x_local;
          p.y = y_local;
          p.z = dense_map.at("elevation", idx);
          roi_cloud->points.push_back(p);
        }
      }

      // 6) Publish point cloud
      sensor_msgs::msg::PointCloud2 output_msg;
      pcl::toROSMsg(*roi_cloud, output_msg);
      output_msg.header.stamp = msg->header.stamp;
      output_msg.header.frame_id = roi_cloud->header.frame_id;
      point_cloud_pub_->publish(output_msg);

    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        get_logger(),
        "Could not transform map -> base_link: %s",
        ex.what()
      );
    }
  }

  // Parameters
  double roi_x_min_, roi_x_max_, roi_y_min_, roi_y_max_;
  double sampling_resolution_;

  // TF2
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ROS interfaces
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr elevation_map_sub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr elevation_map_dense_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ElevationMapProcessor>());
  rclcpp::shutdown();
  return 0;
}
