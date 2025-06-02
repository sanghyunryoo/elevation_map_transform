#include <rclcpp/rclcpp.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <cmath>

// 기존 PointCloud2 헤더 대신, custom HeightMap 메시지 헤더를 include
#include <height_map_msgs/msg/height_map.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class ElevationMapProcessor : public rclcpp::Node {
public:
  explicit ElevationMapProcessor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("elevation_map_processor", options)
  {
    // 1) 파라미터 선언
    declare_parameter<double>("roi_x_min", -1.0);
    declare_parameter<double>("roi_x_max",  1.0);
    declare_parameter<double>("roi_y_min", -1.0);
    declare_parameter<double>("roi_y_max",  1.0);
    declare_parameter<double>("sampling_resolution", 0.05);

    // 2) 파라미터 읽기
    roi_x_min_           = get_parameter("roi_x_min").as_double();
    roi_x_max_           = get_parameter("roi_x_max").as_double();
    roi_y_min_           = get_parameter("roi_y_min").as_double();
    roi_y_max_           = get_parameter("roi_y_max").as_double();
    sampling_resolution_ = get_parameter("sampling_resolution").as_double();

    // 3) TF2 버퍼 및 리스너
    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 4) 구독자: elevation_map (GridMap)
    elevation_map_sub_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
      "/elevation_map", rclcpp::QoS(1),
      std::bind(&ElevationMapProcessor::elevationMapCallback, this, std::placeholders::_1)
    );

    // 5) 기존: dense map 퍼블리셔 (변경 없음)
    elevation_map_dense_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
      "/elevation_map_dense", rclcpp::QoS(1)
    );

    // 6) **변경**: PointCloud2 대신 HeightMap 타입 퍼블리셔로 변경
    pointcloud_as_vector_pub_ = this->create_publisher<height_map_msgs::msg::HeightMap>(
      "/dense_map_roi_vector", rclcpp::QoS(1)
    );
  }

private:
  void elevationMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
  {
    try {
      // ───────────────────────────────────────────────────────
      // (1) GridMap 메시지를 grid_map::GridMap 객체로 변환
      // ───────────────────────────────────────────────────────
      grid_map::GridMap map;
      if (!grid_map::GridMapRosConverter::fromMessage(*msg, map)) {
        RCLCPP_ERROR(get_logger(), "Failed to convert GridMap message to object.");
        return;
      }

      // ───────────────────────────────────────────────────────
      // (2) NaN인 부분을 0.0으로 대체하여 dense_map 생성
      // ───────────────────────────────────────────────────────
      grid_map::GridMap dense_map = map;
      for (const auto & layer : dense_map.getLayers()) {
        for (grid_map::GridMapIterator it(dense_map); !it.isPastEnd(); ++it) {
          if (!dense_map.isValid(*it, layer)) {
            dense_map.at(layer, *it) = 0.0;
          }
        }
      }

      // ───────────────────────────────────────────────────────
      // (3) dense_map 퍼블리시 (기존 로직 유지)
      // ───────────────────────────────────────────────────────
      dense_map.setFrameId(map.getFrameId());
      auto dense_msg_ptr = grid_map::GridMapRosConverter::toMessage(dense_map);
      if (dense_msg_ptr) {
        dense_msg_ptr->header.stamp = msg->header.stamp;
        elevation_map_dense_pub_->publish(*dense_msg_ptr);
        RCLCPP_INFO(get_logger(), "Published elevation_map_dense");
      }

      // ───────────────────────────────────────────────────────
      // (4) 로봇 위치 TF 조회 (map -> base_link)
      // ───────────────────────────────────────────────────────
      auto transform_stamped = tf_buffer_->lookupTransform(
        "map", "base_link",
        msg->header.stamp,
        rclcpp::Duration::from_seconds(0.1)
      );
      double trans_x = transform_stamped.transform.translation.x;
      double trans_y = transform_stamped.transform.translation.y;
      double yaw     = tf2::getYaw(transform_stamped.transform.rotation);

      // ───────────────────────────────────────────────────────
      // (5) ROI 내 격자 크기(행×열) 계산
      // ───────────────────────────────────────────────────────
      // X 방향 칸 수 (열)
      int cols = static_cast<int>(std::floor((roi_x_max_ - roi_x_min_) / sampling_resolution_)) + 1;
      // Y 방향 칸 수 (행)
      int rows = static_cast<int>(std::floor((roi_y_max_ - roi_y_min_) / sampling_resolution_)) + 1;

      // (a) 결과 벡터 초기화 (rows × cols만큼)
      std::vector<float> flat_data;
      flat_data.assign(static_cast<size_t>(rows * cols), 0.0f);

      // ───────────────────────────────────────────────────────
      // (6) ROI 그리드마다 map에서 z값(“elevation” 레이어) 읽어 와서 flat_data에 저장
      // ───────────────────────────────────────────────────────
      // for문 인덱스: i=X축 인덱스(0..cols-1), j=Y축 인덱스(0..rows-1)
      for (int i = 0; i < cols; ++i) {
        double x_local = roi_x_min_ + i * sampling_resolution_;
        for (int j = 0; j < rows; ++j) {
          double y_local = roi_y_min_ + j * sampling_resolution_;

          // (a) 로봇 좌표계 → 맵 좌표계로 변환
          double cos_y = std::cos(yaw);
          double sin_y = std::sin(yaw);
          double x_map = cos_y * x_local - sin_y * y_local + trans_x;
          double y_map = sin_y * x_local + cos_y * y_local + trans_y;

          // (b) 맵 좌표 → GridMap 인덱스
          grid_map::Index idx;
          bool is_in_map = dense_map.getIndex({x_map, y_map}, idx) &&
                           dense_map.isValid(idx, "elevation");
          float z_val = 0.0f;
          if (is_in_map) {
            z_val = static_cast<float>(dense_map.at("elevation", idx));
          } else {
            // 범위 밖이거나 유효하지 않을 땐, 0.0 (또는 NaN 등 원하는 값) 사용
            z_val = 0.0f;
          }

          // (c) flat_data에 저장: 인덱스 = j*cols + i
          flat_data[static_cast<size_t>(j * cols + i)] = z_val;
        }
      }

      // ───────────────────────────────────────────────────────
      // (7) custom HeightMap 메시지 생성하여 퍼블리시
      // ───────────────────────────────────────────────────────
      auto out_msg = std::make_shared<height_map_msgs::msg::HeightMap>();

      // (a) Header 설정
      out_msg->header.stamp = msg->header.stamp;
      out_msg->header.frame_id = "map";  // 필요에 따라 프레임 아이디 변경

      // (b) width, height 설정
      out_msg->width  = static_cast<uint32_t>(cols);
      out_msg->height = static_cast<uint32_t>(rows);

      // (c) data 벡터 할당 (std::vector<float> → ROS 메시지 필드)
      out_msg->data = flat_data;

      // (d) 퍼블리시
      pointcloud_as_vector_pub_->publish(*out_msg);
      RCLCPP_INFO(get_logger(),
                  "Published HeightMap vector: [%u cols × %u rows] (total %zu floats)",
                  out_msg->width,
                  out_msg->height,
                  out_msg->data.size());
    }
    catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        get_logger(),
        "Could not transform map -> base_link: %s",
        ex.what()
      );
    }
  }

  // ───────────────────────────────────────────────────────
  //  멤버 변수 선언
  // ───────────────────────────────────────────────────────
  double roi_x_min_, roi_x_max_, roi_y_min_, roi_y_max_;
  double sampling_resolution_;

  // TF2
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // 구독자
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr elevation_map_sub_;

  // 기존 dense map 퍼블리셔
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr elevation_map_dense_pub_;

  // **변경**: 벡터 형태 메시지 퍼블리셔
  rclcpp::Publisher<height_map_msgs::msg::HeightMap>::SharedPtr pointcloud_as_vector_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ElevationMapProcessor>());
  rclcpp::shutdown();
  return 0;
}
