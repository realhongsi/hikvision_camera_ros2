/**
 * Timestamp-based sync of /camera/image_raw and /camera/ir/image_raw.
 * Publishes synced pairs:
 *   - /camera/image_pair_synced (ImagePair: one message = one pair, same Hz)
 *   - /camera/image_raw_synced, /camera/ir/image_raw_synced (separate topics)
 * Run after hikvision_camera_node is publishing.
 */

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "message_filters/subscriber.hpp"
#include "message_filters/sync_policies/approximate_time.hpp"
#include "message_filters/synchronizer.hpp"

#include "hikvision_camera_ros2/msg/image_pair.hpp"

namespace hikvision_camera_ros2
{

class ImageSyncNode : public rclcpp::Node
{
public:
  ImageSyncNode()
  : Node("image_sync_node")
  {
    declare_parameter<std::string>("camera_topic", "/camera/image_raw");
    declare_parameter<std::string>("ir_topic", "/camera/ir/image_raw");
    declare_parameter<std::string>("camera_synced_topic", "/camera/image_raw_synced");
    declare_parameter<std::string>("ir_synced_topic", "/camera/ir/image_raw_synced");
    declare_parameter<std::string>("pair_topic", "/camera/image_pair_synced");
    declare_parameter<int>("queue_size", 10);
    declare_parameter<double>("max_interval", 0.05);

    std::string camera_topic = get_parameter("camera_topic").as_string();
    std::string ir_topic = get_parameter("ir_topic").as_string();
    std::string camera_synced = get_parameter("camera_synced_topic").as_string();
    std::string ir_synced = get_parameter("ir_synced_topic").as_string();
    std::string pair_topic = get_parameter("pair_topic").as_string();
    int queue_size = get_parameter("queue_size").as_int();
    double max_interval = get_parameter("max_interval").as_double();

    rclcpp::QoS qos = rclcpp::SensorDataQoS();

    sub_camera_.subscribe(this, camera_topic, qos.get_rmw_qos_profile());
    sub_ir_.subscribe(this, ir_topic, qos.get_rmw_qos_profile());

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image,
      sensor_msgs::msg::Image>;
    SyncPolicy policy(queue_size);
    policy.setMaxIntervalDuration(rclcpp::Duration::from_seconds(max_interval));
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      static_cast<const SyncPolicy&>(policy),
      sub_camera_,
      sub_ir_);
    sync_->registerCallback(
      std::bind(&ImageSyncNode::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

    pub_camera_ = create_publisher<sensor_msgs::msg::Image>(camera_synced, qos);
    pub_ir_ = create_publisher<sensor_msgs::msg::Image>(ir_synced, qos);
    pub_pair_ = create_publisher<hikvision_camera_ros2::msg::ImagePair>(pair_topic, qos);

    RCLCPP_INFO(get_logger(), "Syncing %s + %s -> %s (one topic = same Hz), and %s, %s (max interval %.0f ms)",
      camera_topic.c_str(), ir_topic.c_str(),
      pair_topic.c_str(), camera_synced.c_str(), ir_synced.c_str(),
      max_interval * 1000.0);
  }

private:
  void syncCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& camera_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& ir_msg)
  {
    pub_camera_->publish(*camera_msg);
    pub_ir_->publish(*ir_msg);
    hikvision_camera_ros2::msg::ImagePair pair;
    pair.camera = *camera_msg;
    pair.ir = *ir_msg;
    pub_pair_->publish(pair);
  }

  message_filters::Subscriber<sensor_msgs::msg::Image> sub_camera_;
  message_filters::Subscriber<sensor_msgs::msg::Image> sub_ir_;
  std::shared_ptr<message_filters::Synchronizer<
    message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image,
      sensor_msgs::msg::Image>>> sync_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_camera_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_ir_;
  rclcpp::Publisher<hikvision_camera_ros2::msg::ImagePair>::SharedPtr pub_pair_;
};

}  // namespace hikvision_camera_ros2

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hikvision_camera_ros2::ImageSyncNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
