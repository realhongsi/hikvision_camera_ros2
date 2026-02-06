/**
 * Rotate camera images 90° clockwise.
 * Subscribes: /camera/image_raw, /camera/ir/image_raw
 * Publishes: /camera/image_raw_rotated, /camera/ir/image_raw_rotated
 * Run after hikvision_camera_node (or synced topics). Does not modify existing code.
 */

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

namespace hikvision_camera_rotate
{

class ImageRotateNode : public rclcpp::Node
{
public:
  ImageRotateNode()
  : Node("image_rotate_node")
  {
    declare_parameter<std::string>("camera_topic", "/camera/image_raw");
    declare_parameter<std::string>("ir_topic", "/camera/ir/image_raw");
    declare_parameter<std::string>("camera_out_topic", "/camera/image_raw_rotated");
    declare_parameter<std::string>("ir_out_topic", "/camera/ir/image_raw_rotated");

    std::string camera_in = get_parameter("camera_topic").as_string();
    std::string ir_in = get_parameter("ir_topic").as_string();
    std::string camera_out = get_parameter("camera_out_topic").as_string();
    std::string ir_out = get_parameter("ir_out_topic").as_string();

    rclcpp::QoS qos = rclcpp::SensorDataQoS();

    sub_camera_ = create_subscription<sensor_msgs::msg::Image>(
      camera_in, qos, std::bind(&ImageRotateNode::cameraCallback, this, std::placeholders::_1));
    sub_ir_ = create_subscription<sensor_msgs::msg::Image>(
      ir_in, qos, std::bind(&ImageRotateNode::irCallback, this, std::placeholders::_1));

    pub_camera_ = create_publisher<sensor_msgs::msg::Image>(camera_out, qos);
    pub_ir_ = create_publisher<sensor_msgs::msg::Image>(ir_out, qos);

    RCLCPP_INFO(get_logger(), "Rotate 90° CW: %s -> %s, %s -> %s",
      camera_in.c_str(), camera_out.c_str(), ir_in.c_str(), ir_out.c_str());
  }

private:
  void cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    if (!cv_ptr) return;
    cv::Mat rotated;
    cv::rotate(cv_ptr->image, rotated, cv::ROTATE_90_CLOCKWISE);
    std_msgs::msg::Header header = msg->header;
    cv_bridge::CvImage out(header, "bgr8", rotated);
    pub_camera_->publish(*out.toImageMsg());
  }

  void irCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    if (!cv_ptr) return;
    cv::Mat rotated;
    cv::rotate(cv_ptr->image, rotated, cv::ROTATE_90_CLOCKWISE);
    std_msgs::msg::Header header = msg->header;
    cv_bridge::CvImage out(header, "bgr8", rotated);
    pub_ir_->publish(*out.toImageMsg());
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_camera_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_ir_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_camera_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_ir_;
};

}  // namespace hikvision_camera_rotate

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hikvision_camera_rotate::ImageRotateNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
