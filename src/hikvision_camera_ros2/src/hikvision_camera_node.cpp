/**
 * Hikvision Camera ROS2 Node
 * - RTSP mode (default): smooth high-FPS stream via OpenCV VideoCapture
 * - SDK mode: JPEG capture (low FPS, 1~3 Hz)
 */

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <atomic>
#include <unistd.h>
#include <curl/curl.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

#include "HCNetSDK.h"

namespace hikvision_camera_ros2
{

class HikvisionCameraNode : public rclcpp::Node
{
public:
  HikvisionCameraNode()
  : Node("hikvision_camera_node"),
    user_id_(-1),
    sdk_initialized_(false),
    running_(false)
  {
    declare_parameter<std::string>("device_ip", "192.168.1.64");
    declare_parameter<int>("port", 8000);
    declare_parameter<std::string>("username", "admin");
    declare_parameter<std::string>("password", "");
    declare_parameter<int>("channel_visible", 1);
    declare_parameter<int>("channel_ir", 2);
    declare_parameter<double>("frame_rate", 25.0);
    declare_parameter<std::string>("camera_frame_id", "camera_optical_frame");
    declare_parameter<std::string>("ir_frame_id", "ir_optical_frame");
    declare_parameter<bool>("use_rtsp", true);
    declare_parameter<bool>("low_latency", true);
    declare_parameter<bool>("use_sub_stream", false);
    declare_parameter<bool>("camera_use_ffmpeg", false);
    declare_parameter<std::string>("rtsp_camera_url", "");
    declare_parameter<std::string>("rtsp_ir_url", "");
    declare_parameter<bool>("publish_temperature_topic", true);
    declare_parameter<double>("temperature_poll_interval", 1.0);

    device_ip_ = get_parameter("device_ip").as_string();
    port_ = get_parameter("port").as_int();
    username_ = get_parameter("username").as_string();
    password_ = get_parameter("password").as_string();
    channel_visible_ = get_parameter("channel_visible").as_int();
    channel_ir_ = get_parameter("channel_ir").as_int();
    frame_rate_ = get_parameter("frame_rate").as_double();
    camera_frame_id_ = get_parameter("camera_frame_id").as_string();
    ir_frame_id_ = get_parameter("ir_frame_id").as_string();
    use_rtsp_ = get_parameter("use_rtsp").as_bool();
    low_latency_ = get_parameter("low_latency").as_bool();
    use_sub_stream_ = get_parameter("use_sub_stream").as_bool();
    camera_use_ffmpeg_ = get_parameter("camera_use_ffmpeg").as_bool();
    rtsp_camera_url_ = get_parameter("rtsp_camera_url").as_string();
    rtsp_ir_url_ = get_parameter("rtsp_ir_url").as_string();
    publish_temperature_topic_ = get_parameter("publish_temperature_topic").as_bool();
    temperature_poll_interval_ = get_parameter("temperature_poll_interval").as_double();

    rclcpp::QoS qos = rclcpp::SensorDataQoS();
    if (low_latency_) qos.keep_last(1);
    pub_camera_ = create_publisher<sensor_msgs::msg::Image>("camera/image_raw", qos);
    pub_ir_ = create_publisher<sensor_msgs::msg::Image>("camera/ir/image_raw", qos);
    if (publish_temperature_topic_) {
      pub_temperature_ = create_publisher<std_msgs::msg::Float64MultiArray>("camera/ir/temperature_info", 10);
    }

    if (use_rtsp_) {
      if (publish_temperature_topic_) {
        if (!init_sdk_and_login()) {
          RCLCPP_INFO(get_logger(), "SDK login failed; temperature will be fetched via ISAPI (HTTP Digest) when available.");
        }
        RCLCPP_INFO(get_logger(), "Publishing temperature on /camera/ir/temperature_info (cached, poll every %.1f s; no block on video).", temperature_poll_interval_);
        running_temp_ = true;
        temperature_thread_ = std::thread(&HikvisionCameraNode::temperature_background_loop, this);
      }
      double publish_hz = (low_latency_ && frame_rate_ < 30.0) ? 30.0 : frame_rate_;
      publish_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_hz),
        std::bind(&HikvisionCameraNode::publish_buffered_frames, this));
      start_rtsp();
    } else {
      if (!init_sdk_and_login()) {
        RCLCPP_ERROR(get_logger(), "Failed to init SDK or login.");
        return;
      }
      if (publish_temperature_topic_) {
        running_temp_ = true;
        temperature_thread_ = std::thread(&HikvisionCameraNode::temperature_background_loop, this);
      }
      start_sdk_capture();
    }
  }

  ~HikvisionCameraNode()
  {
    running_ = false;
    running_temp_ = false;
    if (temperature_thread_.joinable()) temperature_thread_.join();
    if (rtsp_thread_.joinable()) rtsp_thread_.join();
    if (rtsp_ir_thread_.joinable()) rtsp_ir_thread_.join();
    if (user_id_ >= 0) NET_DVR_Logout_V30(user_id_);
    if (sdk_initialized_) NET_DVR_Cleanup();
  }

private:
  std::string ensure_rtsp_tcp(const std::string& url) const
  {
    if (url.find("rtsp_transport=") != std::string::npos) return url;
    return url + (url.find('?') != std::string::npos ? "&" : "?") + "rtsp_transport=tcp";
  }

  void start_rtsp()
  {
    std::string cam_url = rtsp_camera_url_.empty() ? build_rtsp_url(channel_visible_) : ensure_rtsp_tcp(rtsp_camera_url_);
    std::string ir_url  = rtsp_ir_url_.empty() ? build_rtsp_url(channel_ir_) : ensure_rtsp_tcp(rtsp_ir_url_);

    RCLCPP_INFO(get_logger(), "Publishing to /camera/image_raw and /camera/ir/image_raw (run: ros2 topic list)");
    RCLCPP_INFO(get_logger(), "RTSP mode: camera=%s, ir=%s", cam_url.c_str(), ir_url.c_str());

    running_ = true;
    rtsp_thread_ = std::thread(&HikvisionCameraNode::rtsp_loop, this, cam_url, true);
    if (channel_ir_ != channel_visible_ && !ir_url.empty())
      rtsp_ir_thread_ = std::thread(&HikvisionCameraNode::rtsp_loop, this, ir_url, false);
    else
      rtsp_ir_thread_ = std::thread(&HikvisionCameraNode::rtsp_loop, this, cam_url, false);
  }

  std::string build_rtsp_url(int channel)
  {
    int path = channel * 100 + (use_sub_stream_ ? 2 : 1);
    char buf[512];
    snprintf(buf, sizeof(buf), "rtsp://%s:%s@%s/Streaming/Channels/%d?rtsp_transport=tcp",
             username_.c_str(), password_.c_str(), device_ip_.c_str(), path);
    return std::string(buf);
  }

  bool open_rtsp_low_latency(cv::VideoCapture& cap, const std::string& url, bool is_camera)
  {
    bool prefer_ffmpeg = is_camera && camera_use_ffmpeg_;
    if (prefer_ffmpeg) {
      if (cap.open(url, cv::CAP_FFMPEG)) {
        if (low_latency_) cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
        return true;
      }
      return false;
    }
    std::string pipeline = "rtspsrc location=\"" + url + "\" latency=0 protocols=tcp ! rtph264depay ! h264parse ! decodebin ! videoconvert ! video/x-raw,format=BGR ! appsink";
    if (cap.open(pipeline, cv::CAP_GSTREAMER)) return true;
    if (cap.open(url, cv::CAP_FFMPEG)) {
      cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
      return true;
    }
    return false;
  }

  void rtsp_loop(const std::string& url, bool is_camera)
  {
    cv::VideoCapture cap;
    bool ok = low_latency_
      ? open_rtsp_low_latency(cap, url, is_camera)
      : (cap.open(url, cv::CAP_FFMPEG));
    if (!ok) {
      RCLCPP_ERROR(get_logger(), "Failed to open RTSP: %s", url.c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "RTSP connected, publishing %s", is_camera ? "camera_optical_frame" : "ir_optical_frame");
    double fps = cap.get(cv::CAP_PROP_FPS);
    if (fps <= 0) fps = 25.0;
    auto period = std::chrono::duration<double>(1.0 / fps);
    cv::Mat frame;
    while (rclcpp::ok() && running_) {
      if (!cap.read(frame) || frame.empty()) {
        cap.release();
        std::this_thread::sleep_for(std::chrono::seconds(1));
        ok = low_latency_ ? open_rtsp_low_latency(cap, url, is_camera) : cap.open(url, cv::CAP_FFMPEG);
        if (!ok) continue;
        continue;
      }
      if (is_camera) {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        camera_buffer_ = frame.clone();
      } else {
        std::lock_guard<std::mutex> lock(ir_mutex_);
        ir_buffer_ = frame.clone();
      }
      if (!low_latency_) std::this_thread::sleep_for(period);
    }
    cap.release();
  }

  void publish_buffered_frames()
  {
    {
      std::lock_guard<std::mutex> lock(camera_mutex_);
      if (!camera_buffer_.empty()) {
        camera_publish_ = camera_buffer_.clone();
      }
    }
    if (!camera_publish_.empty()) {
      std_msgs::msg::Header header;
      header.stamp = now();
      header.frame_id = camera_frame_id_;
      std::shared_ptr<cv_bridge::CvImage> cv_ptr = std::make_shared<cv_bridge::CvImage>(header, "bgr8", camera_publish_);
      pub_camera_->publish(*cv_ptr->toImageMsg());
    }
    {
      std::lock_guard<std::mutex> lock(ir_mutex_);
      if (!ir_buffer_.empty()) {
        ir_publish_ = ir_buffer_.clone();
      }
    }
    if (!ir_publish_.empty()) {
      if (publish_temperature_topic_) {
        publish_temperature();
      }
      std_msgs::msg::Header header;
      header.stamp = now();
      header.frame_id = ir_frame_id_;
      std::shared_ptr<cv_bridge::CvImage> cv_ptr = std::make_shared<cv_bridge::CvImage>(header, "bgr8", ir_publish_);
      pub_ir_->publish(*cv_ptr->toImageMsg());
    }
  }

  bool init_sdk_and_login()
  {
    if (password_.empty()) {
      RCLCPP_ERROR(get_logger(), "Parameter 'password' is empty.");
      return false;
    }
    NET_DVR_Init();
    sdk_initialized_ = true;
    NET_DVR_USER_LOGIN_INFO login_info = {};
    NET_DVR_DEVICEINFO_V40 device_info = {};
    login_info.bUseAsynLogin = 0;
    login_info.wPort = static_cast<WORD>(port_);
    copy_string_to_field(login_info.sDeviceAddress, device_ip_, sizeof(login_info.sDeviceAddress));
    copy_string_to_field(login_info.sUserName, username_, NAME_LEN);
    copy_string_to_field(login_info.sPassword, password_, NAME_LEN);
    user_id_ = NET_DVR_Login_V40(&login_info, &device_info);
    if (user_id_ < 0) {
      RCLCPP_ERROR(get_logger(), "Login failed, error %lu", NET_DVR_GetLastError());
      NET_DVR_Cleanup();
      sdk_initialized_ = false;
      return false;
    }
    return true;
  }

  template<size_t N>
  void copy_string_to_field(char (&field)[N], const std::string& s, size_t max_len)
  {
    size_t len = std::min(s.size(), std::min(max_len, N - 1));
    std::memcpy(field, s.c_str(), len);
    field[len] = '\0';
  }

  bool fetch_temperature_info(LONG channel, float& temp_c, float& humidity)
  {
    if (user_id_ < 0) return false;
    NET_DVR_TEMP_HUMI_INFO info = {};
    info.dwSize = sizeof(NET_DVR_TEMP_HUMI_INFO);
    DWORD ret = 0;
    if (!NET_DVR_GetDVRConfig(user_id_, NET_DVR_GET_TEMP_HUMI_INFO, channel,
                              &info, sizeof(info), &ret)) {
      return false;
    }
    temp_c = info.fTemperature;
    humidity = info.fHumidity;
    return true;
  }

  bool fetch_manual_therm(LONG channel, float& max_t, float& min_t, float& avg_t)
  {
    if (user_id_ < 0) return false;
    NET_SDK_MANUAL_THERMOMETRY therm = {};
    therm.dwSize = sizeof(NET_SDK_MANUAL_THERMOMETRY);
    therm.dwChannel = static_cast<DWORD>(channel);
    DWORD ret = 0;
    if (!NET_DVR_GetDVRConfig(user_id_, NET_DVR_GET_MANUALTHERM_INFO, channel,
                              &therm, sizeof(therm), &ret)) {
      return false;
    }
    const NET_SDK_MANUALTHERM_RULE& r = therm.struRuleInfo;
    max_t = r.struRegionTherm.fMaxTemperature;
    min_t = r.struRegionTherm.fMinTemperature;
    avg_t = r.struRegionTherm.fAverageTemperature;
    if (r.byRuleID == 0 || !r.byEnable) return false;
    return true;
  }

  bool fetch_realtime_therm(LONG channel, float& min_t, float& max_t)
  {
    if (user_id_ < 0) return false;
    NET_DVR_THERMOMETRY_UPLOAD upload = {};
    upload.dwSize = sizeof(NET_DVR_THERMOMETRY_UPLOAD);
    DWORD ret = 0;
    if (!NET_DVR_GetDVRConfig(user_id_, NET_DVR_GET_REALTIME_THERMOMETRY,
                              channel, &upload, sizeof(upload), &ret)) {
      return false;
    }
    min_t = upload.fLowestPointTemperature;
    max_t = upload.fHighestPointTemperature;
    return true;
  }

  static size_t curl_write_cb(char* ptr, size_t size, size_t nmemb, void* userdata)
  {
    size_t total = size * nmemb;
    static_cast<std::string*>(userdata)->append(ptr, total);
    return total;
  }

  bool fetch_temperature_isapi(float& min_t, float& max_t, float& avg_t)
  {
    std::string url = "http://" + device_ip_ + "/ISAPI/Thermal/channels/" +
                      std::to_string(channel_ir_) + "/thermometry/1/rulesTemperatureInfo?format=json";
    std::string response;
    CURL* curl = curl_easy_init();
    if (!curl) return false;
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPAUTH, CURLAUTH_DIGEST);
    curl_easy_setopt(curl, CURLOPT_USERPWD, (username_ + ":" + password_).c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, &HikvisionCameraNode::curl_write_cb);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 5L);
    CURLcode res = curl_easy_perform(curl);
    curl_easy_cleanup(curl);
    if (res != CURLE_OK || response.empty()) return false;
    const char* p = response.c_str();
    auto find_num = [&p](const char* key) -> double {
      const char* q = strstr(p, key);
      if (!q) return -999.0;
      q = strchr(q, ':');
      if (!q) return -999.0;
      return std::strtod(q + 1, nullptr);
    };
    max_t = static_cast<float>(find_num("maxTemperature"));
    min_t = static_cast<float>(find_num("minTemperature"));
    avg_t = static_cast<float>(find_num("averageTemperature"));
    return (min_t > -998.0f && max_t > -998.0f && avg_t > -998.0f);
  }

  void fetch_and_cache_temperature()
  {
    const double kNa = -999.0;
    double min_t = kNa, max_t = kNa, avg_t = kNa, env_temp = kNa, humidity = kNa;
    LONG ch_ir = static_cast<LONG>(channel_ir_);
    LONG ch1 = 1;

    float f_min = 0.f, f_max = 0.f, f_avg = 0.f;
    if (fetch_realtime_therm(ch_ir, f_min, f_max)) {
      min_t = static_cast<double>(f_min);
      max_t = static_cast<double>(f_max);
      avg_t = (min_t + max_t) * 0.5;
    } else if (fetch_realtime_therm(ch1, f_min, f_max)) {
      min_t = static_cast<double>(f_min);
      max_t = static_cast<double>(f_max);
      avg_t = (min_t + max_t) * 0.5;
    } else if (fetch_manual_therm(ch_ir, f_max, f_min, f_avg)) {
      min_t = static_cast<double>(f_min);
      max_t = static_cast<double>(f_max);
      avg_t = static_cast<double>(f_avg);
    } else if (fetch_manual_therm(ch1, f_max, f_min, f_avg)) {
      min_t = static_cast<double>(f_min);
      max_t = static_cast<double>(f_max);
      avg_t = static_cast<double>(f_avg);
    }

    float f_env = 0.f, f_hum = 0.f;
    if (user_id_ >= 0) {
      if (fetch_temperature_info(ch_ir, f_env, f_hum)) {
        env_temp = static_cast<double>(f_env);
        humidity = static_cast<double>(f_hum);
      } else if (fetch_temperature_info(ch1, f_env, f_hum)) {
        env_temp = static_cast<double>(f_env);
        humidity = static_cast<double>(f_hum);
      }
    }

    if (min_t == kNa && max_t == kNa && avg_t == kNa) {
      float fm = 0.f, fx = 0.f, fa = 0.f;
      if (fetch_temperature_isapi(fm, fx, fa)) {
        min_t = static_cast<double>(fm);
        max_t = static_cast<double>(fx);
        avg_t = static_cast<double>(fa);
      }
    }

    if (min_t == kNa && max_t == kNa && env_temp == kNa) {
      if (!temp_fail_logged_) {
        temp_fail_logged_ = true;
        RCLCPP_WARN(get_logger(),
          "[temperature_info] All temp APIs failed (SDK err %lu). Temperature will be updated via ISAPI in background. Set publish_temperature_topic: false to disable.",
          static_cast<unsigned long>(NET_DVR_GetLastError()));
      }
    }

    std::lock_guard<std::mutex> lock(temp_mutex_);
    cached_min_t_ = min_t;
    cached_max_t_ = max_t;
    cached_avg_t_ = avg_t;
    cached_env_temp_ = env_temp;
    cached_humidity_ = humidity;
  }

  void temperature_background_loop()
  {
    while (rclcpp::ok() && running_temp_) {
      fetch_and_cache_temperature();
      for (double t = 0.0; t < temperature_poll_interval_ && rclcpp::ok() && running_temp_; t += 0.05)
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  void publish_temperature()
  {
    if (!pub_temperature_) return;
    const double kNa = -999.0;
    double min_t, max_t, avg_t, env_temp, humidity;
    {
      std::lock_guard<std::mutex> lock(temp_mutex_);
      min_t = cached_min_t_;
      max_t = cached_max_t_;
      avg_t = cached_avg_t_;
      env_temp = cached_env_temp_;
      humidity = cached_humidity_;
    }
    std_msgs::msg::Float64MultiArray msg;
    msg.data.clear();
    msg.layout.dim.clear();
    auto add_dim = [&msg](const char* label, double value, double na) {
      if (value == na) return;
      std_msgs::msg::MultiArrayDimension d;
      d.label = label;
      d.size = 1;
      d.stride = 1;
      msg.layout.dim.push_back(d);
      msg.data.push_back(value);
    };
    add_dim("min_temperature", min_t, kNa);
    add_dim("max_temperature", max_t, kNa);
    add_dim("avg_temperature", avg_t, kNa);
    add_dim("env_temperature", env_temp, kNa);
    add_dim("humidity", humidity, kNa);
    if (!msg.data.empty())
      pub_temperature_->publish(msg);
  }

  void start_sdk_capture()
  {
    double period_sec = 1.0 / frame_rate_;
    timer_ = create_wall_timer(
      std::chrono::duration<double>(period_sec),
      std::bind(&HikvisionCameraNode::timer_callback, this));
    RCLCPP_INFO(get_logger(), "SDK JPEG capture mode, %.1f Hz", frame_rate_);
  }

  void timer_callback()
  {
    if (user_id_ < 0) return;
    rclcpp::Time stamp = now();
    capture_and_publish(channel_visible_, pub_camera_, camera_frame_id_, stamp);
    capture_and_publish(channel_ir_, pub_ir_, ir_frame_id_, stamp);
  }

  void capture_and_publish(
    int channel,
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub,
    const std::string& frame_id,
    const rclcpp::Time& stamp)
  {
    NET_DVR_JPEGPARA jpeg_para = {};
    jpeg_para.wPicQuality = 2;
    jpeg_para.wPicSize = 0;
    char tmp_path[64];
    snprintf(tmp_path, sizeof(tmp_path), "/tmp/hik_capture_ch%d_%d.jpg", channel, getpid());
    if (!NET_DVR_CaptureJPEGPicture(user_id_, static_cast<LONG>(channel), &jpeg_para, tmp_path)) return;
    cv::Mat img = cv::imread(tmp_path);
    std::remove(tmp_path);
    if (img.empty()) return;
    if (channel == static_cast<int>(channel_ir_) && publish_temperature_topic_) {
      publish_temperature();
    }
    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = frame_id;
    std::shared_ptr<cv_bridge::CvImage> cv_ptr = std::make_shared<cv_bridge::CvImage>(header, "bgr8", img);
    pub->publish(*cv_ptr->toImageMsg());
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_camera_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_ir_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_temperature_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::thread rtsp_thread_;
  std::thread rtsp_ir_thread_;
  std::thread temperature_thread_;
  std::atomic<bool> running_;
  std::atomic<bool> running_temp_{false};

  cv::Mat camera_buffer_;
  cv::Mat ir_buffer_;
  cv::Mat camera_publish_;
  cv::Mat ir_publish_;
  std::mutex camera_mutex_;
  std::mutex ir_mutex_;

  std::string device_ip_;
  int port_;
  std::string username_;
  std::string password_;
  int channel_visible_;
  int channel_ir_;
  double frame_rate_;
  std::string camera_frame_id_;
  std::string ir_frame_id_;
  bool use_rtsp_;
  bool low_latency_;
  bool use_sub_stream_;
  bool camera_use_ffmpeg_;
  std::string rtsp_camera_url_;
  std::string rtsp_ir_url_;
  bool publish_temperature_topic_;
  double temperature_poll_interval_;

  std::mutex temp_mutex_;
  double cached_min_t_{-999.0};
  double cached_max_t_{-999.0};
  double cached_avg_t_{-999.0};
  double cached_env_temp_{-999.0};
  double cached_humidity_{-999.0};

  LONG user_id_;
  bool sdk_initialized_;
  bool temp_fail_logged_{false};
};

}  // namespace hikvision_camera_ros2

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hikvision_camera_ros2::HikvisionCameraNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
