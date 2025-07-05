#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <cjson/cJSON.h>

#include "sipeed_tof_ms_a010/serial_port.hpp"
#include "sipeed_tof_ms_a010/frame_parser.hpp"
#include "sipeed_tof_ms_a010/color_maps.hpp"

using namespace std::chrono_literals;

namespace sipeed_tof
{

class TofNode : public rclcpp::Node
{
public:
  TofNode() : Node("sipeed_tof_node")
  {
    // Declare parameters
    this->declare_parameter<std::string>("device", "/dev/ttyUSB0");
    this->declare_parameter<std::string>("frame_id", "tof_camera");
    this->declare_parameter<int>("timer_period_ms", 30);

    // Get parameters
    device_path_ = this->get_parameter("device").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    auto timer_period = std::chrono::milliseconds(this->get_parameter("timer_period_ms").as_int());

    // Initialize publishers
    depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("depth", 10);
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 10);

    // Initialize camera
    if (!initialize_camera()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera. Shutting down.");
      rclcpp::shutdown();
      return;
    }

    // Start the main timer
    timer_ = this->create_wall_timer(timer_period, std::bind(&TofNode::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "Sipeed ToF camera node started successfully.");
  }

private:
  bool initialize_camera()
  {
    try {
      serial_port_ = std::make_unique<SerialPort>(device_path_);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error opening serial port %s: %s", device_path_.c_str(), e.what());
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Configuring camera at %s...", device_path_.c_str());

    // Sequence of AT commands to initialize the camera
    if (!serial_port_->send_command("AT\r", "OK\r\n")) {
      RCLCPP_ERROR(this->get_logger(), "Camera not responding to AT command.");
      return false;
    }
    serial_port_->send_command("AT+ISP=0\r", "OK\r\n");
    serial_port_->send_command("AT+DISP=1\r", "OK\r\n");
    serial_port_->send_command("AT+ISP=1\r", "OK\r\n");

    if (!get_camera_coefficients()) {
      return false;
    }

    if (!serial_port_->send_command("AT+DISP=3\r", "OK\r\n")) {
      RCLCPP_ERROR(this->get_logger(), "Failed to start data stream (AT+DISP=3).");
      return false;
    }

    return true;
  }

  bool get_camera_coefficients()
  {
    serial_port_->write_string("AT+COEFF?\r");
    std::string response;
    // Read until we get the JSON data, which starts with '{'
    for (int i = 0; i < 20; ++i) { // 2-second timeout
        response += serial_port_->read_string();
        if (response.find('{') != std::string::npos) break;
        std::this_thread::sleep_for(100ms);
    }

    size_t json_start = response.find('{');
    size_t json_end = response.rfind('}');
    if (json_start == std::string::npos || json_end == std::string::npos) {
        RCLCPP_ERROR(this->get_logger(), "Did not receive camera coefficients in JSON format.");
        return false;
    }
    std::string json_str = response.substr(json_start, json_end - json_start + 1);

    std::unique_ptr<cJSON, decltype(&cJSON_Delete)> cparms(
        cJSON_Parse(json_str.c_str()), &cJSON_Delete);

    if (!cparms) {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse camera coefficients JSON.");
      return false;
    }

    fx_ = cJSON_GetNumberValue(cJSON_GetObjectItem(cparms.get(), "fx")) / 262144.0f;
    fy_ = cJSON_GetNumberValue(cJSON_GetObjectItem(cparms.get(), "fy")) / 262144.0f;
    u0_ = cJSON_GetNumberValue(cJSON_GetObjectItem(cparms.get(), "u0")) / 262144.0f;
    v0_ = cJSON_GetNumberValue(cJSON_GetObjectItem(cparms.get(), "v0")) / 262144.0f;

    RCLCPP_INFO(this->get_logger(), "Camera Coefficients: fx=%.2f, fy=%.2f, u0=%.2f, v0=%.2f", fx_, fy_, u0_, v0_);
    return true;
  }

  void timer_callback()
  {
    frame_parser_.add_data(serial_port_->read_string());
    
    while (auto frame = frame_parser_.parse()) {
      auto header = create_header();
      publish_depth_image(*frame, header);
      publish_point_cloud(*frame, header);
    }
  }

  std_msgs::msg::Header create_header()
  {
    std_msgs::msg::Header header;
    header.stamp = this->get_clock()->now();
    header.frame_id = frame_id_;
    return header;
  }

  void publish_depth_image(const frame_t& frame, const std_msgs::msg::Header& header)
  {
    uint8_t rows = frame.frame_head.resolution_rows;
    uint8_t cols = frame.frame_head.resolution_cols;
    cv::Mat depth_map(rows, cols, CV_8UC1, const_cast<uint8_t*>(frame.payload));
    
    auto msg = cv_bridge::CvImage(header, "mono8", depth_map).toImageMsg();
    depth_pub_->publish(*msg);
  }

  void publish_point_cloud(const frame_t& frame, const std_msgs::msg::Header& header)
  {
    uint8_t rows = frame.frame_head.resolution_rows;
    uint8_t cols = frame.frame_head.resolution_cols;
    const uint8_t* depth_data = frame.payload;

    auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    msg->header = header;
    msg->height = rows;
    msg->width = cols;
    msg->is_bigendian = false;
    msg->is_dense = false;

    sensor_msgs::msg::PointField field;
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;

    field.name = "x";
    field.offset = 0;
    msg->fields.push_back(field);

    field.name = "y";
    field.offset = 4;
    msg->fields.push_back(field);

    field.name = "z";
    field.offset = 8;
    msg->fields.push_back(field);

    field.name = "rgb";
    field.offset = 12;
    field.datatype = sensor_msgs::msg::PointField::UINT32; // Packed RGB
    msg->fields.push_back(field);

    msg->point_step = 16;
    msg->row_step = msg->point_step * cols;
    msg->data.resize(msg->row_step * rows);

    uint8_t* ptr = msg->data.data();
    for (uint8_t j = 0; j < rows; ++j) {
      for (uint8_t i = 0; i < cols; ++i) {
        float z = static_cast<float>(depth_data[j * cols + i]) / 100.0f; // depth in meters
        float x = (static_cast<float>(i) - u0_) * z / fx_;
        float y = (static_cast<float>(j) - v0_) * z / fy_;
        
        *reinterpret_cast<float*>(ptr + 0) = x;
        *reinterpret_cast<float*>(ptr + 4) = y;
        *reinterpret_cast<float*>(ptr + 8) = z;

        const uint8_t* color = COLOR_LUT_JET[depth_data[j * cols + i]];
        uint32_t rgb = (static_cast<uint32_t>(color[0]) << 16) |
                       (static_cast<uint32_t>(color[1]) << 8)  |
                       (static_cast<uint32_t>(color[2]));
        *reinterpret_cast<uint32_t*>(ptr + 12) = rgb;
        
        ptr += msg->point_step;
      }
    }
    cloud_pub_->publish(std::move(msg));
  }

  // Member variables
  std::unique_ptr<SerialPort> serial_port_;
  FrameParser frame_parser_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

  std::string device_path_;
  std::string frame_id_;
  float fx_, fy_, u0_, v0_;
};

} // namespace sipeed_tof

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sipeed_tof::TofNode>());
  rclcpp::shutdown();
  return 0;
}