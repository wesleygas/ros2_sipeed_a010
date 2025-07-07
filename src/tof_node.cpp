#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
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
  ~TofNode()
  {
    // 1. Signal the reading thread to stop.
    is_running_ = false;

    // 2. Send the command to the camera to stop streaming.
    if (serial_port_ && serial_port_->is_open()) {
      RCLCPP_INFO(this->get_logger(), "Stopping camera data stream...");
      // We don't need to wait for a response, just send the command.
      serial_port_->write_string("AT+DISP=0\r");
      // Give it a moment to process
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 3. Join the reading thread to ensure it has exited cleanly.
    if (read_thread_.joinable()) {
      read_thread_.join();
    }

    // 4. The SerialPort destructor will be called automatically, closing the port.
    RCLCPP_INFO(this->get_logger(), "TofNode shut down cleanly.");
  }

  TofNode() : Node("sipeed_tof_node")
  {
    // Declare parameters
    this->declare_parameter<std::string>("device", "/dev/ttyUSB0");
    this->declare_parameter<std::string>("frame_id", "tof_camera");
    this->declare_parameter<int>("timer_period_ms", 30);
    this->declare_parameter<int>("binning", 1);
    this->declare_parameter<int>("display_mode", 2);
    this->declare_parameter<int>("fps", 10);
    this->declare_parameter<int>("quantization_unit", 0);

    // Get parameters
    device_path_ = this->get_parameter("device").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    auto timer_period = std::chrono::milliseconds(this->get_parameter("timer_period_ms").as_int());

    // Initialize publishers
    depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("depth", 10);
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 10);
    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("depth/camera_info", 10);

    // Initialize camera
    if (!initialize_camera()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera. Shutting down.");
      rclcpp::shutdown();
      return;
    }

    // *** START THE DEDICATED READING THREAD ***
    read_thread_ = std::thread(&TofNode::serial_read_loop, this);

    // The timer now only processes data, it doesn't read it.
    timer_ = this->create_wall_timer(timer_period, std::bind(&TofNode::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "Sipeed ToF camera node started successfully.");
  }

private:
  // New method to be run in the dedicated thread
  void serial_read_loop()
  {
    while (is_running_) {
      // This call will block, but it's in its own thread, so it's safe.
      std::string new_data = serial_port_->read_string();

      if (!new_data.empty()) {
        // Lock the mutex before accessing the shared frame_parser_
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        frame_parser_.add_data(new_data);
      }
    }
  }


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
    if (!apply_camera_settings()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to apply one or more camera settings.");
      return false;
    }
    if (!get_camera_coefficients()) {
      return false;
    }

    // Finally, start the data stream
    RCLCPP_INFO(this->get_logger(), "Starting data stream...");
    int display_mode = this->get_parameter("display_mode").as_int();
    if (!serial_port_->send_command("AT+DISP=" + std::to_string(display_mode) + "\r", "OK\r\n")) {
      return false;
    }
    return true;
  }
  bool apply_camera_settings()
  {
    RCLCPP_INFO(this->get_logger(), "Applying camera settings...");

    // Set Binning
    int binning = this->get_parameter("binning").as_int();
    RCLCPP_INFO(this->get_logger(), "Setting Binning to mode %d.", binning);
    if (!serial_port_->send_command("AT+BINN=" + std::to_string(binning) + "\r", "OK\r\n")) {
      return false;
    }

    // Set FPS
    int fps = this->get_parameter("fps").as_int();
    RCLCPP_INFO(this->get_logger(), "Setting FPS to %d.", fps);
    if (!serial_port_->send_command("AT+FPS=" + std::to_string(fps) + "\r", "OK\r\n")) {
      return false;
    }

    // Set Quantization Unit
    int unit = this->get_parameter("quantization_unit").as_int();
    RCLCPP_INFO(this->get_logger(), "Setting Quantization Unit to %d.", unit);
    if (!serial_port_->send_command("AT+UNIT=" + std::to_string(unit) + "\r", "OK\r\n")) {
      return false;
    }
    
    // The docs mention a 1-2 second wait after starting the ISP
    std::this_thread::sleep_for(std::chrono::seconds(1));

    return true;
  }
  void populate_camera_info_msg()
  {
      // This function is called once during initialization.
      // It fills the CameraInfo message with data that does not change.
      
      // For now, we assume 100x100 resolution. This could be made dynamic
      // if the binning parameter changes resolution.
      int binning = this->get_parameter("binning").as_int();
      int image_width = 100/binning;
      int image_height = 100/binning;

      camera_info_msg_.header.frame_id = frame_id_; // Will be updated with timestamp later
      camera_info_msg_.width = image_width;
      camera_info_msg_.height = image_height;

      // Camera Intrinsic Matrix K = [fx 0 cx; 0 fy cy; 0 0 1]
      camera_info_msg_.k[0] = fx_; // fx
      camera_info_msg_.k[1] = 0.0;
      camera_info_msg_.k[2] = u0_; // cx
      camera_info_msg_.k[3] = 0.0;
      camera_info_msg_.k[4] = fy_; // fy
      camera_info_msg_.k[5] = v0_; // cy
      camera_info_msg_.k[6] = 0.0;
      camera_info_msg_.k[7] = 0.0;
      camera_info_msg_.k[8] = 1.0;

      // Rectification Matrix R (identity for a single camera)
      camera_info_msg_.r[0] = 1.0;
      camera_info_msg_.r[4] = 1.0;
      camera_info_msg_.r[8] = 1.0;

      // Projection Matrix P = [fx 0 cx Tx; 0 fy cy Ty; 0 0 1 0]
      // For a monocular camera, Tx = Ty = 0.
      camera_info_msg_.p[0] = fx_;
      camera_info_msg_.p[1] = 0.0;
      camera_info_msg_.p[2] = u0_;
      camera_info_msg_.p[3] = 0.0;
      camera_info_msg_.p[4] = 0.0;
      camera_info_msg_.p[5] = fy_;
      camera_info_msg_.p[6] = v0_;
      camera_info_msg_.p[7] = 0.0;
      camera_info_msg_.p[8] = 0.0;
      camera_info_msg_.p[9] = 0.0;
      camera_info_msg_.p[10] = 1.0;
      camera_info_msg_.p[11] = 0.0;

      // No distortion from this camera
      camera_info_msg_.distortion_model = "plumb_bob";
      camera_info_msg_.d.clear(); // Empty distortion vector
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
    populate_camera_info_msg();
    return true;
  }

  void timer_callback()
  {
    std::string new_data = serial_port_->read_string();

    if (!new_data.empty()) {
      // Log the size of the data chunk we received. This tells us if the port is alive.
      // RCLCPP_INFO(this->get_logger(), "Received %zu bytes from serial port.", new_data.length());
      frame_parser_.add_data(new_data);
    }
    
    while (auto frame = frame_parser_.parse()) {
      // If we get here, it means parsing was successful!
      RCLCPP_DEBUG(this->get_logger(), "Successfully parsed a frame!");
      auto header = create_header();
      camera_info_msg_.header.stamp = header.stamp;
      camera_info_pub_->publish(camera_info_msg_);
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
        float z = pow(static_cast<float>(depth_data[j * cols + i])/5.1, 2) / 1000.0f; // depth in meters
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
  std::thread read_thread_;
  std::atomic<bool> is_running_{true};
  std::mutex buffer_mutex_; // To protect the frame_parser_'s internal buffer
  std::unique_ptr<SerialPort> serial_port_;
  FrameParser frame_parser_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

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