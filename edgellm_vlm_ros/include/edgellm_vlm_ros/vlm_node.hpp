#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace edgellm_vlm_ros
{

struct FrameSnapshot
{
  cv::Mat rgb;
  rclcpp::Time stamp;
  std::string frame_id;
  std::string source_encoding;
  int width = 0;
  int height = 0;
};

struct GenerationConfig
{
  int max_generate_length = 96;
  double temperature = 0.2;
  double top_p = 0.9;
  int top_k = 40;
  int stream_interval = 4;
  bool publish_partials = true;
  std::string system_prompt;
  std::string user_prompt;
};

struct InferenceResult
{
  std::string text;
  std::string finish_reason = "stop";
  double ttft_ms = 0.0;
  double total_ms = 0.0;
  size_t tokens = 0;
};

class InferenceBackend
{
public:
  using PartialCallback = std::function<void(std::string const&)>;

  virtual ~InferenceBackend() = default;
  virtual InferenceResult infer(
    FrameSnapshot const& frame,
    GenerationConfig const& config,
    PartialCallback const& publish_partial,
    std::atomic<bool> const& shutting_down) = 0;
};

class VlmNode : public rclcpp::Node
{
public:
  explicit VlmNode(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());
  ~VlmNode() override;

private:
  void imageCallback(sensor_msgs::msg::Image::ConstSharedPtr msg);
  void timerCallback();
  void handleSetPointMode(
    std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response);
  void runInference(FrameSnapshot frame);

  bool convertImage(sensor_msgs::msg::Image::ConstSharedPtr const& msg, FrameSnapshot& frame);
  void initializeBackend();
  void publishStatus(std::string const& status);
  void publishPartial(std::string const& text);
  void publishResult(
    FrameSnapshot const& frame,
    std::string const& prompt_mode,
    std::string const& text,
    std::string const& finish_reason,
    double ttft_ms,
    double total_ms,
    size_t tokens);

  std::string image_topic_;
  std::string backend_type_;
  std::string engine_dir_;
  std::string multimodal_engine_dir_;
  std::string plugin_path_;
  double fixed_rate_hz_ = 1.0;
  GenerationConfig generation_;
  GenerationConfig point_generation_;
  GenerationConfig primitive_generation_;
  GenerationConfig region_generation_;
  std::string prompt_mode_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr partial_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_point_mode_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex frame_mutex_;
  std::mutex generation_mutex_;
  std::optional<FrameSnapshot> latest_frame_;

  std::atomic<bool> inference_running_{false};
  std::atomic<bool> shutting_down_{false};
  std::thread worker_thread_;
  std::unique_ptr<InferenceBackend> backend_;
};

}  // namespace edgellm_vlm_ros
