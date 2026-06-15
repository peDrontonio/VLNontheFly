#include "edgellm_vlm_ros/vlm_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>

#ifdef EDGELLM_VLM_ENABLE_EDGELLM
#include <cuda_runtime_api.h>

#include "common/tensor.h"
#include "common/trtUtils.h"
#include "runtime/imageUtils.h"
#include "runtime/llmInferenceRuntime.h"
#include "runtime/llmRuntimeUtils.h"
#include "runtime/streaming.h"
#endif

namespace
{

std::string jsonEscape(std::string const& input)
{
  std::ostringstream out;
  for (char c : input) {
    switch (c) {
      case '"':
        out << "\\\"";
        break;
      case '\\':
        out << "\\\\";
        break;
      case '\b':
        out << "\\b";
        break;
      case '\f':
        out << "\\f";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          out << "\\u00";
          constexpr char hex[] = "0123456789abcdef";
          out << hex[(c >> 4) & 0x0f] << hex[c & 0x0f];
        } else {
          out << c;
        }
        break;
    }
  }
  return out.str();
}

std::vector<std::string> splitWords(std::string const& text)
{
  std::istringstream stream(text);
  std::vector<std::string> words;
  std::string word;
  while (stream >> word) {
    words.push_back(word);
  }
  return words;
}

double stampToSec(rclcpp::Time const& stamp)
{
  return static_cast<double>(stamp.nanoseconds()) / 1e9;
}

#ifdef EDGELLM_VLM_ENABLE_EDGELLM
trt_edgellm::rt::imageUtils::ImageData makeImageDataFromRgbMat(cv::Mat const& rgb)
{
  CV_Assert(rgb.type() == CV_8UC3);
  CV_Assert(rgb.isContinuous());

  trt_edgellm::rt::Tensor tensor(
    trt_edgellm::rt::Coords{
      static_cast<int64_t>(rgb.rows),
      static_cast<int64_t>(rgb.cols),
      3},
    trt_edgellm::rt::DeviceType::kCPU,
    nvinfer1::DataType::kUINT8,
    "edgellm_vlm_ros::rgb_image");

  std::memcpy(
    tensor.dataPointer<unsigned char>(),
    rgb.data,
    static_cast<size_t>(rgb.rows) * static_cast<size_t>(rgb.cols) * 3U);

  return trt_edgellm::rt::imageUtils::ImageData(std::move(tensor));
}
#endif

}  // namespace

namespace edgellm_vlm_ros
{

class MockBackend final : public InferenceBackend
{
public:
  InferenceResult infer(
    FrameSnapshot const& frame,
    GenerationConfig const& config,
    PartialCallback const& publish_partial,
    std::atomic<bool> const& shutting_down) override
  {
    auto const started = std::chrono::steady_clock::now();
    InferenceResult result;
    auto words = splitWords(generateText(frame));
    if (config.max_generate_length > 0) {
      words.resize(std::min(words.size(), static_cast<size_t>(config.max_generate_length)));
    }

    std::string chunk;
    for (size_t i = 0; i < words.size() && !shutting_down.load(); ++i) {
      if (!chunk.empty()) {
        chunk += " ";
      }
      chunk += words[i];
      ++result.tokens;

      bool const should_flush =
        ((i + 1) % static_cast<size_t>(config.stream_interval) == 0) || (i + 1 == words.size());
      if (!should_flush) {
        continue;
      }

      if (result.text.empty()) {
        auto const first_token = std::chrono::steady_clock::now();
        result.ttft_ms = std::chrono::duration<double, std::milli>(first_token - started).count();
      } else {
        result.text += " ";
      }
      result.text += chunk;
      if (config.publish_partials) {
        publish_partial(chunk);
      }
      chunk.clear();
      std::this_thread::sleep_for(std::chrono::milliseconds(80));
    }

    auto const finished = std::chrono::steady_clock::now();
    result.total_ms = std::chrono::duration<double, std::milli>(finished - started).count();
    result.finish_reason = shutting_down.load() ? "cancelled" : "stop";
    return result;
  }

private:
  static std::string generateText(FrameSnapshot const& frame)
  {
    std::ostringstream text;
    text << "Mock VLM observation from " << frame.width << " by " << frame.height
         << " RGB frame. Source frame is " << frame.frame_id
         << ". Set backend_type to edgellm to use TensorRT Edge-LLM.";
    return text.str();
  }
};

#ifdef EDGELLM_VLM_ENABLE_EDGELLM
class EdgeLlmBackend final : public InferenceBackend
{
public:
  EdgeLlmBackend(
    std::string const& engine_dir,
    std::string const& multimodal_engine_dir,
    std::string const& plugin_path)
  {
    if (!plugin_path.empty()) {
      setenv("EDGELLM_PLUGIN_PATH", plugin_path.c_str(), 1);
    }

    plugin_handle_ = trt_edgellm::loadEdgellmPluginLib();
    if (!plugin_handle_) {
      throw std::runtime_error("failed to load TensorRT Edge-LLM plugin");
    }

    cudaError_t const stream_status = cudaStreamCreate(&stream_);
    if (stream_status != cudaSuccess) {
      throw std::runtime_error(
        std::string("cudaStreamCreate failed: ") + cudaGetErrorString(stream_status));
    }

    std::unordered_map<std::string, std::string> lora_weights;
    runtime_ = std::make_unique<trt_edgellm::rt::LLMInferenceRuntime>(
      engine_dir, multimodal_engine_dir, lora_weights, stream_);
    runtime_->captureDecodingCUDAGraph(stream_);
  }

  ~EdgeLlmBackend() override
  {
    runtime_.reset();
    if (stream_ != nullptr) {
      cudaStreamDestroy(stream_);
      stream_ = nullptr;
    }
  }

  InferenceResult infer(
    FrameSnapshot const& frame,
    GenerationConfig const& config,
    PartialCallback const& publish_partial,
    std::atomic<bool> const& shutting_down) override
  {
    using namespace trt_edgellm;

    rt::LLMGenerationRequest request;
    request.temperature = static_cast<float>(config.temperature);
    request.topP = static_cast<float>(config.top_p);
    request.topK = config.top_k;
    request.maxGenerateLength = config.max_generate_length;
    request.applyChatTemplate = true;
    request.addGenerationPrompt = true;
    request.enableThinking = false;
    request.disableSpecDecode = true;

    rt::Message system;
    system.role = "system";
    system.contents.push_back({"text", config.system_prompt});

    rt::Message user;
    user.role = "user";
    user.contents.push_back({"image", ""});
    user.contents.push_back({"text", config.user_prompt});

    rt::LLMGenerationRequest::Request slot;
    slot.messages = {std::move(system), std::move(user)};
    slot.imageBuffers.push_back(makeImageDataFromRgbMat(frame.rgb));
    request.requests.push_back(std::move(slot));

    auto channel = rt::StreamChannel::create();
    channel->setStreamInterval(config.stream_interval);
    channel->setSkipSpecialTokens(true);
    request.streamChannels.push_back(channel);

    rt::LLMGenerationResponse response;
    std::atomic<bool> worker_ok{false};
    std::exception_ptr worker_exception;

    InferenceResult result;
    rt::FinishReason finish_reason = rt::FinishReason::kNotFinished;
    auto const started = std::chrono::steady_clock::now();
    std::thread worker([&] {
      try {
        worker_ok = runtime_->handleRequest(request, response, stream_);
      } catch (...) {
        worker_exception = std::current_exception();
      }
    });

    bool first_chunk = true;
    channel->consume([&](rt::StreamChunk&& chunk) {
      if (first_chunk) {
        auto const first = std::chrono::steady_clock::now();
        result.ttft_ms = std::chrono::duration<double, std::milli>(first - started).count();
        first_chunk = false;
      }

      result.tokens += chunk.tokenIds.size();
      if (!chunk.text.empty()) {
        result.text += chunk.text;
        if (config.publish_partials) {
          publish_partial(chunk.text);
        }
      }
      if (chunk.finished) {
        finish_reason = chunk.reason;
      }
      if (shutting_down.load()) {
        channel->cancel();
      }
    });

    worker.join();

    if (worker_exception) {
      std::rethrow_exception(worker_exception);
    }
    if (!worker_ok.load()) {
      throw std::runtime_error("TensorRT Edge-LLM handleRequest returned false");
    }

    if (result.text.empty() && !response.outputTexts.empty()) {
      result.text = response.outputTexts.front();
    }
    if (result.tokens == 0 && !response.outputIds.empty()) {
      result.tokens = response.outputIds.front().size();
    }
    if (finish_reason == rt::FinishReason::kNotFinished && channel->isFinished()) {
      finish_reason = channel->getReason();
    }
    if (finish_reason == rt::FinishReason::kNotFinished && !response.finishReasons.empty()) {
      finish_reason = response.finishReasons.front();
    }

    auto const finished = std::chrono::steady_clock::now();
    result.total_ms = std::chrono::duration<double, std::milli>(finished - started).count();
    result.finish_reason = rt::finishReasonName(finish_reason);
    return result;
  }

private:
  cudaStream_t stream_{};
  std::unique_ptr<void, trt_edgellm::DlDeleter> plugin_handle_;
  std::unique_ptr<trt_edgellm::rt::LLMInferenceRuntime> runtime_;
};
#endif

VlmNode::VlmNode(rclcpp::NodeOptions const& options)
: Node("edgellm_vlm_node", options)
{
  engine_dir_ = declare_parameter<std::string>(
    "engine_dir", "/home/orin/imav/tensorrt-edgellm-workspace/Qwen3.5-2B/int4_awq/engines/llm");
  multimodal_engine_dir_ = declare_parameter<std::string>(
    "multimodal_engine_dir",
    "/home/orin/imav/tensorrt-edgellm-workspace/Qwen3.5-2B/int4_awq/engines/visual");
  plugin_path_ = declare_parameter<std::string>(
    "plugin_path", "/home/orin/TensorRT-Edge-LLM/build/libNvInfer_edgellm_plugin.so");
  image_topic_ = declare_parameter<std::string>("image_topic", "/camera/camera/color/image_raw");
  fixed_rate_hz_ = declare_parameter<double>("fixed_rate_hz", 1.0);
  backend_type_ = declare_parameter<std::string>("backend_type", "edgellm");
  generation_.max_generate_length = declare_parameter<int>("max_generate_length", 96);
  generation_.temperature = declare_parameter<double>("temperature", 0.2);
  generation_.top_p = declare_parameter<double>("top_p", 0.9);
  generation_.top_k = declare_parameter<int>("top_k", 40);
  generation_.stream_interval = declare_parameter<int>("stream_interval", 4);
  generation_.publish_partials = declare_parameter<bool>("publish_partials", true);

  std::string const point_system_prompt = declare_parameter<std::string>(
    "point_system_prompt",
    "You are a drone image-goal selector. Return exactly one minified JSON object on one line "
    "with keys u, v, and confidence. u and v are pixel coordinates in the input RGB image. "
    "Choose one safe reachable point on the ground or open corridor that the drone can move "
    "toward while staying at the same altitude. Do not include markdown, prose, whitespace, "
    "or extra keys. Prefer the center of free space when uncertain.");
  std::string const point_user_prompt = declare_parameter<std::string>(
    "point_user_prompt", "Select one safe navigation point in the current RGB image. Return JSON only.");
  std::string const primitive_system_prompt = declare_parameter<std::string>(
    "primitive_system_prompt",
    "You are a drone altitude primitive selector. Return exactly one minified JSON object on one "
    "line with keys primitive, distance_m, and confidence. Do not include reason, markdown, prose, "
    "or whitespace. Allowed primitives are HOLD, UP, and DOWN. Allowed distance_m values are 0.0 "
    "for HOLD and 0.5 or 1.0 for movement. Prefer HOLD when uncertain.");
  std::string const primitive_user_prompt = declare_parameter<std::string>(
    "primitive_user_prompt",
    "Choose one safe altitude primitive from the current camera image. Return JSON only.");
  std::string const region_system_prompt = declare_parameter<std::string>(
    "region_system_prompt",
    "You are a drone navigation assistant. The image is divided into a 3x3 grid of regions named "
    "TOP-LEFT, TOP-CENTER, TOP-RIGHT, MIDDLE-LEFT, CENTER, MIDDLE-RIGHT, BOTTOM-LEFT, "
    "BOTTOM-CENTER, BOTTOM-RIGHT. Choose the ONE region that contains the most open, safe free "
    "space to move toward. Return exactly one minified JSON object on one line in this format: "
    "{\"region\":\"CENTER\",\"confidence\":0.8}. region MUST be exactly one of the nine names. "
    "confidence is a decimal between 0 and 1 with exactly ONE digit after the decimal point. "
    "Output the closing brace and then stop. Do not include markdown, prose, whitespace, or "
    "extra keys.");
  std::string const region_user_prompt = declare_parameter<std::string>(
    "region_user_prompt",
    "Which grid region has the most open space to move toward? Return JSON only.");
  // When non-empty, region mode looks for this object instead of generic free
  // space. Finding a salient named object grounds far better on the small VLM
  // than judging open space, which tends to collapse to CENTER.
  std::string const target_object = declare_parameter<std::string>("target_object", "");
  prompt_mode_ = declare_parameter<std::string>("prompt_mode", "point");

  std::string region_system = region_system_prompt;
  std::string region_user = region_user_prompt;
  if (!target_object.empty()) {
    region_system =
      "You are a drone navigation assistant. The image is divided into a 3x3 grid of regions "
      "named TOP-LEFT, TOP-CENTER, TOP-RIGHT, MIDDLE-LEFT, CENTER, MIDDLE-RIGHT, BOTTOM-LEFT, "
      "BOTTOM-CENTER, BOTTOM-RIGHT. Report which ONE region contains the " + target_object +
      ". Return exactly one minified JSON object on one line in this format: "
      "{\"region\":\"CENTER\",\"confidence\":0.8}. region MUST be exactly one of the nine names, "
      "or \"NONE\" if the " + target_object + " is not visible in the image. confidence is a "
      "decimal between 0 and 1 with exactly ONE digit after the decimal point: high when you "
      "clearly see the " + target_object + ", and 0.0 when it is not visible. Output the closing "
      "brace and then stop. Do not include markdown, prose, whitespace, or extra keys.";
    region_user = "Which grid region contains the " + target_object + "? Return JSON only.";
  }

  point_generation_ = generation_;
  point_generation_.system_prompt = point_system_prompt;
  point_generation_.user_prompt = point_user_prompt;
  primitive_generation_ = generation_;
  primitive_generation_.system_prompt = primitive_system_prompt;
  primitive_generation_.user_prompt = primitive_user_prompt;
  region_generation_ = generation_;
  region_generation_.system_prompt = region_system;
  region_generation_.user_prompt = region_user;

  if (prompt_mode_ == "point") {
    generation_ = point_generation_;
  } else if (prompt_mode_ == "primitive") {
    generation_ = primitive_generation_;
  } else if (prompt_mode_ == "region") {
    generation_ = region_generation_;
  } else {
    throw std::invalid_argument("prompt_mode must be 'point', 'primitive', or 'region'");
  }

  if (fixed_rate_hz_ <= 0.0 || !std::isfinite(fixed_rate_hz_)) {
    throw std::invalid_argument("fixed_rate_hz must be positive");
  }
  if (generation_.stream_interval <= 0) {
    throw std::invalid_argument("stream_interval must be positive");
  }

  partial_pub_ = create_publisher<std_msgs::msg::String>("~/partial_text", 10);
  result_pub_ = create_publisher<std_msgs::msg::String>("~/result", 10);
  status_pub_ = create_publisher<std_msgs::msg::String>("~/status", 10);
  set_point_mode_srv_ = create_service<std_srvs::srv::SetBool>(
    "~/set_point_mode",
    [this](
      std_srvs::srv::SetBool::Request::SharedPtr request,
      std_srvs::srv::SetBool::Response::SharedPtr response) {
      handleSetPointMode(std::move(request), std::move(response));
    });

  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    image_topic_, rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { imageCallback(std::move(msg)); });

  auto period = std::chrono::duration<double>(1.0 / fixed_rate_hz_);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this]() { timerCallback(); });

  if (backend_type_ != "mock" && backend_type_ != "edgellm") {
    throw std::invalid_argument("backend_type must be 'mock' or 'edgellm'");
  }
  initializeBackend();

  RCLCPP_INFO(
    get_logger(), "started with image_topic=%s fixed_rate_hz=%.3f backend_type=%s prompt_mode=%s",
    image_topic_.c_str(), fixed_rate_hz_, backend_type_.c_str(), prompt_mode_.c_str());
}

VlmNode::~VlmNode()
{
  shutting_down_.store(true);
  if (timer_) {
    timer_->cancel();
  }
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

void VlmNode::imageCallback(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  FrameSnapshot frame;
  if (!convertImage(msg, frame)) {
    publishStatus("error:image_conversion");
    return;
  }

  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    latest_frame_ = std::move(frame);
  }
}

bool VlmNode::convertImage(
  sensor_msgs::msg::Image::ConstSharedPtr const& msg, FrameSnapshot& frame)
{
  namespace enc = sensor_msgs::image_encodings;

  try {
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
    cv::Mat rgb;

    if (msg->encoding == enc::RGB8) {
      rgb = cv_ptr->image;
    } else if (msg->encoding == enc::BGR8) {
      cv::cvtColor(cv_ptr->image, rgb, cv::COLOR_BGR2RGB);
    } else if (msg->encoding == enc::RGBA8) {
      cv::cvtColor(cv_ptr->image, rgb, cv::COLOR_RGBA2RGB);
    } else if (msg->encoding == enc::BGRA8) {
      cv::cvtColor(cv_ptr->image, rgb, cv::COLOR_BGRA2RGB);
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "unsupported image encoding: %s",
        msg->encoding.c_str());
      return false;
    }

    frame.rgb = rgb.isContinuous() ? rgb.clone() : rgb.clone();
    frame.stamp = rclcpp::Time(msg->header.stamp);
    frame.frame_id = msg->header.frame_id;
    frame.source_encoding = msg->encoding;
    frame.width = static_cast<int>(msg->width);
    frame.height = static_cast<int>(msg->height);
    return true;
  } catch (std::exception const& ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "image conversion failed: %s", ex.what());
    return false;
  }
}

void VlmNode::initializeBackend()
{
  if (backend_type_ == "mock") {
    backend_ = std::make_unique<MockBackend>();
    return;
  }

  if (backend_type_ == "edgellm") {
#ifdef EDGELLM_VLM_ENABLE_EDGELLM
    backend_ = std::make_unique<EdgeLlmBackend>(engine_dir_, multimodal_engine_dir_, plugin_path_);
    return;
#else
    throw std::runtime_error(
      "backend_type:=edgellm requires rebuilding edgellm_vlm_ros with "
      "-DEDGELLM_VLM_ENABLE_EDGELLM=ON");
#endif
  }

  throw std::invalid_argument("backend_type must be 'mock' or 'edgellm'");
}

void VlmNode::timerCallback()
{
  if (shutting_down_.load()) {
    return;
  }

  if (worker_thread_.joinable() && !inference_running_.load()) {
    worker_thread_.join();
  }

  FrameSnapshot frame;
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (!latest_frame_) {
      publishStatus("idle:no_frame");
      return;
    }
    frame = *latest_frame_;
    frame.rgb = latest_frame_->rgb.clone();
  }

  bool expected = false;
  if (!inference_running_.compare_exchange_strong(expected, true)) {
    publishStatus("skip:busy");
    return;
  }

  publishStatus("running");
  worker_thread_ = std::thread([this, frame = std::move(frame)]() mutable {
    runInference(std::move(frame));
  });
}

void VlmNode::handleSetPointMode(
  std_srvs::srv::SetBool::Request::SharedPtr request,
  std_srvs::srv::SetBool::Response::SharedPtr response)
{
  std::string mode;
  {
    std::lock_guard<std::mutex> lock(generation_mutex_);
    if (request->data) {
      prompt_mode_ = "point";
      generation_ = point_generation_;
    } else {
      prompt_mode_ = "primitive";
      generation_ = primitive_generation_;
    }
    mode = prompt_mode_;
  }

  response->success = true;
  response->message = "prompt_mode=" + mode;
  publishStatus("prompt_mode:" + mode);
}

void VlmNode::runInference(FrameSnapshot frame)
{
  try {
    GenerationConfig generation;
    std::string prompt_mode;
    {
      std::lock_guard<std::mutex> lock(generation_mutex_);
      generation = generation_;
      prompt_mode = prompt_mode_;
    }

    InferenceResult const result = backend_->infer(
      frame,
      generation,
      [this](std::string const& text) { publishPartial(text); },
      shutting_down_);
    publishResult(
      frame,
      prompt_mode,
      result.text,
      result.finish_reason,
      result.ttft_ms,
      result.total_ms,
      result.tokens);
    publishStatus("done");
  } catch (std::exception const& ex) {
    publishStatus(std::string("error:exception:") + ex.what());
  }

  inference_running_.store(false);
}

void VlmNode::publishStatus(std::string const& status)
{
  std_msgs::msg::String msg;
  msg.data = status;
  status_pub_->publish(msg);
}

void VlmNode::publishPartial(std::string const& text)
{
  std_msgs::msg::String msg;
  msg.data = text;
  partial_pub_->publish(msg);
}

void VlmNode::publishResult(
  FrameSnapshot const& frame,
  std::string const& prompt_mode,
  std::string const& text,
  std::string const& finish_reason,
  double ttft_ms,
  double total_ms,
  size_t tokens)
{
  std::ostringstream json;
  json << "{"
       << "\"stamp\":" << stampToSec(frame.stamp) << ","
       << "\"frame_id\":\"" << jsonEscape(frame.frame_id) << "\","
       << "\"text\":\"" << jsonEscape(text) << "\","
       << "\"finish_reason\":\"" << jsonEscape(finish_reason) << "\","
       << "\"ttft_ms\":" << ttft_ms << ","
       << "\"total_ms\":" << total_ms << ","
       << "\"tokens\":" << tokens << ","
       << "\"image_width\":" << frame.width << ","
       << "\"image_height\":" << frame.height << ","
       << "\"source_encoding\":\"" << jsonEscape(frame.source_encoding) << "\","
       << "\"prompt_mode\":\"" << jsonEscape(prompt_mode) << "\","
       << "\"backend_type\":\"" << jsonEscape(backend_type_) << "\""
       << "}";

  std_msgs::msg::String msg;
  msg.data = json.str();
  result_pub_->publish(msg);
}

}  // namespace edgellm_vlm_ros
