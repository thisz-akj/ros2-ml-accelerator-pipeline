#include "overlay.hpp"

#include <simaai/pose_serialization.h>

#include <fstream>
#include <iostream>
#include <rclcpp_components/register_node_macro.hpp>
#include <vector>

#include "simaai/pose/pose.h"


std::vector<unsigned char> Overlay::read_dump_file(std::string dump_file) {
  std::ifstream file(dump_file, std::ios::binary | std::ios::ate);
  if (!file) {
    std::cerr << "Error opening file.\n";
    return {};
  }
  std::streamsize file_size = file.tellg();
  file.seekg(0, std::ios::beg);
  std::vector<unsigned char> buffer(file_size);
  if (!file.read(reinterpret_cast<char *>(buffer.data()), file_size)) {
    std::cerr << "Error reading file.\n";
    return {};
  }

  return buffer;
}

static void safe_unmap_and_free(
    simaai::memory_resource::GenericMemoryResource *resource,
    simaai_common::msg::SimaaiBuffer &handle,
    bool was_mapped)
{
  if (was_mapped) {
    resource->simaai_buffer_unmap(handle);
  }
  resource->simaai_buffer_free(handle);
}

bool Overlay::process_and_draw(
    const std::vector<std::vector<float>> &pose_entries,
    const std::vector<std::vector<float>> &all_keypoints) {
  if (pose_entries.empty() || all_keypoints.empty()) {
    RCLCPP_WARN(get_logger(), "Overlay: no pose or keypoint data");
    return false;
  }

  auto poses =
      simaai::cv_helpers::create_current_poses(pose_entries, all_keypoints);
  for (auto &pose : poses) {
    pose.drawYUV420(y_, u_, v_);
  }
  return true;
}

bool Overlay::update_renderer(unsigned char *pixels, unsigned int width,
                              unsigned int height) {
  size_t lumaSize   = static_cast<size_t>(width) * height;
  size_t chromaSize = (static_cast<size_t>(width) / 2) * (height / 2);
  
  // Re‐point the Mats’ data pointers (no allocations/copies here):
  y_.data = pixels;                               // first width*height bytes
  u_.data = pixels + lumaSize;                    // next (width/2)*(height/2) bytes
  v_.data = pixels + lumaSize + chromaSize;       // final (width/2)*(height/2) bytes

  // Update strides if you ever need them (often stride == width for single-channel):
  y_stride_ = static_cast<int>(width);
  u_stride_ = static_cast<int>(width / 2);
  v_stride_ = static_cast<int>(width / 2);

  return true;
}

void Overlay::image_callback(simaai_common::msg::Image::ConstSharedPtr msg) {
  rclcpp::Time t0;
  if (!silent)
    t0 = now();
  auto &buf_handle =
      const_cast<simaai_common::msg::SimaaiBuffer &>(msg->image_buffer_overlay);
  
  void *in_ptr = generic_memory_resource_->simaai_buffer_map(buf_handle);
  if (!in_ptr) {
    generic_memory_resource_->simaai_buffer_free(buf_handle);
    RCLCPP_ERROR(get_logger(), "Overlay: could not map overlay buffer");
    return;
  }

  size_t yuv_size = buf_handle.sizes[0];
  pixels_.assign(
    static_cast<unsigned char *>(in_ptr),
    static_cast<unsigned char *>(in_ptr) + yuv_size
  );
  
  generic_memory_resource_->simaai_buffer_unmap(buf_handle);
  generic_memory_resource_->simaai_buffer_free(buf_handle);
  if (!update_renderer(this->pixels_.data(), frame_width, frame_height)) {
    RCLCPP_WARN(get_logger(),
                "Overlay: Update_renderer for frame_id=%lu, dropping",
                msg->frame_id);
  }
  last_frame_id_ = msg->frame_id;
  have_frame_ = true;
  if (!silent){
    rclcpp::Duration t1 = now() - t0;
    double t_ms = static_cast<double>(t1.nanoseconds()) / 1e6;
    RCLCPP_INFO(get_logger(),
                "*** Overlay: image_callback took %.2f ms ***",
                t_ms);
  }
}

bool Overlay::keypoints_callback(
    simaai_common::msg::SimaaiMessage::ConstSharedPtr msg) {
  
  RCLCPP_DEBUG(get_logger(), "Overlay: Received keypoints for frame_id=%lu",
              msg->frame_id);

  if (!have_frame_) {
    RCLCPP_WARN(get_logger(),
                "Overlay: No frame ready for frame_id=%lu, dropping",
                msg->frame_id);
    return false;
  }
  rclcpp::Time t0;
  if (!silent)
    t0 = now();
  auto &kp_handle = const_cast<simaai_common::msg::SimaaiBuffer &>(msg->payload);
  void *kp_data = generic_memory_resource_->simaai_buffer_map(kp_handle);
  if (!kp_data) {
    RCLCPP_ERROR(get_logger(), "Overlay: could not map keypoints buffer");
    generic_memory_resource_->simaai_buffer_free(kp_handle);
    return false;
  }
  bool mapped_success = true;
  std::vector<std::vector<float>> pose_entries;
  std::vector<std::vector<float>> all_keypoints;

  int offset = simaai::cv_helpers::deserialize(kp_data, pose_entries);
  if (offset < 0) {
    safe_unmap_and_free(generic_memory_resource_, kp_handle, mapped_success);
    return false;
  }
  void *next_ptr = static_cast<unsigned char *>(kp_data) + offset;
  int offset2 = simaai::cv_helpers::deserialize(next_ptr, all_keypoints);
  if (offset2 < 0) {
    safe_unmap_and_free(generic_memory_resource_, kp_handle, mapped_success);
    return false;
  }
  safe_unmap_and_free(generic_memory_resource_, kp_handle, mapped_success);

  
  // auto out_msg = std::make_shared<simaai_common::msg::Image>();
  auto out_msg = std::make_shared<simaai_common::msg::SimaaiMessage>();
  simaai::memory_resource::SimaaiMessageInfo info_ov;
  info_ov.num_segments = 1;
  info_ov.sizes.fill(0);
  info_ov.sizes[0] =
      static_cast<size_t>(frame_width) * frame_height * 3 / 2;  // YUV420P
  generic_memory_resource_->simaai_buffer_allocate(
      out_msg->payload, info_ov);

  void *ov_ptr = generic_memory_resource_->simaai_buffer_map(
      out_msg->payload);
  if (!ov_ptr) {
    RCLCPP_ERROR(get_logger(), "Overlay: map(overlay) fail");
    generic_memory_resource_->simaai_buffer_free(out_msg->payload);
    return false;
  }

  {
    auto byte_ptr = static_cast<unsigned char *>(ov_ptr);
    size_t yuv_size = static_cast<size_t>(frame_width) * frame_height * 3 / 2;
    // Copy all YUV planes at once from the previously‐stored frame
    std::memcpy(byte_ptr, pixels_.data(), yuv_size);
  }
  
  const size_t lumaSize = static_cast<size_t>(frame_width) * frame_height;
  const size_t chromaSize =
      (static_cast<size_t>(frame_width) / 2) * (frame_height / 2);

  
  cv::Mat y_ov(
    cv::Size(frame_width, frame_height),
    CV_8UC1,
    static_cast<unsigned char *>(ov_ptr),
    static_cast<size_t>(frame_width)
  );
  cv::Mat u_ov(
    cv::Size(frame_width / 2, frame_height / 2),
    CV_8UC1,
    static_cast<unsigned char *>(ov_ptr) + lumaSize,
    static_cast<size_t>(frame_width / 2)
  );
  cv::Mat v_ov(
    cv::Size(frame_width / 2, frame_height / 2),
    CV_8UC1,
    static_cast<unsigned char *>(ov_ptr) + lumaSize + chromaSize,
    static_cast<size_t>(frame_width / 2)
  );
  auto poses = simaai::cv_helpers::create_current_poses(pose_entries, all_keypoints);
  for (auto &pose : poses) {
    // This calls your existing drawYUV420(cv::Mat&, cv::Mat&, cv::Mat&),
    // but now the Mats point into the overlay’s memory.
    pose.drawYUV420(y_ov, u_ov, v_ov);
  }

  generic_memory_resource_->simaai_buffer_unmap(out_msg->payload);
  if (enable_dump) {
    const uint64_t fid = last_frame_id_;
    std::string dump_name = "/tmp/Overlay_" + std::to_string(fid) + ".bin";

    std::ofstream ofs(dump_name, std::ios::binary);
    if (!ofs) {
      RCLCPP_ERROR(get_logger(), "Overlay: could not open \"%s\" for writing",
                   dump_name.c_str());
    } else {
      // Write exactly info_ov.sizes[0] bytes from ov_ptr
      ofs.write(reinterpret_cast<const char *>(ov_ptr),
                static_cast<std::streamsize>(info_ov.sizes[0]));
      ofs.close();
      RCLCPP_DEBUG(get_logger(), "Overlay: dumped %u bytes to %s",
                  static_cast<unsigned>(info_ov.sizes[0]), dump_name.c_str());
    }
  }

  // Stamp frame_id and publish
  out_msg->frame_id = last_frame_id_;
  RCLCPP_DEBUG(get_logger(), "Overlay: publishing frame_id=%u",
              static_cast<unsigned>(out_msg->frame_id));
  publisher_->publish(*out_msg);
  have_frame_ = false;
  frame_count_++;
  if (!silent){
    rclcpp::Duration delta = now() - t0;
    double elapsed_ms = static_cast<double>(delta.nanoseconds()) / 1e6;
    RCLCPP_INFO(get_logger(),
                "*** Overlay: keypoints_callback execution took %.2f ms ***",
                elapsed_ms);
  }
   
  return true;
}

Overlay::Overlay(const rclcpp::NodeOptions &options)
    : Node("overlay", options) {
  
  // params
  declare_parameter<bool>("dump", false);
  declare_parameter<int>("frame_width", 1280);
  declare_parameter<int>("frame_height", 720);
  declare_parameter<bool>("silent", true);
  int queue_size = declare_parameter<int>("queue_size", 10);
  frame_width = get_parameter("frame_width").as_int();
  frame_height = get_parameter("frame_height").as_int();
  enable_dump = get_parameter("dump").as_bool();
  silent = get_parameter("silent").as_bool();
  RCLCPP_DEBUG(this->get_logger(), "Overlay created");

  size_t yuv_capacity = static_cast<size_t>(frame_width) * frame_height * 3 / 2;
  pixels_.reserve(yuv_capacity);
  y_ = cv::Mat(cv::Size(frame_width, frame_height), CV_8UC1);
  u_ = cv::Mat(cv::Size(frame_width/2, frame_height/2), CV_8UC1);
  v_ = cv::Mat(cv::Size(frame_width/2, frame_height/2), CV_8UC1);

  generic_memory_resource_ =
      static_cast<simaai::memory_resource::GenericMemoryResource *>(
          std::pmr::get_default_resource());
  if (!generic_memory_resource_) {
    RCLCPP_ERROR(this->get_logger(), "Can't obtain generic memory resource!");
    throw std::runtime_error("Failed to obtain generic memory resource!");
  }
  frame_sub_ = create_subscription<simaai_common::msg::Image>(
      "image_raw", queue_size,
      std::bind(&Overlay::image_callback, this, std::placeholders::_1));

  keypoints_sub_ = create_subscription<simaai_common::msg::SimaaiMessage>(
      "simaai/openpose/groupkeypoints/output", queue_size,
      std::bind(&Overlay::keypoints_callback, this, std::placeholders::_1));

  // publisher_ = create_publisher<simaai_common::msg::Image>(
  //     "simaai/openpose/overlay/output", 10);
  publisher_ = create_publisher<simaai_common::msg::SimaaiMessage>(
    "simaai/openpose/overlay/output", queue_size);

  RCLCPP_DEBUG(this->get_logger(), "dump: %s", enable_dump ? "true" : "false");
}

RCLCPP_COMPONENTS_REGISTER_NODE(Overlay)
