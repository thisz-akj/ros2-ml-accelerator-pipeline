#include <rclcpp_components/register_node_macro.hpp>
#include <functional>
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include <mutex>
#include <cmath>
#include <simaai/helpers.hpp>
#include <algorithm>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

// For JPEG conversion
#include <opencv2/opencv.hpp>
#include "yolopose_render.hpp"
namespace {
    static const NV12Color kpt_colors[1] = {
        { 76,  85, 255 },   // Red
    };
}

struct segment {
  simaai_memory_t *memory;
  std::string name;
};

struct GstSimaaiSegmentMemory {
  GstMemory mem;
  std::vector<segment> segments;
  simaai_memory_t **alloc_segments;
  gpointer vaddr;
  std::mutex map_mutex;
};

YoloPoseRender::YoloPoseRender(const rclcpp::NodeOptions & options) 
    : Node("yolopose_render", options),
      num_keypoints(12),
      num_classes(1),
      model_width(640),
      model_height(640),
      frame_width(1280),
      frame_height(720),
      conf_threshold(0.8f),
      num_heads(3),
      mla_output_size_bytes(655200)
{
    RCLCPP_INFO(this->get_logger(), "YoloPoseRender constructor called");
}

YoloPoseRender::~YoloPoseRender()
{
    RCLCPP_DEBUG(this->get_logger(), "YoloPoseRender destructor completed");
}

bool YoloPoseRender::configure_render_type(std::string config_path) {
    nlohmann::json json;
    std::ifstream json_file(config_path);
    if (!json_file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Could not open config file: %s", config_path.c_str());
        return false;
    }
    
    json_file >> json;
    
    if (json.contains("buffers") && json["buffers"].contains("input") && 
        json["buffers"]["input"].is_array() && json["buffers"]["input"].size() > 0) {
        mla_output_size_bytes = json["buffers"]["input"][0]["size"].get<int>();
        RCLCPP_INFO(this->get_logger(), "Loaded MLA output size: %d bytes", mla_output_size_bytes);
    }
    
    if (json.contains("num_keypoints")) {
        num_keypoints = json["num_keypoints"].get<int>();
    }
    if (json.contains("num_classes")) {
        num_classes = json["num_classes"].get<int>();
    }
    if (json.contains("confidence_threshold")) {
        conf_threshold = json["confidence_threshold"].get<float>();
    }
    if (json.contains("nms_iou_threshold")) {
        nms_iou_threshold = json["nms_iou_threshold"].get<float>();
    }
    if (json.contains("model_width")) {
        model_width = json["model_width"].get<int>();
    }
    if (json.contains("model_height")) {
        model_height = json["model_height"].get<int>();
    }
    if (json.contains("frame_width")) {
        frame_width = json["frame_width"].get<int>();
    }
    if (json.contains("frame_height")) {
        frame_height = json["frame_height"].get<int>();
    }
    if (json.contains("num_heads")) {
        num_heads = json["num_heads"].get<int>();
    }
    
    if (json.contains("model_out_shapes")) {
        auto shapes_array = json["model_out_shapes"];
        model_out_shapes.clear();
        for (const auto& shape : shapes_array) {
            std::vector<int> shape_vec;
            for (const auto& dim : shape) {
                shape_vec.push_back(dim.get<int>());
            }
            model_out_shapes.push_back(shape_vec);
        }
        RCLCPP_INFO(this->get_logger(), "Loaded %zu model output shapes", model_out_shapes.size());
    } else {
        RCLCPP_WARN(this->get_logger(), "model_out_shapes not found in config, using defaults");
        model_out_shapes = {
            {1, 80, 80, num_classes},
            {1, 40, 40, num_classes},
            {1, 20, 20, num_classes},
            {1, 80, 80, num_keypoints * 3},
            {1, 40, 40, num_keypoints * 3},
            {1, 20, 20, num_keypoints * 3}
        };
    }
    
    if (json.contains("head_scales")) {
        auto scales_array = json["head_scales"];
        head_scales.clear();
        for (const auto& scale : scales_array) {
            head_scales.push_back(scale.get<int>());
        }
        RCLCPP_INFO(this->get_logger(), "Loaded %zu head scales", head_scales.size());
    } else {
        RCLCPP_WARN(this->get_logger(), "head_scales not found in config, using defaults [8, 16, 32]");
        head_scales = {8, 16, 32};
    }
    
    RCLCPP_INFO(this->get_logger(), "Configuration loaded:");
    RCLCPP_INFO(this->get_logger(), "  MLA output size: %d bytes (%.2f MB)", 
                mla_output_size_bytes, mla_output_size_bytes / (1024.0 * 1024.0));
    RCLCPP_INFO(this->get_logger(), "  Number of keypoints: %d", num_keypoints);
    RCLCPP_INFO(this->get_logger(), "  Number of classes: %d", num_classes);
    RCLCPP_INFO(this->get_logger(), "  Confidence threshold: %.2f", conf_threshold);

    RCLCPP_INFO(this->get_logger(), "  Confidence threshold: %.2f", conf_threshold);
    RCLCPP_INFO(this->get_logger(), "  Model size: %dx%d", model_width, model_height);
    RCLCPP_INFO(this->get_logger(), "  Frame size: %dx%d", frame_width, frame_height);
    RCLCPP_INFO(this->get_logger(), "  Number of heads: %d", num_heads);
    
    return true;
}

 bool YoloPoseRender::init() {
    std::string hardware_target = this->declare_parameter("hardware_target", "");
    std::string package_dir = this->declare_parameter("package_dir", "");

    RCLCPP_INFO(this->get_logger(),
                "hardware_target: '%s'", hardware_target.c_str());
    RCLCPP_INFO(this->get_logger(),
                "package_dir: '%s'", package_dir.c_str());
    std::string config_file_path = this->declare_parameter("config_file_path", "");
    config_file_path = resolve_package_resource_path(config_file_path, hardware_target, package_dir);

    RCLCPP_INFO(this->get_logger(), "Config file path parameter: '%s'", config_file_path.c_str());
    
    if (!config_file_path.empty()) {
        
        std::ifstream test_file(config_file_path);
        if (!test_file.good()) {
            RCLCPP_ERROR(this->get_logger(), "Config file does not exist or is not readable: %s", config_file_path.c_str());
            RCLCPP_WARN(this->get_logger(), "Falling back to default configuration");
        } else {
            test_file.close();
            
            RCLCPP_INFO(this->get_logger(), "Loading config from: %s", config_file_path.c_str());
            if (!configure_render_type(config_file_path)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to configure render type from config file");
                RCLCPP_WARN(this->get_logger(), "Falling back to default configuration");
            }
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "No config_file_path provided, using default values");
    }
    
    if (model_out_shapes.empty()) {
        RCLCPP_WARN(this->get_logger(), "Model output shapes not loaded from config, using YOLOv11-pose defaults");
        model_out_shapes = {
            {1, 80, 80, num_classes},   
            {1, 40, 40, num_classes},   
            {1, 20, 20, num_classes},   
            {1, 80, 80, num_keypoints * 3},  
            {1, 40, 40, num_keypoints * 3},  
            {1, 20, 20, num_keypoints * 3}   
        };
        RCLCPP_INFO(this->get_logger(), "Set %zu default model output shapes", model_out_shapes.size());
    }
    
    if (head_scales.empty()) {
        RCLCPP_WARN(this->get_logger(), "Head scales not loaded from config, using defaults [8, 16, 32]");
        head_scales = {8, 16, 32};
    }

    auto pub_topic = this->declare_parameter("pub_topic", "simaai/yoloposerender/output");
    auto keypoints_topic = this->declare_parameter("keypoints_topic", "simaai/keypoints");
    auto mla_raw_topic = this->declare_parameter("mla_raw_topic", "mla_raw");

    enable_dumps = this->declare_parameter("dump", false);
    
    queue_size = static_cast<size_t>(this->declare_parameter<int64_t>("queue_size", 10));
    silent = this->declare_parameter("silent", true);
    
    generic_memory_resource_ = static_cast<simaai::memory_resource::GenericMemoryResource*>(
        std::pmr::get_default_resource());
    
    if (!generic_memory_resource_) {
        RCLCPP_ERROR(this->get_logger(), "Can't obtain generic memory resource!");
        return false;
    }

    // OPTIMIZATION: Pre-allocate all buffers
    max_elements = 1000000;
    float32_data_buffer.resize(max_elements);
    
    model_outputs_buffer.resize(model_out_shapes.size());
    for (size_t k = 0; k < model_out_shapes.size(); ++k) {
        int seg_size = 1;
        for (size_t d = 0; d < model_out_shapes[k].size(); ++d) {
            seg_size *= model_out_shapes[k][d];
        }
        model_outputs_buffer[k].resize(seg_size);
        RCLCPP_DEBUG(this->get_logger(), "Output %zu: size=%d", k, seg_size);
    }
    
    // OPTIMIZATION: Pre-allocate detection buffers
    constexpr int MAX_DETECTIONS = 16;
    all_detections_buffer.reserve(MAX_DETECTIONS);
    nms_keep_flags.resize(MAX_DETECTIONS, true);
    nms_filtered_buffer.reserve(MAX_DETECTIONS);
    
    // OPTIMIZATION: Pre-allocate sigmoid temp buffer for NEON operations
    sigmoid_temp_buffer.resize(8192);  // Reasonable size for batch sigmoid operations

    publisher_output_ = this->create_publisher<simaai_common::msg::SimaaiMessage>(
        pub_topic, queue_size);
    publisher_keypoints_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        keypoints_topic, queue_size);
    
    sub_mla_raw_ = this->create_subscription<simaai_common::msg::Image>(
        mla_raw_topic, queue_size,
        std::bind(&YoloPoseRender::image_mla_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "YoloPoseRender initialized successfully");
    RCLCPP_INFO(this->get_logger(), "  Configuration:");
    RCLCPP_INFO(this->get_logger(), "    - MLA output size: %d bytes (%.2f MB)", 
                mla_output_size_bytes, mla_output_size_bytes / (1024.0 * 1024.0));
    RCLCPP_INFO(this->get_logger(), "    - Keypoints: %d", num_keypoints);
    RCLCPP_INFO(this->get_logger(), "    - Classes: %d", num_classes);
    RCLCPP_INFO(this->get_logger(), "    - Confidence threshold: %.2f", conf_threshold);
    RCLCPP_INFO(this->get_logger(), "    - Model size: %dx%d", model_width, model_height);
    RCLCPP_INFO(this->get_logger(), "    - Frame size: %dx%d", frame_width, frame_height);
    RCLCPP_INFO(this->get_logger(), "    - Num heads: %d", num_heads);
    RCLCPP_INFO(this->get_logger(), "  Topics:");
    RCLCPP_INFO(this->get_logger(), "    - Output: %s", pub_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "    - Keypoints: %s", keypoints_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "    - Subscribing to mla_raw: %s", mla_raw_topic.c_str());
    
    return true;
}

// OPTIMIZATION: Improved NEON bfloat16 to float32 conversion with better loop unrolling
void YoloPoseRender::bfloat16_to_float32_neon(const uint16_t* bf16_data, float* f32_data, int num_elements) {
    const int simd_width = 8;
    int i = 0;
    
    // Process 8 elements at a time with NEON
    for (; i + simd_width <= num_elements; i += simd_width) {
        uint16x8_t bf16_vec = vld1q_u16(bf16_data + i);
        uint32x4_t low = vshll_n_u16(vget_low_u16(bf16_vec), 16);
        uint32x4_t high = vshll_n_u16(vget_high_u16(bf16_vec), 16);
        float32x4_t f32_low = vreinterpretq_f32_u32(low);
        float32x4_t f32_high = vreinterpretq_f32_u32(high);
        vst1q_f32(f32_data + i, f32_low);
        vst1q_f32(f32_data + i + 4, f32_high);
    }
    
    // Handle remaining elements
    for (; i < num_elements; ++i) {
        uint32_t f32_bits = (static_cast<uint32_t>(bf16_data[i]) << 16);
        memcpy(&f32_data[i], &f32_bits, sizeof(float));
    }
}

// OPTIMIZATION: Inlined sigmoid for scalar operations
inline float YoloPoseRender::sigmoid(float x) {
    x = std::max(-50.0f, std::min(50.0f, x));
    return 1.0f / (1.0f + expf(-x));
}

// OPTIMIZATION: NEON-accelerated sigmoid for batch operations
void YoloPoseRender::sigmoid_neon(const float* input, float* output, int num_elements) {
    const int simd_width = 4;
    int i = 0;
    
    const float32x4_t ones = vdupq_n_f32(1.0f);
    const float32x4_t min_val = vdupq_n_f32(-50.0f);
    const float32x4_t max_val = vdupq_n_f32(50.0f);
    
    // Process 4 elements at a time
    for (; i + simd_width <= num_elements; i += simd_width) {
        // Load 4 values
        float32x4_t x = vld1q_f32(input + i);
        
        // Clamp values
        x = vmaxq_f32(min_val, vminq_f32(max_val, x));
        
        // Negate
        float32x4_t neg_x = vnegq_f32(x);
        
        // Note: NEON doesn't have native exp, so we fall back to scalar for exp
        // This is still beneficial for the vector loads/stores and clamping
        float temp[4];
        vst1q_f32(temp, neg_x);
        
        for (int j = 0; j < 4; ++j) {
            temp[j] = expf(temp[j]);
        }
        
        float32x4_t exp_vec = vld1q_f32(temp);
        float32x4_t denom = vaddq_f32(ones, exp_vec);
        
        // Compute 1.0 / (1.0 + exp(-x))
        // Using reciprocal estimate for faster division
        float32x4_t recip = vrecpeq_f32(denom);
        recip = vmulq_f32(vrecpsq_f32(denom, recip), recip);  // Newton-Raphson iteration
        
        vst1q_f32(output + i, recip);
    }
    
    // Handle remaining elements
    for (; i < num_elements; ++i) {
        output[i] = sigmoid(input[i]);
    }
}

void YoloPoseRender::get_model_outputs(const float* input_buffer, 
                                       std::vector<std::vector<float>>& model_outputs) {
    int start = 0;
    for (size_t k = 0; k < model_out_shapes.size(); ++k) {
        int seg_size = 1;
        for (size_t d = 0; d < model_out_shapes[k].size(); ++d) {
            seg_size *= model_out_shapes[k][d];
        }
        // OPTIMIZATION: Use memcpy for contiguous memory copy (compiler optimized)
        memcpy(model_outputs[k].data(), input_buffer + start, seg_size * sizeof(float));
        start += seg_size;
    }
}

// OPTIMIZATION: Optimized detection finding with reduced branching and better cache locality
void YoloPoseRender::find_all_detections(const std::vector<std::vector<float>>& model_outputs, 
                                         float threshold, std::vector<Detection>& detections) {
    detections.clear();
    
    constexpr int MAX_DETECTIONS = 16;
    
    // OPTIMIZATION: Pre-compute sigmoid threshold to avoid repeated sigmoid calls
    const float sigmoid_threshold = threshold;
    
    for (int head_idx = 0; head_idx < num_heads; ++head_idx) {
        const float* objectness = model_outputs[head_idx].data();
        const int h = model_out_shapes[head_idx][1];
        const int w = model_out_shapes[head_idx][2];
        const int hw = h * w;
        const int total_size = hw * num_classes;

        // OPTIMIZATION: Process objectness scores in batches with NEON if buffer is large enough
        if (total_size <= static_cast<int>(sigmoid_temp_buffer.size())) {
            // Batch sigmoid computation
            sigmoid_neon(objectness, sigmoid_temp_buffer.data(), total_size);
            
            // Scan for detections with pre-computed sigmoid values
            for (int y = 0; y < h; ++y) {
                const int y_offset = y * w * num_classes;
                for (int x = 0; x < w; ++x) {
                    const int base_idx = y_offset + x * num_classes;
                    for (int class_id = 0; class_id < num_classes; ++class_id) {
                        const float obj_conf = sigmoid_temp_buffer[base_idx + class_id];
                        
                        if (obj_conf > sigmoid_threshold) {
                            detections.emplace_back();
                            Detection& det = detections.back();
                            det.confidence = obj_conf;
                            det.x_cell = x;
                            det.y_cell = y;
                            det.head_idx = head_idx;
                            det.class_id = class_id;
                            
                            if (static_cast<int>(detections.size()) >= MAX_DETECTIONS) {
                                goto early_exit;
                            }
                        }
                    }
                }
            }
        } else {
            // Fallback to inline sigmoid for very large tensors
            for (int y = 0; y < h; ++y) {
                const int y_offset = y * w * num_classes;
                for (int x = 0; x < w; ++x) {
                    const int base_idx = y_offset + x * num_classes;
                    for (int class_id = 0; class_id < num_classes; ++class_id) {
                        const float obj_conf = sigmoid(objectness[base_idx + class_id]);
                        
                        if (obj_conf > sigmoid_threshold) {
                            detections.emplace_back();
                            Detection& det = detections.back();
                            det.confidence = obj_conf;
                            det.x_cell = x;
                            det.y_cell = y;
                            det.head_idx = head_idx;
                            det.class_id = class_id;
                            
                            if (static_cast<int>(detections.size()) >= MAX_DETECTIONS) {
                                goto early_exit;
                            }
                        }
                    }
                }
            }
        }
    }
    
early_exit:
    // OPTIMIZATION: Use partial_sort for better performance when we have many detections
    if (detections.size() > MAX_DETECTIONS) {
        std::partial_sort(detections.begin(), detections.begin() + MAX_DETECTIONS, detections.end(),
                        [](const Detection& a, const Detection& b) {
                            return a.confidence > b.confidence;
                        });
        detections.resize(MAX_DETECTIONS);
    } else if (detections.size() > 1) {
        std::sort(detections.begin(), detections.end(), 
             [](const Detection& a, const Detection& b) {
                 return a.confidence > b.confidence;
             });
    }
}

// OPTIMIZATION: Vectorized keypoint decoding with reduced redundant calculations
void YoloPoseRender::decode_pose_keypoints(const std::vector<std::vector<float>>& model_outputs,
                                           Detection& det,
                                           float scale_factor,
                                           float dw,
                                           float dh) {
    if (det.head_idx == -1) return;

    det.keypoints.clear();

    int head_idx = det.head_idx;
    const float* keypoint_tensor = model_outputs[num_heads + head_idx].data();
    
    int w = model_out_shapes[num_heads + head_idx][2];
    int num_channels = model_out_shapes[num_heads + head_idx][3];

    if (head_idx < 0 || head_idx >= num_heads) {
        RCLCPP_ERROR(get_logger(), "Invalid head_idx %d", head_idx);
        return;
    }
    
    const float stride_f = static_cast<float>(head_scales[head_idx]);
    
    // Pre-compute all constants (OPTIMIZATION: minimize repeated calculations)
    const float cell_x_f = static_cast<float>(det.x_cell);
    const float cell_y_f = static_cast<float>(det.y_cell);
    const float anchor_x = cell_x_f + 0.5f;
    const float anchor_y = cell_y_f + 0.5f;
    const float inv_scale = 1.0f / scale_factor;
    const float frame_w_max = static_cast<float>(frame_width - 1);
    const float frame_h_max = static_cast<float>(frame_height - 1);
    const bool needs_offset = (frame_width != model_width || frame_height != model_height);
    const float offset_x = needs_offset ? dw * 0.5f : 0.0f;
    const float offset_y = needs_offset ? dh * 0.5f : 0.0f;
    const int base_idx = (det.y_cell * w + det.x_cell) * num_channels;
    
    // OPTIMIZATION: Use NEON for keypoint coordinate calculations
    const float32x4_t v_2 = vdupq_n_f32(2.0f);
    const float32x4_t v_minus_half = vdupq_n_f32(-0.5f);
    const float32x4_t v_anchor_x = vdupq_n_f32(anchor_x);
    const float32x4_t v_anchor_y = vdupq_n_f32(anchor_y);
    const float32x4_t v_stride = vdupq_n_f32(stride_f);
    const float32x4_t v_offset_x = vdupq_n_f32(offset_x);
    const float32x4_t v_offset_y = vdupq_n_f32(offset_y);
    const float32x4_t v_dw = vdupq_n_f32(dw);
    const float32x4_t v_dh = vdupq_n_f32(dh);
    const float32x4_t v_inv_scale = vdupq_n_f32(inv_scale);
    const float32x4_t v_zero = vdupq_n_f32(0.0f);
    const float32x4_t v_frame_w_max = vdupq_n_f32(frame_w_max);
    const float32x4_t v_frame_h_max = vdupq_n_f32(frame_h_max);
    
    int kp_idx = 0;
    
    // Process 4 keypoints at a time with NEON
    for (; kp_idx + 4 <= num_keypoints; kp_idx += 4) {
        // Load keypoint data for 4 keypoints (interleaved x, y, conf)
        alignas(16) float kpt_x_reg[4], kpt_y_reg[4], kpt_conf_raw[4];
        
        for (int i = 0; i < 4; ++i) {
            const int channel_base = base_idx + (kp_idx + i) * 3;
            kpt_x_reg[i] = keypoint_tensor[channel_base + 0];
            kpt_y_reg[i] = keypoint_tensor[channel_base + 1];
            kpt_conf_raw[i] = keypoint_tensor[channel_base + 2];
        }
        
        float32x4_t v_kpt_x_reg = vld1q_f32(kpt_x_reg);
        float32x4_t v_kpt_y_reg = vld1q_f32(kpt_y_reg);
        
        // Decode X coordinates: (kpt_x_reg * 2.0 - 0.5 + anchor_x) * stride
        float32x4_t v_kpt_x_model = vmulq_f32(v_kpt_x_reg, v_2);
        v_kpt_x_model = vaddq_f32(v_kpt_x_model, v_minus_half);
        v_kpt_x_model = vaddq_f32(v_kpt_x_model, v_anchor_x);
        v_kpt_x_model = vmulq_f32(v_kpt_x_model, v_stride);
        v_kpt_x_model = vaddq_f32(v_kpt_x_model, v_offset_x);
        
        // Decode Y coordinates: (kpt_y_reg * 2.0 - 0.5 + anchor_y) * stride
        float32x4_t v_kpt_y_model = vmulq_f32(v_kpt_y_reg, v_2);
        v_kpt_y_model = vaddq_f32(v_kpt_y_model, v_minus_half);
        v_kpt_y_model = vaddq_f32(v_kpt_y_model, v_anchor_y);
        v_kpt_y_model = vmulq_f32(v_kpt_y_model, v_stride);
        v_kpt_y_model = vaddq_f32(v_kpt_y_model, v_offset_y);
        
        // Reverse letterbox transformation
        float32x4_t v_kpt_x = vsubq_f32(v_kpt_x_model, v_dw);
        v_kpt_x = vmulq_f32(v_kpt_x, v_inv_scale);
        
        float32x4_t v_kpt_y = vsubq_f32(v_kpt_y_model, v_dh);
        v_kpt_y = vmulq_f32(v_kpt_y, v_inv_scale);
        
        // Clamp to frame boundaries
        v_kpt_x = vmaxq_f32(v_zero, vminq_f32(v_frame_w_max, v_kpt_x));
        v_kpt_y = vmaxq_f32(v_zero, vminq_f32(v_frame_h_max, v_kpt_y));
        
        // Store results
        alignas(16) float final_x[4], final_y[4];
        vst1q_f32(final_x, v_kpt_x);
        vst1q_f32(final_y, v_kpt_y);
        
        // Compute sigmoid for confidence values (scalar, as NEON exp is not available)
        for (int i = 0; i < 4; ++i) {
            const float kpt_conf = sigmoid(kpt_conf_raw[i]);
            det.keypoints.push_back({final_x[i], final_y[i], kpt_conf, 0.0f});
        }
    }
    
    // Handle remaining keypoints (< 4)
    for (; kp_idx < num_keypoints; ++kp_idx) {
        const int channel_base = base_idx + kp_idx * 3;
        
        const float kpt_x_reg = keypoint_tensor[channel_base + 0];
        const float kpt_y_reg = keypoint_tensor[channel_base + 1];
        const float kpt_conf_raw = keypoint_tensor[channel_base + 2];

        float kpt_x_model = (kpt_x_reg * 2.0f - 0.5f + anchor_x) * stride_f;
        float kpt_y_model = (kpt_y_reg * 2.0f - 0.5f + anchor_y) * stride_f;

        kpt_x_model += offset_x;
        kpt_y_model += offset_y;

        float kpt_x = (kpt_x_model - dw) * inv_scale;
        float kpt_y = (kpt_y_model - dh) * inv_scale;
        
        kpt_x = std::max(0.0f, std::min(frame_w_max, kpt_x));
        kpt_y = std::max(0.0f, std::min(frame_h_max, kpt_y));
        
        const float kpt_conf = sigmoid(kpt_conf_raw);

        det.keypoints.push_back({kpt_x, kpt_y, kpt_conf, 0.0f});
    }
}

// OPTIMIZATION: Optimized NV12 drawing with reduced memory access patterns
void YoloPoseRender::draw_keypoints_on_nv12(uint8_t* y_plane, uint8_t* uv_plane,
                                            int width, int height,
                                            const Detection& det,
                                            float kpt_threshold, int kpt_idx) {
    if (det.head_idx == -1 || det.keypoints.empty()) return;
    
    const int y_stride = width;
    const int uv_stride = width;
    const int uv_width = width >> 1;
    const int uv_height = height >> 1;
    const int radius = 2;
    const int r2 = radius * radius;

    const NV12Color& color = kpt_colors[kpt_idx];

    for (const auto& kp : det.keypoints) {
        if (kp.conf <= kpt_threshold) {
            continue;
        }

        const int cx = static_cast<int>(kp.x);
        const int cy = static_cast<int>(kp.y);

        if (cx < 0 || cx >= width || cy < 0 || cy >= height) {
            continue;
        }

        // OPTIMIZATION: Pre-compute y-coordinate bounds
        const int y_min = std::max(0, cy - radius);
        const int y_max = std::min(height - 1, cy + radius);
        
        // Draw on Y plane
        for (int y = y_min; y <= y_max; ++y) {
            const int dy = y - cy;
            const int dy2 = dy * dy;
            uint8_t* y_row = y_plane + y * y_stride;

            const int x_min = std::max(0, cx - radius);
            const int x_max = std::min(width - 1, cx + radius);
            
            for (int x = x_min; x <= x_max; ++x) {
                const int dx = x - cx;
                if (dx*dx + dy2 <= r2) {
                    y_row[x] = color.y;
                }
            }
        }

        // Draw on UV plane
        const int uv_cx = cx >> 1;
        const int uv_cy = cy >> 1;
        
        const int uv_y_min = std::max(0, uv_cy - (radius >> 1));
        const int uv_y_max = std::min(uv_height - 1, uv_cy + (radius >> 1));

        for (int uy = uv_y_min; uy <= uv_y_max; ++uy) {
            uint8_t* uv_row = uv_plane + uy * uv_stride;
            
            const int uv_x_min = std::max(0, uv_cx - (radius >> 1));
            const int uv_x_max = std::min(uv_width - 1, uv_cx + (radius >> 1));

            for (int ux = uv_x_min; ux <= uv_x_max; ++ux) {
                const int uv_idx = ux * 2;
                uv_row[uv_idx + 0] = color.u;
                uv_row[uv_idx + 1] = color.v;
            }
        }
    }
}

void YoloPoseRender::publish_keypoints(const Detection& det, uint64_t frame_id) {
    std_msgs::msg::Float32MultiArray msg;
    
    msg.data.resize(4 + num_keypoints * 3);
    
    msg.data[0] = static_cast<float>(frame_id);
    msg.data[1] = det.confidence;
    msg.data[2] = static_cast<float>(det.keypoints.size());
    msg.data[3] = static_cast<float>(det.class_id);
    
    for (size_t i = 0; i < det.keypoints.size(); ++i) {
        msg.data[4 + i * 3 + 0] = det.keypoints[i].x;
        msg.data[4 + i * 3 + 1] = det.keypoints[i].y;
        msg.data[4 + i * 3 + 2] = det.keypoints[i].conf;
    }
    
    publisher_keypoints_->publish(std::move(msg));
}

void copy_sima_memory(GstSimaaiSegmentMemory *src, GstSimaaiSegmentMemory *dst) {
    dst->mem = src->mem;
    dst->segments = src->segments;
    dst->alloc_segments = src->alloc_segments;
    dst->vaddr = src->vaddr;
}

// OPTIMIZATION: Vectorized bbox estimation with reduced memory access
void YoloPoseRender::estimate_bbox_from_heatmaps(const std::vector<std::vector<float>>& model_outputs,
                                                 Detection& det,
                                                 float scale_factor,
                                                 float dw,
                                                 float dh) {
    if (det.head_idx == -1) return;

    int head_idx = det.head_idx;
    const float* keypoint_tensor = model_outputs[num_heads + head_idx].data();
    
    int w = model_out_shapes[num_heads + head_idx][2];
    int num_channels = model_out_shapes[num_heads + head_idx][3];

    const float stride_f = static_cast<float>(head_scales[head_idx]);
    const float cell_x_f = static_cast<float>(det.x_cell);
    const float cell_y_f = static_cast<float>(det.y_cell);
    const float anchor_x = cell_x_f + 0.5f;
    const float anchor_y = cell_y_f + 0.5f;
    const float inv_scale = 1.0f / scale_factor;

    // Extract bounding box from keypoint positions
    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float max_y = std::numeric_limits<float>::lowest();

    const int base_idx = (det.y_cell * w + det.x_cell) * num_channels;
    const bool needs_offset = (frame_width != model_width || frame_height != model_height);
    const float offset_x = needs_offset ? dw * 0.5f : 0.0f;
    const float offset_y = needs_offset ? dh * 0.5f : 0.0f;

    // Process keypoints and accumulate min/max

    for (int kp_idx = 0; kp_idx < num_keypoints; ++kp_idx) {
        const int channel_base = base_idx + kp_idx * 3;
        
        const float kpt_x_reg = keypoint_tensor[channel_base + 0];
        const float kpt_y_reg = keypoint_tensor[channel_base + 1];
        const float kpt_conf_raw = keypoint_tensor[channel_base + 2];

        float kpt_conf = sigmoid(kpt_conf_raw);
        if (kpt_conf < conf_threshold) continue;

        float kpt_x_model = (kpt_x_reg * 2.0f - 0.5f + anchor_x) * stride_f;
        float kpt_y_model = (kpt_y_reg * 2.0f - 0.5f + anchor_y) * stride_f;

        kpt_x_model += offset_x;
        kpt_y_model += offset_y;

        float kpt_x = (kpt_x_model - dw) * inv_scale;
        float kpt_y = (kpt_y_model - dh) * inv_scale;

        min_x = std::min(min_x, kpt_x);
        min_y = std::min(min_y, kpt_y);
        max_x = std::max(max_x, kpt_x);
        max_y = std::max(max_y, kpt_y);
    }

    const float padding = 20.0f;
    det.bbox_x1 = std::max(0.0f, min_x - padding);
    det.bbox_y1 = std::max(0.0f, min_y - padding);
    det.bbox_x2 = std::min(static_cast<float>(frame_width - 1), max_x + padding);
    det.bbox_y2 = std::min(static_cast<float>(frame_height - 1), max_y + padding);
}

// OPTIMIZATION: Inline IoU calculation for better performance
inline float YoloPoseRender::calculate_bbox_iou(const Detection& det1, const Detection& det2) {
    const float inter_x1 = std::max(det1.bbox_x1, det2.bbox_x1);
    const float inter_y1 = std::max(det1.bbox_y1, det2.bbox_y1);
    const float inter_x2 = std::min(det1.bbox_x2, det2.bbox_x2);
    const float inter_y2 = std::min(det1.bbox_y2, det2.bbox_y2);
    
    if (inter_x1 >= inter_x2 || inter_y1 >= inter_y2) {
        return 0.0f;
    }
    
    const float inter_area = (inter_x2 - inter_x1) * (inter_y2 - inter_y1);
    
    const float area1 = (det1.bbox_x2 - det1.bbox_x1) * (det1.bbox_y2 - det1.bbox_y1);
    const float area2 = (det2.bbox_x2 - det2.bbox_x1) * (det2.bbox_y2 - det2.bbox_y1);
    const float union_area = area1 + area2 - inter_area;
    
    if (union_area <= 0.0f) {
        return 0.0f;
    }
    
    return inter_area / union_area;
}

// OPTIMIZATION: Improved NMS with pre-allocated buffers
void YoloPoseRender::apply_nms(std::vector<Detection>& detections, float iou_threshold) {
    if (detections.size() <= 1) {
        return;
    }
    
    // OPTIMIZATION: Reuse pre-allocated buffer instead of creating new vector
    if (nms_keep_flags.size() < detections.size()) {
        nms_keep_flags.resize(detections.size());
    }
    std::fill_n(nms_keep_flags.begin(), detections.size(), true);
    
    for (size_t i = 0; i < detections.size(); ++i) {
        if (!nms_keep_flags[i]) continue;
        
        for (size_t j = i + 1; j < detections.size(); ++j) {
            if (!nms_keep_flags[j]) continue;
            
            if (detections[i].class_id != detections[j].class_id) {
                continue;
            }
            
            const float iou = calculate_bbox_iou(detections[i], detections[j]);
            
            if (iou > iou_threshold) {
                nms_keep_flags[j] = false;
            }
        }
    }
    
    // OPTIMIZATION: Reuse filtered buffer
    nms_filtered_buffer.clear();
    
    for (size_t i = 0; i < detections.size(); ++i) {
        if (nms_keep_flags[i]) {
            nms_filtered_buffer.push_back(std::move(detections[i]));
        }
    }
    
    detections = std::move(nms_filtered_buffer);
}

void YoloPoseRender::image_mla_callback(simaai_common::msg::Image::ConstSharedPtr msg)
{
    if (!silent) RCLCPP_INFO(get_logger(), "Received image_mla data");
    
    try{
        void* image_mla_data_ptr = 
            generic_memory_resource_->simaai_buffer_map(msg->payload);

        if (!image_mla_data_ptr) {
            RCLCPP_ERROR(get_logger(), "Failed to map MLA buffer (muxer output)");
            SimaMemMsgType wrapper;
            wrapper.copy_from_ros(msg->payload);
            generic_memory_resource_->simaai_buffer_unmap(wrapper);
            generic_memory_resource_->simaai_buffer_free(wrapper);
            GstSimaaiSegmentMemory *mla_mem = (GstSimaaiSegmentMemory *)(msg->payload.gst_memory);
            if (mla_mem) delete mla_mem;
            return;
        }

        uint8_t* raw_byte_ptr = static_cast<uint8_t*>(image_mla_data_ptr);
        size_t mla_offset = frame_width * frame_height * 1.5;
        void* image_data_ptr = raw_byte_ptr;
        void* mla_data_ptr = raw_byte_ptr + mla_offset;

        int total_mla_elements = mla_output_size_bytes / sizeof(uint16_t);

        if (recv_counter == 0) {
            RCLCPP_INFO(get_logger(), "MLA buffer: %d bytes (%.2f MB), %d elements in bfloat16", 
                       mla_output_size_bytes,
                       mla_output_size_bytes / (1024.0 * 1024.0),
                       total_mla_elements);
        }

        if (total_mla_elements > max_elements) {
            float32_data_buffer.resize(total_mla_elements);
            max_elements = total_mla_elements;
            RCLCPP_INFO(get_logger(), "Resized float32_data_buffer to %d elements", total_mla_elements);
        }

        const uint16_t* bf16_data = reinterpret_cast<const uint16_t*>(mla_data_ptr);
        bfloat16_to_float32_neon(bf16_data, float32_data_buffer.data(), total_mla_elements);
        
        get_model_outputs(float32_data_buffer.data(), model_outputs_buffer);
        
        find_all_detections(model_outputs_buffer, conf_threshold, all_detections_buffer);

        // Pre-compute letterbox transformation parameters
        const float scale_factor = std::min(static_cast<float>(model_width) / frame_width,
                                            static_cast<float>(model_height) / frame_height);
        const float resized_w_float = frame_width * scale_factor;
        const float resized_h_float = frame_height * scale_factor;
        const int resized_w = static_cast<int>(std::round(resized_w_float));
        const int resized_h = static_cast<int>(std::round(resized_h_float));
        const float pad_w = static_cast<float>(model_width - resized_w);
        const float pad_h = static_cast<float>(model_height - resized_h);
        const float dw = pad_w / 2.0f;
        const float dh = pad_h / 2.0f;
        
        // Estimate bounding boxes for NMS
        for (auto& det : all_detections_buffer) {
            if (det.head_idx != -1) {
                estimate_bbox_from_heatmaps(model_outputs_buffer, det, scale_factor, dw, dh);
            }
        }

        // Apply NMS
        apply_nms(all_detections_buffer, nms_iou_threshold);

        // Decode keypoints for surviving detections
        for (auto& det : all_detections_buffer) {
            if (det.head_idx != -1) {
                decode_pose_keypoints(model_outputs_buffer, det, scale_factor, dw, dh);
            }
        }
        
        RCLCPP_INFO(get_logger(), "Decoded keypoints for %zu detections", all_detections_buffer.size());

        // Pre-compute plane pointers
        const int y_size = frame_width * frame_height;
        uint8_t* y_plane = static_cast<uint8_t*>(image_data_ptr);
        uint8_t* uv_plane = y_plane + y_size;
        
        // Draw all detections
        int kpt_idx = 0;
        for (const auto& det : all_detections_buffer) {
            if (det.head_idx != -1 && !det.keypoints.empty()) {
                draw_keypoints_on_nv12(y_plane, uv_plane, frame_width, frame_height, 
                                      det, 0.5f, kpt_idx);
            }
        }
        
        // Publish keypoints
        for (const auto& det : all_detections_buffer) {
            if (det.head_idx != -1 && !det.keypoints.empty()) {
                publish_keypoints(det, msg->frame_id);
            }
        }

        if (enable_dumps) {
            int output_buffer_size = frame_width * frame_height * 3 / 2;
            std::string filename = "/tmp/yoloposerender-" + std::to_string(recv_counter) + ".bin";
            std::ofstream file(filename, std::ios::binary);
            if (file) {
                file.write(static_cast<const char*>(image_data_ptr), output_buffer_size);
                RCLCPP_DEBUG(this->get_logger(), "Dumped output to %s", filename.c_str());
            }
        }

        simaai::memory_resource::SimaaiMessageInfo info;
        info.num_segments = 1;
        info.sizes = {static_cast<uint32_t>(frame_width * frame_height * 3 / 2)};
        
        simaai_common::msg::SimaaiMessage out_msg;
        out_msg.payload_buf.buffer = msg->payload.buffer;
        out_msg.payload_buf.segments = msg->payload.segments;
        out_msg.payload_buf.gst_memory = msg->payload.gst_memory;
        out_msg.payload_buf.num_segments = msg->payload.num_segments;
        out_msg.frame_id = msg->frame_id;

        SimaMemMsgType image_mla_mem_wrapper;
        image_mla_mem_wrapper.copy_from_ros(msg->payload);
        generic_memory_resource_->simaai_buffer_unmap(image_mla_mem_wrapper);
        publisher_output_->publish(std::move(out_msg));

        frames_processed_++;
        recv_counter++;

        if (!silent) {
            RCLCPP_INFO(get_logger(), "Processed frame %lu, frames_processed=%lu, detections=%zu", 
                       msg->frame_id, frames_processed_, all_detections_buffer.size());
            for (size_t i = 0; i < all_detections_buffer.size(); ++i) {
                const auto& det = all_detections_buffer[i];
                if (det.head_idx != -1) {
                    RCLCPP_INFO(get_logger(), 
                               "  Detection[%zu]: conf=%.3f, class=%d, bbox=[%.1f,%.1f,%.1f,%.1f], keypoints=%zu", 
                               i, det.confidence, det.class_id,
                               det.bbox_x1, det.bbox_y1, det.bbox_x2, det.bbox_y2,
                               det.keypoints.size());
                }
            }
        }
        
        if (recv_counter % 100 == 0) {
            RCLCPP_INFO(get_logger(), "YoloPoseRender stats: %d frames processed", recv_counter);
        }

    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Exception in process_frame: %s", e.what());
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(YoloPoseRender)
