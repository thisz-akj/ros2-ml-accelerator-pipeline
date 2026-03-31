#ifndef YoloPoseRender_HPP_
#define YoloPoseRender_HPP_

#include <memory>
#include <nlohmann/json.hpp>
#include <arm_fp16.h>
#include <arm_neon.h>
#include <iostream>
#include <vector>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "rclcpp/rclcpp.hpp"
#include "simaai_common/msg/simaai_message.hpp"
#include "simaai_common/msg/image.hpp"
#include "memory_generic.hpp"
#include "utils.hpp"


// Keypoint structure with alignment for NEON
struct alignas(16) Keypoint {
    float x, y, conf;
    float _pad;  // Padding for 16-byte alignment
};

struct Detection {
    float confidence;
    int x_cell;
    int y_cell;
    int head_idx;
    int class_id;
    std::vector<Keypoint> keypoints;
    
    // Bounding box fields for NMS
    float bbox_x1;
    float bbox_y1;
    float bbox_x2;
    float bbox_y2;
    
    Detection() : confidence(0.0f), x_cell(-1), y_cell(-1), head_idx(-1), class_id(0),
                  bbox_x1(0.0f), bbox_y1(0.0f), bbox_x2(0.0f), bbox_y2(0.0f) {
        keypoints.reserve(17);  // Pre-reserve for typical pose models
    }
};

struct NV12Color {
    uint8_t y;
    uint8_t u;
    uint8_t v;
};

class YoloPoseRender : public rclcpp::Node
{
public:
    YoloPoseRender(const rclcpp::NodeOptions & options);
    ~YoloPoseRender();
    bool init();
private:
    bool enable_dumps;
    size_t queue_size;
    bool silent = true;
    int recv_counter = 0;
    std::string render_type_;
    bool external_publish_;
    
    // Configuration parameters
    int num_keypoints;
    int num_classes;
    int model_width;
    int model_height;
    int frame_width;
    int frame_height;
    float conf_threshold;
    int num_heads;
    int mla_output_size_bytes;

    float nms_iou_threshold = 0.5f;
    
    // Dynamic model output shapes
    std::vector<std::vector<int>> model_out_shapes;
    std::vector<int> head_scales;
    
    // Pre-allocated buffers (OPTIMIZATION: avoid repeated allocations)
    std::vector<float> float32_data_buffer;
    std::vector<std::vector<float>> model_outputs_buffer;
    std::vector<Detection> all_detections_buffer;
    std::vector<bool> nms_keep_flags;  // Pre-allocated for NMS
    std::vector<Detection> nms_filtered_buffer;  // Pre-allocated for NMS filtering
    
    // Pre-allocated NEON-aligned temporary buffers
    alignas(16) std::vector<float> sigmoid_temp_buffer;
    
    int max_elements = 0;
    
    std::mutex sync_mutex_;
    uint64_t last_processed_frame_id_ = 0;
    
    simaai::memory_resource::GenericMemoryResource* generic_memory_resource_;
    
    // Publishers
    std::shared_ptr<rclcpp::Publisher<simaai_common::msg::SimaaiMessage>> publisher_output_;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32MultiArray>> publisher_keypoints_;
    
    // Subscribers
    rclcpp::Subscription<simaai_common::msg::Image>::SharedPtr sub_mla_raw_;
    
    uint64_t frames_processed_ = 0;
    
    void image_mla_callback(simaai_common::msg::Image::ConstSharedPtr msg);
    
    // Optimized helper functions
    void bfloat16_to_float32_neon(const uint16_t* bf16_data, float* f32_data, int num_elements);
    inline float sigmoid(float x);
    void sigmoid_neon(const float* input, float* output, int num_elements);  // NEON-optimized sigmoid
    void get_model_outputs(const float* input_buffer, std::vector<std::vector<float>>& model_outputs);
    void find_all_detections(const std::vector<std::vector<float>>& model_outputs, 
                            float threshold, std::vector<Detection>& detections);
    void decode_pose_keypoints(const std::vector<std::vector<float>>& model_outputs,
                              Detection& det,
                              float scale_factor,
                              float dw,
                              float dh);
    void draw_keypoints_on_nv12(uint8_t* y_plane, uint8_t* uv_plane,
                               int width, int height,
                               const Detection& det,
                               float kpt_threshold = 0.5f, int kpt_idx = 0);
    void publish_keypoints(const Detection& det, uint64_t frame_id);
    bool configure_render_type(std::string config_path);
    
    void estimate_bbox_from_heatmaps(const std::vector<std::vector<float>>& model_outputs,
                                  Detection& det,
                                  float scale_factor,
                                  float dw,
                                  float dh);

    float calculate_bbox_iou(const Detection& det1, const Detection& det2);
    void apply_nms(std::vector<Detection>& detections, float iou_threshold);

};

#endif // YoloPoseRender_HPP_
