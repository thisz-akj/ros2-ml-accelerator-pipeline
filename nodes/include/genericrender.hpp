#ifndef GENERICRENDER_HPP_
#define GENERICRENDER_HPP_

#include <memory>
#include <simaai/helpers.hpp>
#include <nlohmann/json.hpp>
#include <arm_fp16.h>
#include <iostream>
#include <simaai/boxrender.h>
#include <simaai/boxdecode.h>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "rclcpp/rclcpp.hpp"
#include "simaai_common/msg/simaai_message.hpp"
#include "simaai_common/msg/image.hpp"
#include "memory_generic.hpp"
#include "utils.hpp"


class GenericRender : public rclcpp::Node
{
public:
    GenericRender(const rclcpp::NodeOptions & options);
    bool init(int image_size, int infer_size);
    uint64_t get_num_frames_processed() {return frames_processed_;}
private:
    bool enable_dumps;
    int output_size;
    int image_input_size;
    int overlay_data_input_size;
    size_t queue_size;
    bool silent = true;
    int recv_counter = 0;
    std::string render_type_;
    bool external_publish_;
    simaai::memory_resource::GenericMemoryResource* generic_memory_resource_;
    std::shared_ptr<rclcpp::Publisher<simaai_common::msg::SimaaiMessage>> publisher_;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32MultiArray>> ext_publisher_;
    rclcpp::Subscription<simaai_common::msg::SimaaiMessage>::SharedPtr sub_;
    uint64_t frames_processed_ = 0;
    bool overlay_callback(simaai_common::msg::SimaaiMessage::ConstSharedPtr msg);
    typedef int (*conffunc_t)(int, const char *);
    typedef int (*runfunc_t)(int, void * , int , void *, int , void *, int); 
    conffunc_t dll_config_ = nullptr;
    runfunc_t dll_run_ = nullptr;
    void publish_box_detection(void* output);
    void publish_pose_detections(void* output);
    bool configure_render_type(std::string config_path);

};

#endif // GENERICRENDER_HPP_