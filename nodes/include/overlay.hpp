#ifndef OPENPOSE__OVERLAY_HPP_
#define OPENPOSE__OVERLAY_HPP_

#include <opencv2/opencv.hpp>
#include <simaai/cv_helpers.h>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/strategies/allocator_memory_strategy.hpp"

#include "overlay.hpp"
#include "simaai_common/msg/image.hpp"
#include "simaai_common/msg/simaai_message.hpp"


#include <nlohmann/json.hpp>

#include <dispatcherbase.hh>
#include <dispatcherfactory.hh>

// SiMa memory resource
#include "memory_generic.hpp"

class Overlay : public rclcpp::Node
{
public:
    Overlay(const rclcpp::NodeOptions & options);
private:
    void image_callback(simaai_common::msg::Image::ConstSharedPtr msg);
    bool update_renderer(unsigned char * pixels, unsigned int width, unsigned int height);
    bool keypoints_callback(simaai_common::msg::SimaaiMessage::ConstSharedPtr msg);
    bool process_and_draw(const std::vector<std::vector<float>> & pose_entries,
                                  const std::vector<std::vector<float>> & all_keypoints);
    std::vector<unsigned char> read_dump_file(std::string dump_file);
    simaai::memory_resource::GenericMemoryResource * generic_memory_resource_;

    rclcpp::Subscription<simaai_common::msg::Image>::SharedPtr frame_sub_;
    rclcpp::Subscription<simaai_common::msg::SimaaiMessage>::SharedPtr keypoints_sub_;
    // rclcpp::Publisher<simaai_common::msg::Image>::SharedPtr publisher_;
    rclcpp::Publisher<simaai_common::msg::SimaaiMessage>::SharedPtr publisher_;
    
    std::vector<unsigned char> pixels_;
    cv::Mat y_, u_, v_; //!< y,u,v channels of the input image
    int y_stride_ = 0, u_stride_ = 0, v_stride_ = 0;
    bool have_frame_{false}, have_keypoints_{false};
    uint64_t last_frame_id_{0};
    unsigned frame_width, frame_height;
    bool enable_dump;
    bool silent{true};


    // Variables for FPS measurement
    uint64_t frame_count_{0};
    rclcpp::Time fps_start_time_;
};

#endif
