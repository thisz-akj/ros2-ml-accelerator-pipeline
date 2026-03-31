#ifndef SIMAAI__NODES__PROCESS_SIMA_YOLOPOSE_HPP_
#define SIMAAI__NODES__PROCESS_SIMA_YOLOPOSE_HPP_

#include <memory>
#include <nlohmann/json.hpp>
#include <dispatcherbase.hh>
#include <dispatcherfactory.hh>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <chrono>
#include <queue>
#include <mutex>

// SiMa memory resource
#include "memory_generic.hpp"
#include "simaai_common/msg/simaai_buffer.hpp"
#include "simaai_common/msg/simaai_message.hpp"
#include "simaai_common/msg/image.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/strategies/allocator_memory_strategy.hpp"

using Alloc = std::pmr::polymorphic_allocator<void>;

static constexpr std::size_t MESSAGE_POOL_SIZE = 10;
class YoloPoseRender;
class ProcessSimaYoloPose : public rclcpp::Node
{
public:
    ProcessSimaYoloPose(const rclcpp::NodeOptions & options);
    ~ProcessSimaYoloPose();
    void set_connected_render(YoloPoseRender *render) {render_node_ = render;}
    bool update_model_path_in_json(
        const std::string& json_file_path,
        const std::string& package_dir,
        const std::string& hardware_target);
private:
    static GstFlowReturn on_new_sample_mla(GstAppSink *appsink, gpointer user_data);

    GstElement *pipeline;
    uint64_t frame_id = 0;
    uint64_t mla_frame_id = 0;
    uint64_t discarded_frames = 0;
    uint64_t mla_discarded_frames = 0;
    bool enable_dump = false;
    size_t queue_size;
    bool silent = true;
    YoloPoseRender *render_node_;
    simaai::memory_resource::GenericMemoryResource* generic_memory_resource_;
    std::shared_ptr<rclcpp::Publisher<simaai_common::msg::Image>> publisher_mla_raw_;
    
    std::queue<uint64_t> mla_frame_id_queue_;
    std::mutex mla_frame_queue_mutex_;
};

#endif // SIMAAI__NODES__PROCESS_SIMA_YOLOPOSE_HPP_
