#ifndef SIMAAI__NODES__ALLEGRO_DECODER_HPP_
#define SIMAAI__NODES__ALLEGRO_DECODER_HPP_


#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/strategies/allocator_memory_strategy.hpp"
#include <nlohmann/json.hpp>

#include <dispatcherbase.hh>
#include <dispatcherfactory.hh>

// SiMa memory resource
#include "memory_generic.hpp"

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <chrono>


#include "simaai_common/msg/simaai_buffer.hpp"
#include "simaai_common/msg/simaai_message.hpp"
#include "simaai_common/msg/image.hpp"


using Alloc = std::pmr::polymorphic_allocator<void>;


static constexpr std::size_t MESSAGE_POOL_SIZE = 10;
class GenericRender;
class AllegroDecoder : public rclcpp::Node
{
public:
    AllegroDecoder(const rclcpp::NodeOptions & options);
    ~AllegroDecoder();
    void set_connected_render(GenericRender *render) {render_node_ = render;}
private:
    static GstFlowReturn on_new_sample(GstAppSink *appsink, gpointer user_data);

    GstElement *pipeline;
    uint64_t frame_id = 0;
    uint64_t discarded_frames = 0;
    bool enable_dump = false;
    size_t queue_size;
    bool silent = true;
    GenericRender *render_node_;
    simaai::memory_resource::GenericMemoryResource* generic_memory_resource_;
    std::shared_ptr<rclcpp::Publisher<simaai_common::msg::Image>> publisher_;
};

#endif // SIMAAI__NODES__ALLEGRO_DECODER_HPP_
