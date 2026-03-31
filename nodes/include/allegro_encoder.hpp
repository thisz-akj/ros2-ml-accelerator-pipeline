#ifndef SIMAAI__NODES__ALLEGRO_ENCODER_HPP_
#define SIMAAI__NODES__ALLEGRO_ENCODER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/strategies/allocator_memory_strategy.hpp"
#include <nlohmann/json.hpp>

#include <dispatcherbase.hh>
#include <dispatcherfactory.hh>

#include "simaai_common/msg/simaai_buffer.hpp"
#include "simaai_common/msg/simaai_message.hpp"
// SiMa memory resource
#include "memory_generic.hpp"

#include <gst/gst.h>
#include <gst/app/gstappsink.h>



class AllegroEncoder : public rclcpp::Node
{
public:
    explicit AllegroEncoder(const rclcpp::NodeOptions & options);
    ~AllegroEncoder();
private:
    GstElement *pipeline;
    GstElement *appsrc;
    bool enable_dump;
    size_t queue_size;
    int image_size;
    bool silent{true};
    simaai::memory_resource::GenericMemoryResource* generic_memory_resource_;
    rclcpp::Subscription<simaai_common::msg::SimaaiMessage>::SharedPtr subscription_;
    void image_callback(simaai_common::msg::SimaaiMessage::ConstSharedPtr msg);
};

#endif  // SIMAAI__NODES__ALLEGRO_ENCODER_HPP_