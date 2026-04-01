#include "allegro_encoder.hpp"
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <cstring>
#include <rclcpp_components/register_node_macro.hpp>

/*
"appsrc ! 'video/x-raw,format=NV12,width=1280,height=720,framerate=30/1' ! simaaiencoder enc-bitrate=4000 name=encoder1 ! h264parse ! rtph264pay ! udpsink host=<HOST_IP> port=<HOST_PORT> "
*/

AllegroEncoder::AllegroEncoder(const rclcpp::NodeOptions & options)
: Node("allegro_encoder", options)
{
    RCLCPP_INFO(this->get_logger(), "AllegroEncoder created");

    this->generic_memory_resource_ = static_cast<simaai::memory_resource::GenericMemoryResource*>(std::pmr::get_default_resource());
    if (!this->generic_memory_resource_) {
        RCLCPP_ERROR(this->get_logger(), "Can't obtain generic memory resource!");
        throw std::runtime_error("Failed to obtain generic memory resource!");
    }
    silent = this->declare_parameter<bool>("silent", true);
    std::string sub_topic = this->declare_parameter("sub_topic", std::string("encoder_input_topic"));
    gst_init(nullptr, nullptr);

    queue_size = static_cast<size_t>(declare_parameter<int64_t>("queue_size", 10));
    int bitrate = declare_parameter<int>("bitrate", 4000);
    int width = declare_parameter<int>("width", 1280);
    int height = declare_parameter<int>("height", 720);
    int fps = declare_parameter<int>("fps", 30);
    image_size = {(int32_t)(width*height*1.5)};
    std::string ip = declare_parameter<std::string>("ip", "127.0.0.1");
    int port = declare_parameter<int>("port", 7000);
    std::string format = declare_parameter<std::string>("format", "NV12");
    std::string input_topic_ = declare_parameter<std::string>("input_topic", "image_raw");

    std::string pipeline_str =
        "appsrc name=src is-live=true format=3 ! "
        "video/x-raw,format=" + format +
        ",width=" + std::to_string(width) +
        ",height=" + std::to_string(height) +
        ",framerate=" + std::to_string(fps) + "/1 ! "
        "simaaiencoder enc-bitrate=" + std::to_string(bitrate) +
        " name=encoder1 !"
    //    " fpssink";
        "h264parse ! rtph264pay ! "
        "udpsink host=" + ip + " port=" + std::to_string(port);

    RCLCPP_INFO(this->get_logger(), "Pipeline: %s", pipeline_str.c_str());

    GError *error = nullptr;
    pipeline = gst_parse_launch(pipeline_str.c_str(), &error);
    if (!pipeline || error) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create GStreamer pipeline: %s", error ? error->message : "unknown error");
        g_clear_error(&error);
        throw std::runtime_error("Failed to create GStreamer pipeline");
    }

    appsrc = gst_bin_get_by_name(GST_BIN(pipeline), "src");
    gst_element_set_state(pipeline, GST_STATE_PLAYING);


    subscription_ = this->create_subscription<simaai_common::msg::SimaaiMessage>(
        sub_topic, queue_size,
        std::bind(&AllegroEncoder::image_callback, this, std::placeholders::_1));
}

AllegroEncoder::~AllegroEncoder()
{
    if (pipeline) {
        gst_element_set_state(pipeline, GST_STATE_NULL);
        gst_object_unref(pipeline);
    }
}



void AllegroEncoder::image_callback(simaai_common::msg::SimaaiMessage::ConstSharedPtr msg)
{
    static auto t_start = std::chrono::steady_clock::now();
    static uint64_t count = 0;
    count++;
    if (!silent) RCLCPP_INFO(this->get_logger(), "Received image with frame_id %lu", msg->frame_id);
    // allocate a new gstBuffer to send to a GST pipeline - we will need to attach existing memory to it
    GstBuffer *buffer = gst_buffer_new();    
    //retreive gst memory object that was sent to us by allegro_decoder ROS node. The object just exists in memory but 
    // the pointer to it was passed by copy through ROS and C++ messages - recasting and copying it from message to message  
    GstSimaaiSegmentMemory * mem_out = (GstSimaaiSegmentMemory *)(msg->payload_buf.gst_memory);
    if (mem_out == nullptr) {
      RCLCPP_ERROR(this->get_logger(), "GstSimaaiSegmentMemory is null");
      return;
    }
    GstMemory *g_mem = (GstMemory *)mem_out; // just a casted same pointer
    // reinit the memory object that we received in the message from allegro_decoder. This is just to reset several
    // important fields like reference count, parent etc. Otherwise, use the same values form the original object
    gst_memory_init (g_mem, (GstMemoryFlags)(((GstMiniObject*)g_mem)->flags), g_mem->allocator, nullptr, g_mem->maxsize, g_mem->align, g_mem->offset, g_mem->size);
    // attach a memory object to a gst_buffer
    gst_buffer_append_memory(buffer, (GstMemory *)mem_out);
    GstFlowReturn ret;
    // send buffer up the gst pipeline. The actual sima memory will be deallocated in the receiving gst plugin - simaaiencoder, when the reference counter will reach 0
    g_signal_emit_by_name(appsrc, "push-buffer", buffer, &ret);
    gst_buffer_unref(buffer);
    if (ret != GST_FLOW_OK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to push buffer to appsrc");
    }
    // frame rate printouts
    if(count%500 == 0) {
      auto t_now = std::chrono::steady_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(t_now-t_start);
      /*if (!silent)*/ RCLCPP_INFO(this->get_logger(), "Allegro Encoder End, Rate: %2f fps, %2f fps", (float)msg->frame_id * 1000000/elapsed.count(), (float)count * 1000000/elapsed.count());
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(AllegroEncoder)
