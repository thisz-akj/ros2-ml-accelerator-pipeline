#include "allegro_decoder.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <iostream>
#include <fstream>
#include <functional>
#include "genericrender.hpp"

#define YUV420P "YUV420P"
#define NV12 "NV12"
#define ROS_TOPIC "image_raw"


gboolean bus_callback(GstBus *, GstMessage *msg, gpointer data)
{
    auto *node = static_cast<AllegroDecoder *>(data);

    switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_ERROR: {
            GError *err;
            gchar *debug_info;
            gst_message_parse_error(msg, &err, &debug_info);
            RCLCPP_ERROR(node->get_logger(), "GStreamer error: %s", err->message);
            g_clear_error(&err);
            g_free(debug_info);
            break;
        }
        case GST_MESSAGE_EOS:
            RCLCPP_WARN(node->get_logger(), "End of stream");
            break;
        default:
            RCLCPP_INFO(node->get_logger(), "Gstreamer bus called with unknown code");
            break;
    }
    return TRUE;
}

AllegroDecoder::AllegroDecoder(const rclcpp::NodeOptions & options) : Node("allegro_decoder", options)
{
    RCLCPP_INFO(this->get_logger(), "AllegroDecoder created");
    
    queue_size = static_cast<size_t>(declare_parameter<int64_t>("queue_size", 10));
    int latency = this->declare_parameter<int>("latency", 100);
    std::string s_latency        = std::to_string(latency);
    this->generic_memory_resource_ = static_cast<simaai::memory_resource::GenericMemoryResource*>(std::pmr::get_default_resource());
    if (!this->generic_memory_resource_) {
        RCLCPP_ERROR(this->get_logger(), "Can't obtain generic memory resource!");
        throw std::runtime_error("Failed to obtain generic memory resource!");
    }

    //get publish topic
    this->declare_parameter<std::string>("pub_topic",
        "image_raw");
    auto pub_topic = this->get_parameter("pub_topic").as_string();

    this->frame_id = 0;
    this->publisher_ = this->create_publisher<simaai_common::msg::Image>(pub_topic, queue_size);

    // If param is not present in the file
    // It will default to the given value (stream running on a NUC, accessible to all our boards)
    this->declare_parameter<std::string>("rtsp_src",
        "rtsp://192.168.89.119:8554/mystream");
    auto rtsp_src = this->get_parameter("rtsp_src").as_string();
    RCLCPP_DEBUG(this->get_logger(), "rtsp_src: %s", rtsp_src.c_str());

    

    // default to YUV420P
    this->declare_parameter<std::string>("dec_fmt", "YUV420P");
    auto dec_fmt = this->get_parameter("dec_fmt").as_string();
    RCLCPP_DEBUG(this->get_logger(), "dec_fmt: %s", dec_fmt.c_str());

    //dump
    this->declare_parameter<bool>("dump", false);
    this->enable_dump = this->get_parameter("dump").as_bool();
    RCLCPP_DEBUG(this->get_logger(), "dump: %s", this->enable_dump ? "true" : "false");
    
    this->silent = this->declare_parameter<bool>("silent", true);
    gst_init(nullptr, nullptr);

    std::string pipeline_str = "rtspsrc drop-on-latency=true location=" + rtsp_src + " latency=" + s_latency + " ! rtph264depay wait-for-keyframe=true ! h264parse ! queue2 ! video/x-h264, parsed=true, stream-format=(string)byte-stream, alignment=(string)au, width=(int)[1,4096], height=(int)[1,4096] ! simaaidecoder sima-allocator-type=2 name=decoder dec-fmt=" + dec_fmt + " ! queue2 ! appsink name=sink"; 
    
    RCLCPP_INFO(this->get_logger(), "Connecting to rtsp stream... %s", pipeline_str.c_str());

    GError *error = nullptr;
    this->pipeline = gst_parse_launch(pipeline_str.c_str(), &error);
    if (!this->pipeline) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create pipeline: %s", error->message);
        return;
    }

    GstElement *appsink = gst_bin_get_by_name(GST_BIN(this->pipeline), "sink");
    gst_app_sink_set_emit_signals((GstAppSink*)appsink, true);
    g_signal_connect(appsink, "new-sample", G_CALLBACK(on_new_sample), this);
    gst_object_unref(appsink);

    gst_element_set_state(this->pipeline, GST_STATE_PLAYING);

    GstBus *bus = gst_element_get_bus(this->pipeline);
    gst_bus_add_watch(bus, bus_callback, this);
    gst_object_unref(bus);
}

AllegroDecoder::~AllegroDecoder()
{
    RCLCPP_DEBUG(this->get_logger(), "Destructor called");
    gst_element_set_state(this->pipeline, GST_STATE_NULL);
    gst_object_unref(this->pipeline);
    RCLCPP_DEBUG(this->get_logger(), "Destructor successful");
}
struct segment {
  simaai_memory_t *memory;
  std::string name;
};
struct GstSimaaiSegmentMemory {
  GstMemory mem;
  std::vector<segment> segments; ///< SiMa memory segment handles used by simamemlib
  simaai_memory_t **alloc_segments;
  gpointer vaddr;          ///< SiMa memory block virtual address after mapping
  std::mutex map_mutex;    ///< Concurrent map/unmap protection
};

void copy_sima_memory (GstSimaaiSegmentMemory *src, GstSimaaiSegmentMemory *dst) {
  dst->mem = src->mem;
  dst->segments = src->segments;
  dst->alloc_segments = src->alloc_segments;
  dst->vaddr = src->vaddr;
}

GstFlowReturn AllegroDecoder::on_new_sample(GstAppSink *appsink, gpointer user_data) {
    auto *node = static_cast<AllegroDecoder *>(user_data);
    if (!node->silent) RCLCPP_INFO(node->get_logger(), "Allegro Decoder New sample");
    // Get sample object fromm the appsink gst plugin
    GstSample *sample = gst_app_sink_pull_sample(appsink);
    if (!sample) return GST_FLOW_ERROR;
    // get width/height from capabilities, derived from a sample object
    GstCaps *caps = gst_sample_get_caps(sample);
    GstStructure *gst_struct = gst_caps_get_structure(caps, 0);
    int width, height;
    gst_structure_get_int(gst_struct, "width", &width);
    gst_structure_get_int(gst_struct, "height", &height);
    // Get GST buffer from the sample object and unreg the sample (can be deleted)
    GstBuffer *writable_buffer = gst_sample_get_buffer(sample);
    gst_sample_unref(sample);
    // frame processing pace control - discard the current frame if the number of frames in the whole pipeline
    // exceeds a threshold value, by this reduce the load, since anyway all frames cannot be processed 
    if((node->frame_id  - node->discarded_frames - node->render_node_->get_num_frames_processed()) > 30) {    
      // just discard the message      
      node->frame_id++;
      node->discarded_frames++;
      return GST_FLOW_OK;
    }
    
    // Extract a memory object [0] pointer from the gst buffer object - the actual bufferto be replaced with a "fresh" memory, while
    // this memory actual buffer to forward to the next ROS node
    GstSimaaiSegmentMemory * new_mem_for_pool = (GstSimaaiSegmentMemory *)gst_buffer_get_memory (writable_buffer, 0); // get memory desc from gst buffer
    if (new_mem_for_pool == nullptr) {
      RCLCPP_ERROR(node->get_logger(), "In Memory empty");
      return GST_FLOW_ERROR;
    }
    // Create a new memory object to be sent forward
    GstSimaaiSegmentMemory *gst_mem = new(std::nothrow) GstSimaaiSegmentMemory;
    // copy existing memory object to a new memory object that would be sent forward
    copy_sima_memory(new_mem_for_pool, gst_mem); 

    // Allocate new simaai memory as segments to be attached to existing gst buffer pool
    simaai::memory_resource::SimaaiMessageInfo info;
    info.num_segments=1;
    info.sizes = {(uint32_t)(width*height*1.5)};
    // Now replace the pointer inside gst_buffer to a new sima memory
    new_mem_for_pool->alloc_segments = node->generic_memory_resource_->simaai_mem_only_allocate(info); 
     if (new_mem_for_pool->alloc_segments == nullptr) {
      RCLCPP_ERROR(node->get_logger() , "ERROR: allocating segments of memory");
      return GST_FLOW_ERROR;
    }
    // Update other fields in the memory object attached to gst_buffer that depend on attached memory addresses
    new_mem_for_pool->segments[0].memory = new_mem_for_pool->alloc_segments[0];
    new_mem_for_pool->vaddr = simaai_memory_map(new_mem_for_pool->alloc_segments[0]);  
    // send a ROS message with a "stolen" memory content from gst buffer pool, while
    // also send a corresponding memory object pointer for recreation of memory object on the other side to send to encoder
    simaai_common::msg::Image out_msg;
    out_msg.payload.segments = (uint64_t)gst_mem->alloc_segments;
    out_msg.payload.num_segments = 1;
    out_msg.payload.gst_memory = (uint64_t)gst_mem;
    out_msg.frame_id = node->frame_id;
    // print out frame counters
    if (node->frame_id%500 == 0) {
        /*if (!node->silent)*/ RCLCPP_ERROR(node->get_logger(), "%ld frames discarded out of %ld", node->discarded_frames, node->frame_id);
    }   
    if (node->enable_dump){
    // try to map the buffer to a memory address - we do not need this since we do not use the buffer
    // we use the map only if we want to dump a memory buffer to a file
       GstMapInfo map;
       RCLCPP_DEBUG(node->get_logger(), "Checking gst buffer map");
       if (!gst_buffer_map(writable_buffer, &map, GST_MAP_READ)) {
         RCLCPP_ERROR(node->get_logger(), "Failed to map a buffer from frame input");
         return GST_FLOW_ERROR;
        }
        std::string dump_path =  "/tmp/AllegroDecoder_Output_Image_" + std::to_string(node->frame_id) + ".bin";
        std::ofstream out_file(dump_path, std::ios::binary);
        if (out_file.is_open()) {
            out_file.write(reinterpret_cast<const char*>(map.data), map.size);
            out_file.close();
            RCLCPP_DEBUG(node->get_logger(), "Dumped image data to %s", dump_path.c_str());
        } else {
            RCLCPP_ERROR(node->get_logger(), "Failed to open file for writing: %s", dump_path.c_str());
        }
        gst_buffer_unmap(writable_buffer, &map);
    }
    node->publisher_->publish(std::move(out_msg));  
    if (!node->silent) RCLCPP_INFO(node->get_logger(), "Allegro Decoder End - Published data with frame_id: %lu and phys addr: %lu", node->frame_id, simaai_memory_get_phys(gst_mem->alloc_segments[0]));
    //gst_buffer_unref(writable_buffer);
    node->frame_id++;
    return GST_FLOW_OK;
}

RCLCPP_COMPONENTS_REGISTER_NODE(AllegroDecoder)
