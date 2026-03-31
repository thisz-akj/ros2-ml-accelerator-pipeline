#include <rclcpp_components/register_node_macro.hpp>
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <functional>
#include <simaai/helpers.hpp>
#include "yolopose_render.hpp"
#include "process_sima_yolopose.hpp"

#define YUV420P "YUV420P"
#define NV12 "NV12"
#define ROS_TOPIC "image_raw"

using json = nlohmann::json;
namespace fs = std::filesystem;

gboolean bus_callback(GstBus *, GstMessage *msg, gpointer data)
{
    auto *node = static_cast<ProcessSimaYoloPose *>(data);

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

bool ProcessSimaYoloPose::update_model_path_in_json(
    const std::string& json_file_path,
    const std::string& package_dir,
    const std::string& hardware_target)
{
    // 1. Open JSON file
    std::ifstream ifs(json_file_path);
    if (!ifs.is_open()) {
        std::cerr << "Failed to open JSON file: " << json_file_path << std::endl;
        return false;
    }

    // 2. Parse JSON
    json config;
    try {
        ifs >> config;
    } catch (const json::parse_error& e) {
        std::cerr << "JSON parse error: " << e.what() << std::endl;
        return false;
    }
    ifs.close();

    // 3. Construct new model path
    fs::path model_path =
        fs::path(package_dir) /
        "resources" /
        "processmla" /
        hardware_target /
        "yolov11_stage1_mla.elf";

    // 4. Replace value
    config["simaai__params"]["model_path"] = model_path.string();

    // 5. Write JSON back to file
    std::ofstream ofs(json_file_path);
    if (!ofs.is_open()) {
        std::cerr << "Failed to write JSON file: " << json_file_path << std::endl;
        return false;
    }

    ofs << config.dump(4);  // pretty print with indentation
    ofs.close();

    return true;
}

ProcessSimaYoloPose::ProcessSimaYoloPose(const rclcpp::NodeOptions & options) : Node("process_sima_yolopose", options)
{
    RCLCPP_INFO(this->get_logger(), "ProcessSimaYoloPose created");
    std::string hardware_target = this->declare_parameter("hardware_target", "");
    std::string package_dir = this->declare_parameter("package_dir", "");

    RCLCPP_INFO(this->get_logger(),
                "hardware_target: '%s'", hardware_target.c_str());
    RCLCPP_INFO(this->get_logger(),
                "package_dir: '%s'", package_dir.c_str());

    this->generic_memory_resource_ = static_cast<simaai::memory_resource::GenericMemoryResource*>(std::pmr::get_default_resource());
    if (!this->generic_memory_resource_) {
        RCLCPP_ERROR(this->get_logger(), "Can't obtain generic memory resource!");
        throw std::runtime_error("Failed to obtain generic memory resource!");
    }

    //get publish topic
     this->declare_parameter<std::string>("pub_topic_mla",
        "mla_raw");
    auto pub_topic_mla = this->get_parameter("pub_topic_mla").as_string();
    this->frame_id = 0;
    this->mla_frame_id = 0;
    
    // Create two publishers
    this->publisher_mla_raw_ = this->create_publisher<simaai_common::msg::Image>(pub_topic_mla, queue_size);
    
    // Declare parameters
    this->declare_parameter<std::string>("cvu_file_path","");
    this->declare_parameter<std::string>("mla_file_path","");
    this->declare_parameter<std::string>("bc_file_3_path","");
    this->declare_parameter<std::string>("bc_file_4_path","");
    this->declare_parameter<std::string>("bc_file_5_path","");
    this->declare_parameter<std::string>("bc_file_6_path","");
    this->declare_parameter<std::string>("bc_file_7_path","");
    this->declare_parameter<std::string>("bc_file_8_path","");
    this->declare_parameter<std::string>("bc_file_mux_path","");
    this->declare_parameter<std::string>("image_mla_buffer_concat_path","");

    this->declare_parameter<std::string>("dt_file_3_path","");
    this->declare_parameter<std::string>("dt_file_4_path","");
    this->declare_parameter<std::string>("dt_file_5_path","");
    this->declare_parameter<std::string>("dt_file_6_path","");
    this->declare_parameter<std::string>("dt_file_7_path","");
    this->declare_parameter<std::string>("dt_file_8_path","");


    std::string cvu_file_path = this->get_parameter("cvu_file_path").as_string();
    cvu_file_path = resolve_package_resource_path(cvu_file_path, hardware_target, package_dir);
    std::string mla_file_path = this->get_parameter("mla_file_path").as_string();
    mla_file_path = resolve_package_resource_path(mla_file_path, hardware_target, package_dir);
    std::string bc_file_3_path = this->get_parameter("bc_file_3_path").as_string();
    bc_file_3_path = resolve_package_resource_path(bc_file_3_path, hardware_target, package_dir);
    std::string bc_file_4_path = this->get_parameter("bc_file_4_path").as_string();
    bc_file_4_path = resolve_package_resource_path(bc_file_4_path, hardware_target, package_dir);
    std::string bc_file_5_path = this->get_parameter("bc_file_5_path").as_string();
    bc_file_5_path = resolve_package_resource_path(bc_file_5_path, hardware_target, package_dir);
    std::string bc_file_6_path = this->get_parameter("bc_file_6_path").as_string();
    bc_file_6_path = resolve_package_resource_path(bc_file_6_path, hardware_target, package_dir);
    std::string bc_file_7_path = this->get_parameter("bc_file_7_path").as_string();
    bc_file_7_path = resolve_package_resource_path(bc_file_7_path, hardware_target, package_dir);
    std::string bc_file_8_path = this->get_parameter("bc_file_8_path").as_string();
    bc_file_8_path = resolve_package_resource_path(bc_file_8_path, hardware_target, package_dir);
    std::string bc_file_mux_path = this->get_parameter("bc_file_mux_path").as_string(); 
    bc_file_mux_path = resolve_package_resource_path(bc_file_mux_path, hardware_target, package_dir);
    std::string image_mla_buffer_concat_path = this->get_parameter("image_mla_buffer_concat_path").as_string();
    image_mla_buffer_concat_path = resolve_package_resource_path(image_mla_buffer_concat_path, hardware_target, package_dir);
    std::string dt_file_3_path = this->get_parameter("dt_file_3_path").as_string();
    dt_file_3_path = resolve_package_resource_path(dt_file_3_path, hardware_target, package_dir);
    std::string dt_file_4_path = this->get_parameter("dt_file_4_path").as_string();
    dt_file_4_path = resolve_package_resource_path(dt_file_4_path, hardware_target, package_dir);
    std::string dt_file_5_path = this->get_parameter("dt_file_5_path").as_string();
    dt_file_5_path = resolve_package_resource_path(dt_file_5_path, hardware_target, package_dir);
    std::string dt_file_6_path = this->get_parameter("dt_file_6_path").as_string();
    dt_file_6_path = resolve_package_resource_path(dt_file_6_path, hardware_target, package_dir);
    std::string dt_file_7_path = this->get_parameter("dt_file_7_path").as_string();
    dt_file_7_path = resolve_package_resource_path(dt_file_7_path, hardware_target, package_dir);
    std::string dt_file_8_path = this->get_parameter("dt_file_8_path").as_string();
    dt_file_8_path = resolve_package_resource_path(dt_file_8_path, hardware_target, package_dir);

    //std::string model_path = package_dir + "/resources/processmla/" + hardware_target + "/yolov11_stage1_mla.elf";
    update_model_path_in_json(mla_file_path, package_dir, hardware_target);

    // If param is not present in the file
    // It will default to the given value (stream running on a NUC, accessible to all our boards)
    this->declare_parameter<std::string>("input_stream_from",
        "rtsp");
    auto input_stream_from = this->get_parameter("input_stream_from").as_string();
    RCLCPP_DEBUG(this->get_logger(), "input_stream_from: %s", input_stream_from.c_str());
    
    // default to YUV420P
    this->declare_parameter<std::string>("dec_fmt", "YUV420P");
    auto dec_fmt = this->get_parameter("dec_fmt").as_string();
    RCLCPP_DEBUG(this->get_logger(), "dec_fmt: %s", dec_fmt.c_str());

    this->declare_parameter<int>("width", 1280);
    auto width = this->get_parameter("width").as_int();
    RCLCPP_DEBUG(this->get_logger(), "width: %ld", width);

    this->declare_parameter<int>("height", 720);
    auto height = this->get_parameter("height").as_int();
    RCLCPP_DEBUG(this->get_logger(), "height: %ld", height);

    this->declare_parameter<bool>("dump", false);
    this->enable_dump = this->get_parameter("dump").as_bool();
    RCLCPP_DEBUG(this->get_logger(), "dump: %s", this->enable_dump ? "true" : "false");
    
        
    queue_size = static_cast<size_t>(declare_parameter<int64_t>("queue_size", 10));

    this->silent = this->declare_parameter<bool>("silent", true);

    std::string pipeline_str;

    /* -------------------------------------------------
     * 2. RTSP source
     * ------------------------------------------------- */
    if (input_stream_from == "rtsp") {

      this->declare_parameter<std::string>( "input_src", "rtsp://127.0.0.1:8554/stream");
      this->declare_parameter<int>("latency", 100);

      std::string input_src = this->get_parameter("input_src").as_string();
      int latency = this->get_parameter("latency").as_int();

      std::string s_latency = std::to_string(latency);

      RCLCPP_INFO(this->get_logger(), "RTSP selected");
      RCLCPP_INFO(this->get_logger(), "input_src: %s", input_src.c_str());
      RCLCPP_INFO(this->get_logger(), "latency: %d", latency);

      pipeline_str =
      "rtspsrc drop-on-latency=true location=" + input_src +
      " latency=" + s_latency +
      " ! rtph264depay wait-for-keyframe=true "
      " ! h264parse "
      " ! queue2 "
      " ! video/x-h264, parsed=true, stream-format=(string)byte-stream,"
      " alignment=(string)au, width=(int)[1,4096], height=(int)[1,4096] "
      " ! simaaidecoder sima-allocator-type=2 name=decoder dec-fmt=" + dec_fmt +
      " ! queue2 "
      " ! tee name=source ";

    }

    /* -------------------------------------------------
     * 3. USB Camera source
     * ------------------------------------------------- */
    else if (input_stream_from == "usbcam") {

      this->declare_parameter<std::string>("input_src", "/dev/video0");
      this->declare_parameter<int>("fps", 30);

      std::string input_src = this->get_parameter("input_src").as_string();
      int fps = this->get_parameter("fps").as_int();

      RCLCPP_INFO(this->get_logger(), "USB camera selected");
      RCLCPP_INFO(this->get_logger(), "input_src: %s", input_src.c_str());
      RCLCPP_INFO(this->get_logger(), "fps: %d", fps);

      pipeline_str =
      "v4l2src device=" + input_src + " io-mode=4 "
      " ! video/x-raw,format=YUY2,width=" + std::to_string(width) + ",height=" + std::to_string(height) + ","
      " framerate=" + std::to_string(fps) + "/1,"
      " interlace-mode=progressive "
      " ! videoconvert qos=true "
      " ! video/x-raw,format=I420 "
      " ! openh264enc bitrate=4000000 rate-control=1 complexity=0 "
      " gop-size=" + std::to_string(fps) +
      " ! h264parse config-interval=1 "
      " ! video/x-h264, parsed=true, stream-format=(string)byte-stream,"
      " alignment=(string)au "
      " ! queue2 "
      " ! simaaidecoder sima-allocator-type=2 name=decoder dec-fmt=" + dec_fmt +
      " ! queue2 "
      " ! tee name=source ";

    }

    /* -------------------------------------------------
     * 4. SIMAAI source
     * ------------------------------------------------- */
    else if (input_stream_from == "simaaisrc") {

      this->declare_parameter<std::string>( "input_src", "");
      this->declare_parameter<int>("delay", 100);

      std::string input_src = this->get_parameter("input_src").as_string();
      int delay = this->get_parameter("delay").as_int();

      RCLCPP_INFO(this->get_logger(), "SIMAAI source selected");
      RCLCPP_INFO(this->get_logger(), "input_src: %s", input_src.c_str());
      RCLCPP_INFO(this->get_logger(), "delay: %d", delay);

      pipeline_str =
      "simaaisrc location=" + input_src +
      " node-name=decoder mem-target=0 index=1 loop=true"
      " delay=" + std::to_string(delay) +
      " ! video/x-raw, format=(string)NV12,"
      " width=(int)" + std::to_string(width) +
      ", height=(int)" + std::to_string(height) +
      " ! tee name=source ";
    }

    /* -------------------------------------------------
     * 5. Invalid value
     * ------------------------------------------------- */
    else {
      RCLCPP_ERROR(this->get_logger(),
                   "Invalid input_stream_from: %s "
                   "(expected: rtsp | usbcam | simaaisrc)",
                   input_stream_from.c_str());
    }

    gst_init(nullptr, nullptr);

    pipeline_str +=
    // ------------------------------------------------------------------------
    // 2. MLA PRE-PROCESSING (Common Path)
    // ------------------------------------------------------------------------
    " source. ! queue2 "
    " ! simaaiprocesscvu "
    "     config=" + cvu_file_path + " "
    "     num-buffers=5 name=simaai_preprocess "
    " ! simaaiprocessmla "
    "     config=" + mla_file_path + " "
    "     num-buffers=5 name=simaai_process_mla "
    " ! tee name=mla_source "

    // ------------------------------------------------------------------------
    // 3. HEAD 3 -> MUXER -> FINAL CONCAT (The "Spine" that creates downstream elements)
    // ------------------------------------------------------------------------
    " mla_source. ! queue2 "
    " ! buffer_concatenator sima-allocator-type=2 "
    "     config=" + bc_file_3_path + " "
    "     name=simaai_buffer_concatenator_3 "
    " ! simaaiprocesscvu num-buffers=5 name=simaai_detesselate_3 "
    "     config=" + dt_file_3_path + " "

    // Create the MUXER here
    " ! buffer_concatenator sima-allocator-type=2 "
    "     config=" + bc_file_mux_path + " "
    "     name=simaai_buffer_concatenator_muxer "

    // Create the FINAL CONCATENATOR here (Image + MLA)
    " ! buffer_concatenator sima-allocator-type=2 "
    "     config=" + image_mla_buffer_concat_path + " "
    "     name=simaai_buffer_concatenator_image_mla "
    " ! queue2 "
    " ! appsink name=sink_mla "

    // ------------------------------------------------------------------------
    // 4. OTHER MLA HEADS (Linking to the now-existing Muxer)
    // ------------------------------------------------------------------------
    " mla_source. ! queue2 "
    " ! buffer_concatenator sima-allocator-type=2 config=" + bc_file_4_path + " name=simaai_buffer_concatenator_4 "
    " ! simaaiprocesscvu num-buffers=5 config=" + dt_file_4_path + " name=simaai_detesselate_4 "
    " ! simaai_buffer_concatenator_muxer. "

    " mla_source. ! queue2 "
    " ! buffer_concatenator sima-allocator-type=2 config=" + bc_file_5_path + " name=simaai_buffer_concatenator_5 "
    " ! simaaiprocesscvu num-buffers=5 config=" + dt_file_5_path + " name=simaai_detesselate_5 "
    " ! simaai_buffer_concatenator_muxer. "

    " mla_source. ! queue2 "
    " ! buffer_concatenator sima-allocator-type=2 config=" + bc_file_6_path + " name=simaai_buffer_concatenator_6 "
    " ! simaaiprocesscvu num-buffers=5 config=" + dt_file_6_path + " name=simaai_detesselate_6 "
    " ! simaai_buffer_concatenator_muxer. "

    " mla_source. ! queue2 "
    " ! buffer_concatenator sima-allocator-type=2 config=" + bc_file_7_path + " name=simaai_buffer_concatenator_7 "
    " ! simaaiprocesscvu num-buffers=5 config=" + dt_file_7_path + " name=simaai_detesselate_7 "
    " ! simaai_buffer_concatenator_muxer. "

    " mla_source. ! queue2 "
    " ! buffer_concatenator sima-allocator-type=2 config=" + bc_file_8_path + " name=simaai_buffer_concatenator_8 "
    " ! simaaiprocesscvu num-buffers=5 config=" + dt_file_8_path + " name=simaai_detesselate_8 "
    " ! simaai_buffer_concatenator_muxer. "

    // ------------------------------------------------------------------------
    // 5. IMAGE BYPASS (Linking to the now-existing Final Concat)
    // ------------------------------------------------------------------------
    " source. ! queue2 "
    " ! simaai_buffer_concatenator_image_mla. ";
    
    RCLCPP_INFO(this->get_logger(), "Connecting to rtsp stream... %s", pipeline_str.c_str());

    GError *error = nullptr;
    this->pipeline = gst_parse_launch(pipeline_str.c_str(), &error);
    if (!this->pipeline) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create pipeline: %s", error->message);
        return;
    }

    // Setup appsink for MLA output (mla_raw)
    GstElement *appsink_mla = gst_bin_get_by_name(GST_BIN(this->pipeline), "sink_mla");
    if (appsink_mla) {
        gst_app_sink_set_emit_signals((GstAppSink*)appsink_mla, true);
        g_signal_connect(appsink_mla, "new-sample", G_CALLBACK(on_new_sample_mla), this);
        gst_object_unref(appsink_mla);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to find sink_mla");
    }

    gst_element_set_state(this->pipeline, GST_STATE_PLAYING);

    GstBus *bus = gst_element_get_bus(this->pipeline);
    gst_bus_add_watch(bus, bus_callback, this);
    gst_object_unref(bus);
}

ProcessSimaYoloPose::~ProcessSimaYoloPose()
{
    RCLCPP_DEBUG(this->get_logger(), "Destructor called");
    
    // CRITICAL FIX: Clean up MLA frame_id queue
    {
        std::lock_guard<std::mutex> lock(mla_frame_queue_mutex_);
        if (!mla_frame_id_queue_.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "Cleaning up %zu frame_ids from MLA queue", mla_frame_id_queue_.size());
            while (!mla_frame_id_queue_.empty()) {
                mla_frame_id_queue_.pop();
            }
        }
    }
    
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

// Callback for MLA output (mla_raw topic)
GstFlowReturn ProcessSimaYoloPose::on_new_sample_mla(GstAppSink *appsink, gpointer user_data) {
    auto *node = static_cast<ProcessSimaYoloPose *>(user_data);
    if (!node->silent) RCLCPP_INFO(node->get_logger(), "MLA New sample");
    
    // Get sample object from the appsink gst plugin
    GstSample *sample = gst_app_sink_pull_sample(appsink);
    if (!sample) {
        RCLCPP_ERROR(node->get_logger(), "Failed to pull MLA sample");
        return GST_FLOW_ERROR;
    }
    
    // Get GST buffer from the sample object
    GstBuffer *writable_buffer = gst_sample_get_buffer(sample);
    if (!writable_buffer) {
        RCLCPP_ERROR(node->get_logger(), "Failed to get buffer from MLA sample");
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }
    
    // Get the actual buffer size from GStreamer
    // This size is defined in image_mla_buffer.json i.e. 2037600  [1382400 image] [655200 bf16 detess tensors]
    gsize buffer_size = gst_buffer_get_size(writable_buffer);
    
    // Log the size on first frame or periodically for verification
    static bool first_mla_frame = true;
    if (first_mla_frame) {
        RCLCPP_INFO(node->get_logger(), "MLA output buffer size: %zu bytes (expected: 1696800)", buffer_size);
        first_mla_frame = false;
    }
    
    // Unreference the sample (we still have the buffer)
    gst_sample_unref(sample);
    
    // Extract memory object from the gst buffer
    GstSimaaiSegmentMemory *new_mem_for_pool = (GstSimaaiSegmentMemory *)gst_buffer_get_memory(writable_buffer, 0);
    if (new_mem_for_pool == nullptr) {
        RCLCPP_ERROR(node->get_logger(), "Failed to get memory from MLA buffer");
        return GST_FLOW_ERROR;
    }
    
    // Create a new memory object to be sent forward via ROS
    GstSimaaiSegmentMemory *gst_mem = new(std::nothrow) GstSimaaiSegmentMemory;
    if (gst_mem == nullptr) {
        RCLCPP_ERROR(node->get_logger(), "Failed to allocate GstSimaaiSegmentMemory for MLA");
        gst_memory_unref((GstMemory*)new_mem_for_pool);
        return GST_FLOW_ERROR;
    }
    
    // Copy existing memory descriptor to new memory object
    copy_sima_memory(new_mem_for_pool, gst_mem); 

    // Allocate new simaai memory to replace the buffer in the GStreamer pool
    // This uses the ACTUAL buffer size from GStreamer (should be 1696800 bytes)
    simaai::memory_resource::SimaaiMessageInfo info;
    info.num_segments = 1;
    info.sizes = {static_cast<uint32_t>(buffer_size)};
    
    new_mem_for_pool->alloc_segments = node->generic_memory_resource_->simaai_mem_only_allocate(info); 
    if (new_mem_for_pool->alloc_segments == nullptr) {
        RCLCPP_ERROR(node->get_logger(), 
                     "ERROR: Failed to allocate MLA memory - requested: %zu bytes (%.2f MB)", 
                     buffer_size, buffer_size / (1024.0 * 1024.0));
        RCLCPP_ERROR(node->get_logger(), "This may indicate insufficient memory or memory fragmentation");
        delete gst_mem;
        gst_memory_unref((GstMemory*)new_mem_for_pool);
        return GST_FLOW_ERROR;
    }
    
    // Update the memory descriptor with the new allocation
    new_mem_for_pool->segments[0].memory = new_mem_for_pool->alloc_segments[0];
    new_mem_for_pool->vaddr = simaai_memory_map(new_mem_for_pool->alloc_segments[0]);  
    
    if (new_mem_for_pool->vaddr == nullptr) {
        RCLCPP_ERROR(node->get_logger(), "Failed to map MLA memory");
        simaai_memory_free(new_mem_for_pool->alloc_segments[0]);
        free(new_mem_for_pool->alloc_segments);
        delete gst_mem;
        gst_memory_unref((GstMemory*)new_mem_for_pool);
        return GST_FLOW_ERROR;
    }
    
    // Create ROS message with the "stolen" memory from GStreamer pool
    simaai_common::msg::Image out_msg;
    out_msg.payload.segments = (uint64_t)gst_mem->alloc_segments;
    out_msg.payload.num_segments = 1;
    out_msg.payload.gst_memory = (uint64_t)gst_mem;
    out_msg.frame_id = node->mla_frame_id;  // CRITICAL FIX: Use matched frame_id instead of mla_frame_id
    
    // Periodic logging with synchronization status
    if (node->mla_frame_id % 500 == 0) {
        std::lock_guard<std::mutex> lock(node->mla_frame_queue_mutex_);
        RCLCPP_INFO(node->get_logger(), 
                   "MLA: %ld frames processed, %ld discarded, queue_size=%zu, current_frame_id=%lu",
                   node->mla_frame_id - node->mla_discarded_frames, 
                   node->mla_discarded_frames,
                   node->mla_frame_id_queue_.size(),
                   node->frame_id);
    }
    
    // Log synchronization warnings periodically
    if (node->mla_frame_id % 100 == 0 && !node->silent) {
        std::lock_guard<std::mutex> lock(node->mla_frame_queue_mutex_);
        uint64_t queue_backlog = node->frame_id > 0 ? 
            (node->mla_frame_id_queue_.size() > 0 ? 
             node->frame_id - node->mla_frame_id_queue_.front() : 0) : 0;
        if (queue_backlog > 10) {
            RCLCPP_WARN(node->get_logger(), 
                       "MLA: Large queue backlog detected (backlog=%lu frames, queue_size=%zu). "
                       "Consider reducing frame rate or optimizing MLA processing.",
                       queue_backlog, node->mla_frame_id_queue_.size());
        }
    }   
    
    // Optional: Dump MLA output for debugging
    if (node->enable_dump) {
        GstMapInfo map;
        if (gst_buffer_map(writable_buffer, &map, GST_MAP_READ)) {
            std::string dump_path = "/tmp/MLA_Output_" + std::to_string(node->mla_frame_id) + ".bin";
            std::ofstream out_file(dump_path, std::ios::binary);
            if (out_file.is_open()) {
                out_file.write(reinterpret_cast<const char*>(map.data), map.size);
                out_file.close();
                RCLCPP_INFO(node->get_logger(), "Dumped MLA data to %s (%zu bytes)", 
                           dump_path.c_str(), map.size);
            } else {
                RCLCPP_ERROR(node->get_logger(), "Failed to open dump file: %s", dump_path.c_str());
            }
            gst_buffer_unmap(writable_buffer, &map);
        } else {
            RCLCPP_ERROR(node->get_logger(), "Failed to map MLA buffer for dumping");
        }
    }
    
    // Publish the ROS message
    node->publisher_mla_raw_->publish(std::move(out_msg));  
    
    if (!node->silent) {
        RCLCPP_DEBUG(node->get_logger(), 
                    "MLA - Published frame_id: %lu, size: %zu bytes, phys: 0x%lx", 
                    node->mla_frame_id, buffer_size, 
                    simaai_memory_get_phys(gst_mem->alloc_segments[0]));
    }
    
    node->mla_frame_id++;
    return GST_FLOW_OK;
}

RCLCPP_COMPONENTS_REGISTER_NODE(ProcessSimaYoloPose)
