#include <arm_fp16.h>
#include <dlfcn.h> 
#include <simaai/boxrender.h>
#include <dlfcn.h> 
#include <rclcpp_components/register_node_macro.hpp>
#include <functional>
#include <simaai/helpers.hpp>
#include <iostream>
#include <nlohmann/json.hpp>
#include <mutex>
#include "genericrender.hpp"

struct Points {
  unsigned int x1; ///< Origin x
  unsigned int y1; ///< Origin y
  unsigned int w;  ///< Width of the rectangle
  unsigned int h;  ///< height of the rectangle
  unsigned trackId; ///< TrackId from tracker
  unsigned classId; ///< ClassId from Centernet
};


void 
GenericRender::publish_pose_detections(void* output)
{
    std_msgs::msg::Float32MultiArray msg;
    msg.data.resize(17 * 3); // 17 keypoints with 3 floats (x,y,visibility score) for each.


    int num_poses = *((int *)(output));
    PoseOut *pose = (PoseOut *)((char *)output + sizeof(int) + 24*sizeof(Points));
    for (int kp_id = 0; kp_id < 17; ++kp_id){
        msg.data[kp_id*3 + 0] = (float)pose->pose_points[kp_id]._x;
        msg.data[kp_id*3 + 1] = (float)pose->pose_points[kp_id]._y;
        msg.data[kp_id*3 + 2] = pose->pose_points[kp_id]._visible;
    }

    ext_publisher_->publish(std::move(msg));

}

void GenericRender::publish_box_detection(void* output)
{
    std_msgs::msg::Float32MultiArray msg;
    int num_detections = *((int*)output);
    //first element for num detections
    // then 24 detections with 6 floating point each for (x1,y1,w,h,score,classId)
    msg.data.resize(1 + (6*num_detections)); 
    msg.data[0] = (float)num_detections;
    Points* points_array = reinterpret_cast<Points*>(
        reinterpret_cast<char*>(output) + sizeof(int)
    );

    for (int i=0; i<num_detections; i++){
        const Points& p = points_array[i];
        msg.data[(i*5)+1 + 0] = (float)p.x1;
        msg.data[(i*5)+1 + 1] = (float)p.y1;
        msg.data[(i*5)+1 + 2] = (float)p.w;
        msg.data[(i*5)+1 + 3] = (float)p.h;
        msg.data[(i*5)+1 + 4] = *reinterpret_cast<const float*> (&p.trackId);
        msg.data[(i*5)+1 + 5] = (float)p.classId;
    }

    ext_publisher_->publish(std::move(msg));
}


bool GenericRender::overlay_callback(simaai_common::msg::SimaaiMessage::ConstSharedPtr msg)
{
    if (!silent) RCLCPP_INFO(get_logger(), "GenericRender Start: Received keypoints for frame_id=%lu" , msg->frame_id);
    void* overlay_data_ptr = (void *)(msg->inference_buf);
    void *image_data_ptr = generic_memory_resource_->simaai_buffer_map(msg->payload_buf);
    if(image_data_ptr == nullptr) {
        generic_memory_resource_->simaai_buffer_free(msg->payload_buf);
        RCLCPP_ERROR(get_logger(), "GenericRender: could not map image buffer");
        return false;
    }      
    if(dll_run_(0, 
        image_data_ptr, image_input_size, 
        overlay_data_ptr, overlay_data_input_size, 
        nullptr, 0) < 0)  {
        return false;
    }

    if(external_publish_){
        if(render_type_ == "poseyolo"){
            publish_pose_detections(overlay_data_ptr);
        } else if(render_type_ == "bboxs") {
            publish_box_detection(overlay_data_ptr);
        }
    }

    free(overlay_data_ptr);
    
    if(enable_dumps){
        std::string filename = "/tmp/genericrender-" + std::to_string(recv_counter) + ".bin";
        std::ofstream file(filename, std::ios::binary);
        if (!file) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file %s for writing!", filename.c_str());
            return false;
        }
        RCLCPP_DEBUG(this->get_logger(), "Writing GenericDecoder result into file %s", filename.c_str());
        file.write(static_cast<const char*>(image_data_ptr), image_input_size);
    }
    //generic_memory_resource_->simaai_buffer_unmap(msg->payload);
    
    simaai_common::msg::SimaaiMessage out_msg;
    out_msg.payload_buf = msg->payload_buf;
    out_msg.frame_id = msg->frame_id;
    publisher_->publish(std::move(out_msg));
    frames_processed_++;
    recv_counter++;
    if (!silent) RCLCPP_INFO(get_logger(), "Render callback End for %lu", frames_processed_);
    return true;
}

bool GenericRender::configure_render_type(std::string config_path){
    nlohmann::json json;
        if (!parse_json_from_file(config_path, json)) return false;

    // parse render_type
    render_type_ = json["render_type"].get<std::string>();
    RCLCPP_INFO(this->get_logger(), "Parsed render type: %s", render_type_.c_str());
    return true;
}

GenericRender::GenericRender(const rclcpp::NodeOptions & options) : Node("genericrender", options) {}

bool GenericRender::init(int image_size, int infer_size) {
    image_input_size = image_size;
    output_size = image_input_size;
    overlay_data_input_size = infer_size;

    std::string config_file_path = declare_parameter<std::string>("config_file_path", "");
    std::string package_dir = this->declare_parameter("package_dir", "");
    std::string hardware_target = this->declare_parameter("hardware_target", "");
    config_file_path = resolve_package_resource_path(config_file_path, hardware_target, package_dir);

    RCLCPP_INFO(this->get_logger(), "Generic render config path: %s", config_file_path.c_str());
    configure_render_type(config_file_path);
    auto sub_topic = this->declare_parameter("sub_topic", "simaai/boxdecode/output");
    auto pub_topic = this->declare_parameter("pub_topic", "simaai/genericrender/output");

    auto ext_pub_topic = this->declare_parameter("ext_pub_topic", "simaai/genericrender/detections");
    ext_pub_topic = this->get_parameter("ext_pub_topic").as_string();
    external_publish_ = this->declare_parameter("external_publish_", false);
    external_publish_ = this->get_parameter("external_publish_").as_bool();

    RCLCPP_INFO(this->get_logger(), "Generic render pub topic: %s", pub_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Generic render ext_pub_topic: %s", ext_pub_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Generic render external_publish_: %s", external_publish_ ? "true" : "false");

    enable_dumps = this->declare_parameter("dump", false);
    queue_size = static_cast<size_t>(this->declare_parameter<int64_t>("queue_size", 10));
    silent = this->declare_parameter("silent", true);
    generic_memory_resource_ = static_cast<simaai::memory_resource::GenericMemoryResource*>(std::pmr::get_default_resource());
  
    void* dl_handle = dlopen("/usr/lib/aarch64-linux-gnu/libboxrender.so", RTLD_LAZY); 
    if (dl_handle == nullptr) {
      RCLCPP_ERROR(this->get_logger(), "Error in opening libboxrender.so lib: " );
      RCLCPP_INFO(this->get_logger(), dlerror());
      throw std::runtime_error("Error in opening libboxrender.so lib: " );
    }
    dll_config_ = (conffunc_t) dlsym(dl_handle, "configure");
    dll_run_ = (runfunc_t) dlsym(dl_handle, "run");
    if ((dll_config_ == nullptr) || (dll_run_ == nullptr)) {
      RCLCPP_ERROR(this->get_logger(), "Error in instantiating lib functions in libboxrender.so: " );
      throw std::runtime_error("Error in instantiating lib functions in libboxrender.so: ");
      return false;
    }
    
    std::cout << "Configuring boxrender kernel with config_file_path= " << config_file_path << std::endl;
    if(dll_config_(0, config_file_path.c_str()) < 0){
        RCLCPP_ERROR(this->get_logger(), "Error configuring genericrender kernel");
        return false;
    }
    // create publisher and subscriber
    publisher_ = this->create_publisher<simaai_common::msg::SimaaiMessage>(pub_topic, queue_size);
    sub_ = this->create_subscription<simaai_common::msg::SimaaiMessage>( sub_topic, queue_size,
        std::bind(&GenericRender::overlay_callback, this, std::placeholders::_1));
    
    ext_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(ext_pub_topic, queue_size);

    
    RCLCPP_INFO(this->get_logger(), "GenericRender created");
    return true;
}
