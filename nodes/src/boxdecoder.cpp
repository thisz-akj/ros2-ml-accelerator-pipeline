#include <arm_fp16.h>
#include <iostream>
#include <dlfcn.h> 
#include <simaai/boxdecode.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <functional>
#include <simaai/helpers.hpp>


#include <nlohmann/json.hpp>
#include "utils.hpp"
#include "boxdecoder.hpp"




void  BoxDecoder::run()  {
  while (!stop_bool.load()) {  
    ImageMsgType in_msg;
    if(!tensor_msg_queue_.receive_blocking(in_msg)) {
      break;
    }
    if (!silent) RCLCPP_INFO(logger_, "BoxDecoder Start");
    void* in_data_ptr = generic_memory_resource_->simaai_buffer_map(in_msg.payload_buf);   
    void* out_data_ptr = malloc(output_size);
    if(dll_run_(0, in_data_ptr, input_size, out_data_ptr, output_size) < 0){
        RCLCPP_ERROR(logger_, "Couldn't run boxdecode");
        throw std::runtime_error("Couldn't run boxdecode");
    }
    if(enable_dumps){
        std::string filename = "/tmp/boxdecoder-" + std::to_string(recv_counter) + ".bin";
        std::ofstream file(filename, std::ios::binary);
        if (!file) {
            RCLCPP_INFO(logger_, "Failed to open file %s for writing!", filename.c_str());
            throw std::runtime_error("Failed ot open file");
        }
        RCLCPP_INFO(logger_, "Writing Boxdecoder result into file %s", filename.c_str());
        file.write(static_cast<const char*>(out_data_ptr), output_size);
    }
    ImageMsgType frame_msg;
    if (!this->frame_msg_queue_.receive_nonblocking(frame_msg)) {
       RCLCPP_ERROR(logger_, "No frame message available. id=%lu", frame_msg.frame_id);
    }
    if (in_msg.frame_id != frame_msg.frame_id) {
       RCLCPP_ERROR(logger_, "Frame ids are different: %lu , %lu ", in_msg.frame_id, frame_msg.frame_id); 
    }
    simaai_common::msg::SimaaiMessage out_msg;
    out_msg.frame_id = frame_msg.frame_id;
    frame_msg.payload_buf.copy_to_ros(out_msg.payload_buf);
    out_msg.inference_buf = (uint64_t)out_data_ptr;
    publisher_->publish(std::move(out_msg));
    recv_counter++;
    generic_memory_resource_->simaai_buffer_free(in_msg.payload_buf);
    if (!silent) RCLCPP_INFO(logger_, "BoxDecoder End");
  }
  RCLCPP_DEBUG(logger_, "BoxDecoder::run() fn exited");

}


bool BoxDecoder::init(std::string config_file_path, std::shared_ptr<rclcpp::Publisher<simaai_common::msg::SimaaiMessage>> publisher, int in_size, bool _silent) 
{
    silent = _silent;
    publisher_ = publisher;
    input_size = in_size;
    std::cout << "boxdecoder config file path: " << config_file_path << std::endl;
    void* dl_handle = dlopen("/usr/lib/aarch64-linux-gnu/libsimaai_genboxdecode.so", RTLD_LAZY); 
    if (dl_handle == nullptr) {
      RCLCPP_ERROR(logger_, "Error in opening libsimaai_genboxdecode.so lib: " );
      RCLCPP_INFO(logger_, dlerror());
      throw std::runtime_error("Error in opening libsimaai_genboxdecode.so lib: " );
    }
    dll_config_ = (conffunc_t) dlsym(dl_handle, "configure");
    dll_run_ = (runfunc_t) dlsym(dl_handle, "run");
    if ((dll_config_ == nullptr) || (dll_run_ == nullptr)) {
      RCLCPP_ERROR(logger_, "Error in instantiating lib functions: " );
      throw std::runtime_error("Error in instantiating lib functions: ");
    } 
    int size = 0;
    if((size = dll_config_(0, config_file_path.c_str())) <= 0) {
        RCLCPP_ERROR(logger_, "Error configuring A65 kernel");
        return false;
    }
    output_size = size;
    generic_memory_resource_ = static_cast<simaai::memory_resource::GenericMemoryResource*>(std::pmr::get_default_resource());
    thread_ = std::thread(&BoxDecoder::run, this);
    RCLCPP_INFO(logger_, "BoxDecoder created");
    return true;
}

void BoxDecoder::stop()
{
  RCLCPP_DEBUG(logger_, "BoxDecoder::stop() called");
  stop_bool.store(true);
  tensor_msg_queue_.stop();
  frame_msg_queue_.stop();
}

BoxDecoder::~BoxDecoder()
{
    RCLCPP_DEBUG(logger_, "BoxDecoder Desctructor Called");
    if (thread_.joinable()) {
        thread_.join();
        RCLCPP_DEBUG(logger_, "BoxDecoder::run() thread joined");
    }
    RCLCPP_DEBUG(logger_, "BoxDecoder Desctructor Successful");
}