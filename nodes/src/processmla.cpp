#include <string>
#include <fstream>
#include <chrono>
#include <iostream>
#include <rclcpp_components/register_node_macro.hpp>
#include <configManager.h>
#include <job.hh>
#include "utils.hpp"
#include "boxdecoder.hpp"
#include "processmla.hpp"
#include "simaai/helpers.hpp"

bool ProcessMLA::init(std::string config_file_path, std::string model_file_path, BoxDecoder &box_decoder, bool _silent) {
    silent = _silent;
    box_decoder_ = &box_decoder;
    generic_memory_resource_ = static_cast<simaai::memory_resource::GenericMemoryResource*>(std::pmr::get_default_resource());
    if (!generic_memory_resource_) {
        RCLCPP_ERROR(logger_, "Can't obtain generic memory resource!");
        throw std::runtime_error("Failed to obtain generic memory resource!");
    }
    output_message_info_.num_segments = 0;
    output_message_info_.sizes.fill(0);
    if (!parse_output_segments(config_file_path)) {
        throw std::runtime_error("Failed to populate output message info");
    }
    dispatcher = simaaidispatcher::DispatcherFactory::getDispatcher(simaaidispatcher::DispatcherFactory::MLASHM);    
    RCLCPP_INFO(logger_, "Loading MLA model %s", model_file_path.c_str());
    model_handle = dispatcher->load(model_file_path.c_str());
    RCLCPP_INFO(logger_, "Loaded MLA model");
    thread_ = std::thread(&ProcessMLA::run, this); 
    RCLCPP_DEBUG(logger_, "ProcessMLA created");
    return true;
}

ProcessMLA::~ProcessMLA() {

    RCLCPP_DEBUG(logger_, "ProcessMLA Desctructor Called");
    if (thread_.joinable()) {
        thread_.join();
        RCLCPP_DEBUG(logger_, "ProcessMLA::run() thread joined");
    }
    
    if (dispatcher){
        if (model_handle){
            dispatcher->release(model_handle);
            delete dispatcher;
        }
    }
    RCLCPP_DEBUG(logger_, "ProcessMLA Desctructor Succesful");
} 


bool ProcessMLA::parse_output_segments(std::string config_file_path) {
    nlohmann::json config;
    if (!parse_json_from_file(config_file_path, config)) {
        RCLCPP_ERROR(logger_, "Failed to open config file %s", config_file_path.c_str());
        return false;
    }
    if (!config.contains("simaai__params")) {
        std::cerr << "No 'simaai__params' key found in config JSON!" << std::endl;
        return false;
    }
    const auto &params = config["simaai__params"];
    if (!params.contains("outputs")) {
        std::cerr << "No 'outputs' key found in simaai__params!" << std::endl;
        return false;
    }
    const auto &outputs = params["outputs"];
    size_t num_segments = outputs.size();
    if (num_segments == 0) {
        std::cerr << "No output segments provided!" << std::endl;
        return false;
    }
    output_message_info_.num_segments = num_segments;
    for (size_t i = 0; i < num_segments; ++i) {
        const auto &entry = outputs[i];
        std::string name = entry["name"];
        size_t size = entry["size"];
        output_message_info_.sizes[i] = size;
        out_size_ += size;
        std::cout << "Adding output segment " << name << " with size " << size << std::endl;
    }
    return true;
}

void ProcessMLA::stop()
{
    RCLCPP_DEBUG(logger_, "ProcessMLA::stop() Called");
    stop_bool.store(true);
    frame_msg_queue_.stop();
}

void ProcessMLA::run() {
  while (!stop_bool.load())  {  
    ImageMsgType in_msg;
    if(!frame_msg_queue_.receive_blocking(in_msg)) {
        break;
    }
    if (!silent) RCLCPP_INFO(logger_, "MLA Start");
    ImageMsgType out_msg;
    generic_memory_resource_->simaai_buffer_allocate(out_msg.payload_buf, output_message_info_);
    simaai_memory_t *ifm = in_msg.payload_buf.segments[0];
    if (ifm == nullptr) {
        RCLCPP_ERROR(logger_, "ifm: Can't get simaai memory pointer");
        return;
    }
    simaai_memory_t *ofm = out_msg.payload_buf.segments[0];
    if (ofm==nullptr) {
        RCLCPP_ERROR(logger_, "ofm: Can't get simaai memory pointer");
        return;
    } 
    simaaidispatcher::JobMLA job;
    job.path = model_path;
    job.handle = model_handle;
    job.batchSize = 1;
    job.buffers["ifm0"] = ifm;
    job.buffers["ofm0"] = ofm;
    int retval = dispatcher->run(job);
    if (retval != 0) {
        RCLCPP_ERROR(logger_, "Dispatcher returned error: %d", retval);
        return;
    }    
    generic_memory_resource_->simaai_buffer_free(in_msg.payload_buf);
    static int counter = 0;
    if (enable_dumps) {
        void *mla_out_virt = generic_memory_resource_->simaai_buffer_map(out_msg.payload_buf);
        std::string filename = "/tmp/processmla-" + std::to_string(counter++) + ".bin";
        std::ofstream file(filename, std::ios::binary);
        if (!file) {
            RCLCPP_ERROR(logger_, "Failed to open file %s for writing!", filename.c_str());
            dispatcher->release(job.handle);
            return;
        }
        RCLCPP_DEBUG(logger_, "Writing MLA result into file %s", filename.c_str());
        file.write(static_cast<const char*>(mla_out_virt), out_size_);
        generic_memory_resource_->simaai_buffer_unmap(out_msg.payload_buf);
    } 
    out_msg.frame_id = in_msg.frame_id;
    box_decoder_->send_inf_to(out_msg);    
    if (!silent) RCLCPP_INFO(logger_, "MLA End");
  }
  RCLCPP_DEBUG(logger_, "ProcessMLA::run() fn exited");
}
