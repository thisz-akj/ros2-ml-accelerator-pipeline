#ifndef SIMAAI_NODES__PROCESSMLA_HPP_
#define SIMAAI_NODES__PROCESSMLA_HPP_

#include "rclcpp/rclcpp.hpp"

#include <nlohmann/json.hpp>

#include <dispatcherbase.hh>
#include <dispatcherfactory.hh>

// SiMa memory resource
#include "simaai_common/msg/simaai_message.hpp"
#include "memory_generic.hpp"
#include "utils.hpp"


class ProcessMLA {
public:
    ProcessMLA(rclcpp::Logger logger):logger_(logger), frame_msg_queue_(1) {}
    bool init(std::string config_file_path, std::string model_file_path, BoxDecoder &box_decoder, bool silent);
    virtual ~ProcessMLA();
    void send_frame_to(ImageMsgType &msg) { frame_msg_queue_.send(msg); }
    int get_out_size() {return out_size_;}
    void stop();
private:    
    std::atomic<bool> stop_bool{false};
    bool enable_dumps = false;
    std::string model_path;
    void *model_handle;
    simaaidispatcher::DispatcherBase *dispatcher;
    size_t out_size_ = 0;
    bool silent = true;
    bool parse_output_segments(std::string config_file_path);
    void run();
    BoxDecoder *box_decoder_ = nullptr;
    std::thread thread_;
    const rclcpp::Logger logger_;
    simaai::memory_resource::SimaaiMessageInfo output_message_info_;
    simaai::memory_resource::GenericMemoryResource* generic_memory_resource_;
    MsgQueue<ImageMsgType> frame_msg_queue_;
};

#endif // SIMAAI_NODES__PROCESSMLA_HPP_
