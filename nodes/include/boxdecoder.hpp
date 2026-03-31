#ifndef BOXDECODER_HPP_
#define BOXDECODER_HPP_

#include <memory>
#include <simaai/helpers.hpp>
#include <nlohmann/json.hpp>
#include <arm_fp16.h>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "simaai_common/msg/simaai_message.hpp"
#include "memory_generic.hpp"


class BoxDecoder {
public:
    BoxDecoder(rclcpp::Logger logger):logger_(logger), tensor_msg_queue_(1),frame_msg_queue_(10)  {}
    bool init(std::string config_file_path, std::shared_ptr<rclcpp::Publisher<simaai_common::msg::SimaaiMessage>> publisher, int in_size, bool silent);
    void stop();
    void send_frame_to(ImageMsgType &msg) { frame_msg_queue_.send(msg); }
    void send_inf_to(ImageMsgType &msg) { tensor_msg_queue_.send(msg); }
    int get_output_size() {return output_size;}
    virtual ~BoxDecoder();
private:    
    void run();
    bool enable_dumps = false;
    std::atomic<bool> stop_bool{false};
    // Size of output memory chunk
    int output_size;
    int input_size;
    bool silent = true;
    int recv_counter = 0;
    const rclcpp::Logger logger_;
    std::thread thread_;
    MsgQueue<ImageMsgType> tensor_msg_queue_;
    MsgQueue<ImageMsgType> frame_msg_queue_;
    simaai::memory_resource::GenericMemoryResource* generic_memory_resource_;
    std::shared_ptr<rclcpp::Publisher<simaai_common::msg::SimaaiMessage>> publisher_;
    typedef int (*conffunc_t)(int, const char *);
    typedef int (*runfunc_t)(int, void * , int , void *, int ); 
    conffunc_t dll_config_ = nullptr;
    runfunc_t dll_run_ = nullptr;
};

#endif // BOXDECOCER_HPP_