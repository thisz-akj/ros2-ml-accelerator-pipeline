#ifndef PROCESSSIMA_HPP_
#define PROCESSSIMA_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"
// SiMa specific pools and memory wrappers
#include "simaai_common/msg/image.hpp"
#include "simaai_common/msg/simaai_message.hpp"

#include "memory_generic.hpp"


class ProcessSima : public rclcpp::Node {
    public:
        ProcessSima(const rclcpp::NodeOptions & options);
        bool init();
        
        int get_input_size() { return full_size/4;}
        int get_output_size() { return box_decoder_.get_output_size();}
        virtual ~ProcessSima();
    private:
        bool run(simaai_common::msg::Image::ConstSharedPtr msg);  
        bool configure_job(simaaidispatcher::JobEVXX &job,
                                const simaai_common::msg::SimaaiBuffer &input_buf,
                                const SimaMemMsgType &output_buf);
        bool init_config_manager(std::string config_file_path);
        //enable dumps
        bool enable_dumps;
        bool subscribe_to_allegro_decoder;
        //Queue size
        size_t queue_size;        
        bool silent = true;
        // @brief Combined output size of outgoing buffers. Used for dumping
        int full_size = 0;
        simaaidispatcher::DispatcherBase *dispatcher;
        std::unique_ptr <ConfigManager> config_manager;
        std::vector<std::string> output_memories;
        std::vector<std::string> input_memories;
        // @brief Info object for outgoing buffers from this node
        simaai::memory_resource::SimaaiMessageInfo output_message_info;
        int recv_counter = 0;
        simaai::memory_resource::GenericMemoryResource* generic_memory_resource_;
        std::shared_ptr<rclcpp::Publisher<simaai_common::msg::SimaaiMessage>> publisher_;
        std::shared_ptr<rclcpp::SubscriptionBase> subscriber_; 
        BoxDecoder box_decoder_;
        ProcessMLA mla_proc_;
};

#endif 
