#ifndef PROCESSCVU_HPP_
#define PROCESSCVU_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include <configManager.h>
#include <job.hh>
#include <dispatcherbase.hh>

// SiMa specific pools and memory wrappers
#include "memory_generic.hpp"

#include "simaai_common/msg/image.hpp"
#include "simaai_common/msg/simaai_message.hpp"

#define CM2PLUGIN_HW_SIZE 3
typedef std::chrono::time_point<std::chrono::steady_clock> TimePoint;

class ProcessCvu : public rclcpp::Node
{
    public:
        ProcessCvu(const rclcpp::NodeOptions & options);
        bool run_processcvu(simaai_common::msg::Image::ConstSharedPtr msg);
        virtual ~ProcessCvu();
    
    private:
        // JSON configuration file
        std::string config_file_path;

        //enable dumps
        bool enable_dumps;

        bool subscribe_to_allegro_decoder;

        // Graph config manager
        std::unique_ptr <ConfigManager> config_manager;

        // CPU of current plugin
        int target_cpu;

        // CPU of next plugin
        int next_cpu;

        // Plugin instance node name
        std::string node_name;

        // Map of graph mempories where key is name of memory, and value is a pair where first is
        // size and second is type of buffer 
        std::map <std::string, std::pair<unsigned int, enum bufferType>> cm_memories;

        // Size of output memory chunk
        int output_size;

        //Queue size
        size_t queue_size;

        // davinci / modalix
        std::string hardware_target;

        std::string package_dir;

        
        bool silent{true};
        // Dispatcher handle
        simaaidispatcher::DispatcherBase * dispatcher;

        enum {
            SIMA_CPU_A65,
            SIMA_CPU_EVXX,
            SIMA_CPU_MLA,
            SIMA_CPU_TVM,
            SIMA_CPU_NUM
        };

        std::string cm2pluginHW[CM2PLUGIN_HW_SIZE] = {
            [SIMA_CPU_A65] = "APU",
            [SIMA_CPU_EVXX] = "CVU",
            [SIMA_CPU_MLA] = "MLA",
        };

        /**
         * @brief structure wich holds data about graph input/output memories
         */
        struct GraphMemory {
            // name of memory
            std::string memory_name;
            // name of dispatcher input/output
            std::string dispatcher_name;
            // size of memory
            size_t size;
        };

        // @brief Combined output size of outgoing buffers. Used for dumping
        int full_size;

        // @brief Info object for outgoing buffers from this node
        simaai::memory_resource::SimaaiMessageInfo output_message_info;

        // @brief map of buffers, that contains memories used by current graph.
        // @param string name of buffer
        // @param vector<GraphMemory> buffer memories
        std::map<std::string, std::vector<GraphMemory>> graph_buffers;
        std::vector<std::string> output_memories;
        std::vector<std::string> input_memories;
        int recv_counter;

        bool init_config_manager();

        template<typename MsgT>
        bool configure_job(simaaidispatcher::JobEVXX &job,
                                MsgT &input_msg,
                                simaai_common::msg::SimaaiMessage &output_msg);
        bool processcvu_parse_buffers_memories();
        int32_t string2pluginCPU(std::string & in);
        int populate_output_buffer_info(simaai::memory_resource::SimaaiMessageInfo &info);
        
        simaai::memory_resource::GenericMemoryResource* generic_memory_resource_;
        std::shared_ptr<rclcpp::Publisher<simaai_common::msg::SimaaiMessage>> publisher_;
        std::shared_ptr<rclcpp::SubscriptionBase> subscriber_;

        std::vector<std::uint8_t> file_buffer_;
        std::string input_file_path_;
        bool file_loaded_ = false; 
        rclcpp::TimerBase::SharedPtr timer_;
        uint64_t last_image_frame_id_{0};

};

#endif // PROCESSCVU_HPP_
