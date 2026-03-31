#include <rclcpp_components/register_node_macro.hpp>
#include <configManager.h>
#include <job.hh>
#include <dispatcherbase.hh>
#include <dispatcherfactory.hh>
#include <functional>
#include <fstream>
#include <simaai/helpers.hpp>
#include <typeinfo>
#include "utils.hpp"
#include "boxdecoder.hpp"
#include "processmla.hpp"
#include "process_sima.hpp"

using namespace std::chrono_literals;

bool ProcessSima::configure_job(simaaidispatcher::JobEVXX &job,
                                const simaai_common::msg::SimaaiBuffer &input_buf,
                                const SimaMemMsgType &output_buf) {

  job.graphID = config_manager->getPipelineConfig().graphId;
  job.cm = config_manager.get();
  job.timeout = 10s;
  simaai_memory_t **segs = output_buf.segments;
  for (int i = 0; i < output_message_info.num_segments; i++) {
    simaai_memory_t *out_memory = segs[i];
    if (!out_memory) {
      RCLCPP_ERROR(this->get_logger(), "Failed to attach output memory: %s", output_memories[i].c_str());
      return false;
    }
    job.buffers[output_memories[i]] = out_memory;
    simaai_memory_flush_cache(out_memory);
  }
  segs = (simaai_memory_t **)input_buf.segments;
  for (size_t i = 0; i < input_memories.size(); ++i) {
    simaai_memory_t *in_memory = segs[i];
    if (!in_memory) {
      RCLCPP_ERROR(this->get_logger(), "Failed to attach input memory: %s", input_memories[i].c_str());
      return false;
    }
    job.buffers[input_memories[i]] = in_memory;
    simaai_memory_invalidate_cache(in_memory);
  }
  return true;
}

bool ProcessSima::run(simaai_common::msg::Image::ConstSharedPtr in_msg) {
  if (!silent) RCLCPP_INFO(this->get_logger(), "ProcessSima Start");
  ImageMsgType out_msg;
  // forward in frame message to the boxdecoder output
  ImageMsgType out_in_msg;
  out_in_msg.frame_id = in_msg->frame_id;
  out_in_msg.payload_buf.copy_from_ros(in_msg->payload);
  box_decoder_.send_frame_to(out_in_msg);
  // now take care of CVU processing
  generic_memory_resource_->simaai_buffer_allocate(out_msg.payload_buf, output_message_info);
  simaaidispatcher::JobEVXX job;
  if (!configure_job(job, in_msg->payload, out_msg.payload_buf)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to configure EVXX job");
    return false;
  }
  int res = dispatcher->run(job);
  if (res != 0) {
    RCLCPP_ERROR(this->get_logger(), "CVU dispatch failed with error code: %d", res);
    return false;
  }
  if (enable_dumps) {
    void *out_virt = generic_memory_resource_->simaai_buffer_map(out_msg.payload_buf);
    std::string filename = "tmp/processsima-" + std::to_string(job.graphID) + "-" + std::to_string(recv_counter++) + ".bin";
    std::ofstream file(filename, std::ios::binary);
    if (!file) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", filename.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Dumping to file: %s", filename.c_str());
      file.write(static_cast<const char*>(out_virt), full_size);
    }
  }
  // send output of CVU to MLA
  out_msg.frame_id = in_msg->frame_id;
  mla_proc_.send_frame_to(out_msg);
  if (!silent) RCLCPP_INFO(this->get_logger(), "ProcessSima End");
  return true;
} 


bool ProcessSima::init_config_manager(std::string config_file_path) {
  RCLCPP_INFO(this->get_logger(), "init_config_manager with file %s", config_file_path.c_str());
  try {
    config_manager = std::make_unique<ConfigManager>(config_file_path);
  } catch (const std::exception &ex) {
    RCLCPP_ERROR(this->get_logger(), "Error allocating CM: %s", ex.what());
    return false;
  }
  if (!config_manager->getConfigStruct()) {
    RCLCPP_ERROR(this->get_logger(), "Error getting config structure. Check syslogs.");
    return false;
  }
  auto conf = config_manager->getPipelineConfig();
  auto cm_memories = config_manager->getBuffers();
  for (const auto &entry : cm_memories) {
    std::cout << "Key: " << entry.first << "  Value: {" << entry.second.first << "," << entry.second.second << "}" << std::endl;
  }
  
  nlohmann::json json;
  if (!parse_json_from_file(config_file_path, json)) return false;
  if (!json.contains("input_buffers") || !json.contains("output_memory_order")) return false;
  auto &input_buffers = json["input_buffers"];
      
  for (const auto &mem : json["output_memory_order"]) {
    std::string name = mem.get<std::string>();
    std::cout << "Added output memory: " << name << std::endl;
    output_memories.push_back(name);
  }

  for (const auto &buffer : input_buffers) {
    for (const auto &memory : buffer["memories"]) {
      std::string name = memory["graph_input_name"];
      std::cout << "Added input memory: " << name << std::endl;
      input_memories.push_back(name);
    }
  }
  output_message_info.num_segments = 0;
  for (const auto &buf_name : output_memories) {
    auto it = cm_memories.find(buf_name);
    if (it == cm_memories.end() || it->second.second != bufferType::BUFFER_TYPE_OUTPUT) {
      RCLCPP_ERROR(this->get_logger(), "Invalid output memory: '%s'", buf_name.c_str());
      return 0;
    }
    RCLCPP_INFO(this->get_logger(), "output #%d '%s' size %d", output_message_info.num_segments, buf_name.c_str(), it->second.first);
    output_message_info.sizes[output_message_info.num_segments++] = it->second.first;
    full_size += it->second.first;
  }
  return true;
}

bool ProcessSima::init() {
  std::string hardware_target = this->declare_parameter("hardware_target", "");
  std::string package_dir = this->declare_parameter("package_dir", "");

  std::string cvu_config_file_path = this->declare_parameter("cvu_config_file_path", "");
  cvu_config_file_path = resolve_package_resource_path(cvu_config_file_path, hardware_target, package_dir);

  std::string mla_config_file_path = this->declare_parameter("mla_config_file_path", "");
  mla_config_file_path = resolve_package_resource_path(mla_config_file_path, hardware_target, package_dir);
  std::string mla_model_file_path = this->declare_parameter("model_path", "");
  mla_model_file_path = resolve_package_resource_path(mla_model_file_path, hardware_target, package_dir);


  std::string boxdecode_config_file_path = this->declare_parameter("boxdecode_config_file_path", "");
  boxdecode_config_file_path = resolve_package_resource_path(boxdecode_config_file_path, hardware_target, package_dir);

  auto sub_topic = this->declare_parameter("sub_topic", "simaai/infer/input");
  auto pub_topic = this->declare_parameter("pub_topic", "simaai/infer/output");
  enable_dumps = this->declare_parameter("dump", false);
  subscribe_to_allegro_decoder = this->declare_parameter("subscribe_to_allegro_decoder", false);
  queue_size = static_cast<size_t>(this->declare_parameter<int64_t>("queue_size", 10));
  silent = this->declare_parameter("silent", true);

  subscriber_ = this->create_subscription<simaai_common::msg::Image>(  sub_topic, queue_size,
      [this](simaai_common::msg::Image::ConstSharedPtr msg) {   this->run(msg); });

  std::shared_ptr<rclcpp::Publisher<simaai_common::msg::SimaaiMessage>> inf_publisher = this->create_publisher<simaai_common::msg::SimaaiMessage>(pub_topic, 1);

  dispatcher = simaaidispatcher::DispatcherFactory::getDispatcher(simaaidispatcher::DispatcherFactory::EVXX);
  if (!dispatcher) throw std::runtime_error("Failed to initialize CVU Dispatcher!");

  if (!init_config_manager(cvu_config_file_path))
    throw std::runtime_error("Failed to initialize configuration or parse memories");

  generic_memory_resource_ = static_cast<simaai::memory_resource::GenericMemoryResource*>(std::pmr::get_default_resource());
  
  mla_proc_.init(mla_config_file_path, mla_model_file_path, box_decoder_, silent);
  box_decoder_.init(boxdecode_config_file_path, inf_publisher, mla_proc_.get_out_size(), silent);

  RCLCPP_INFO(this->get_logger(), "ProcessSima ROS Node created");
  return true;
}

ProcessSima::~ProcessSima()
{
  RCLCPP_DEBUG(this->get_logger(), "Destructor Called");
  mla_proc_.stop();
  box_decoder_.stop();
  RCLCPP_DEBUG(this->get_logger(), "Destructor Successful");
}

ProcessSima::ProcessSima(const rclcpp::NodeOptions &options) : Node("processsima", options), 
   box_decoder_(get_logger()), mla_proc_(get_logger()) {}

RCLCPP_COMPONENTS_REGISTER_NODE(ProcessSima)