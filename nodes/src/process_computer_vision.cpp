#include <rclcpp_components/register_node_macro.hpp>
#include <dispatcherfactory.hh>
#include <functional>
#include <fstream>
#include <simaai/helpers.hpp>
#include <typeinfo>

#include "processcvu.hpp"

using namespace std::chrono_literals;

int32_t ProcessCvu::string2pluginCPU(std::string &in) {
  for (size_t i = 0; i < sizeof(cm2pluginHW) / sizeof(cm2pluginHW[0]); i++) {
    if (cm2pluginHW[i] == in)
      return i;
  }
  return SIMA_CPU_EVXX;
}

bool ProcessCvu::processcvu_parse_buffers_memories() {
  std::cout << "processcvu_parse_buffers_memories\n";
  nlohmann::json json;
  if (!parse_json_from_file(this->config_file_path, json)) return false;
  if (!json.contains("input_buffers") || !json.contains("output_memory_order")) return false;

  std::cout << "input_buffers and output_memory_order are present\n";
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

  return true;
}

bool ProcessCvu::init_config_manager() {
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
  target_cpu = string2pluginCPU(conf.currentCpu);
  next_cpu = string2pluginCPU(conf.nextCpu);
  cm_memories = config_manager->getBuffers();

  for (const auto &entry : cm_memories) {
    std::cout << "Key: " << entry.first << "  Value: {" << entry.second.first << "," << entry.second.second << "}" << std::endl;
  }

  return true;
}

int ProcessCvu::populate_output_buffer_info(simaai::memory_resource::SimaaiMessageInfo &info) {
  int full_size = 0;
  for (const auto &buf_name : output_memories) {
    auto it = cm_memories.find(buf_name);
    if (it == cm_memories.end() || it->second.second != bufferType::BUFFER_TYPE_OUTPUT) {
      RCLCPP_ERROR(this->get_logger(), "Invalid output memory: '%s'", buf_name.c_str());
      return 0;
    }
    RCLCPP_INFO(this->get_logger(), "output #%d '%s' size %d", info.num_segments, buf_name.c_str(), it->second.first);
    info.sizes[info.num_segments++] = it->second.first;
    full_size += it->second.first;
  }
  return full_size;
}

template<typename MsgT>
bool ProcessCvu::configure_job(simaaidispatcher::JobEVXX &job,
                                MsgT &input_msg,
                                simaai_common::msg::SimaaiMessage &output_msg) {

  job.graphID = config_manager->getPipelineConfig().graphId;
  job.cm = config_manager.get();
  job.timeout = 60s;

  for (int i = 0; i < output_message_info.num_segments; i++) {
    simaai_memory_t *out_memory = generic_memory_resource_->simaai_buffer_attach(output_msg.payload, i);
    if (!out_memory) {
      RCLCPP_ERROR(this->get_logger(), "Failed to attach output memory: %s", output_memories[i].c_str());
      generic_memory_resource_->simaai_buffer_free(input_msg->payload);
      generic_memory_resource_->simaai_buffer_free(output_msg.payload);
      return false;
    }
    job.buffers[output_memories[i]] = out_memory;
    simaai_memory_flush_cache(out_memory);
  }

  for (size_t i = 0; i < input_memories.size(); ++i) {
    simaai_memory_t *in_memory = generic_memory_resource_->simaai_buffer_attach(input_msg->payload, i);
    if (!in_memory) {
      RCLCPP_ERROR(this->get_logger(), "Failed to attach input memory: %s", input_memories[i].c_str());
      generic_memory_resource_->simaai_buffer_free(input_msg->payload);
      generic_memory_resource_->simaai_buffer_free(output_msg.payload);
      return false;
    }
    job.buffers[input_memories[i]] = in_memory;
    simaai_memory_invalidate_cache(in_memory);
  }

  return true;
}

bool ProcessCvu::run_processcvu(simaai_common::msg::Image::ConstSharedPtr msg) {
  simaai_common::msg::SimaaiMessage out_msg;
  auto t0 = std::chrono::steady_clock::now();

  RCLCPP_DEBUG(this->get_logger(), "Output buffers: %u", output_message_info.num_segments);
  generic_memory_resource_->simaai_buffer_allocate(out_msg.payload, output_message_info);

  simaaidispatcher::JobEVXX job;
  if (!configure_job(job, msg, out_msg)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to configure EVXX job");
    return false;
  }

  int res = dispatcher->run(job);
  //generic_memory_resource_->simaai_buffer_free(msg->payload);

  if (res != 0) {
    RCLCPP_ERROR(this->get_logger(), "CVU dispatch failed with error code: %d", res);
    return false;
  }

  void *out_virt = generic_memory_resource_->simaai_buffer_map(out_msg.payload);
  out_msg.frame_id = msg->frame_id;

  if (enable_dumps) {
    std::string filename = "tmp/processcvu-" + std::to_string(job.graphID) + "-" + std::to_string(recv_counter++) + ".bin";
    std::ofstream file(filename, std::ios::binary);
    if (!file) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", filename.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Dumping to file: %s", filename.c_str());
      file.write(static_cast<const char*>(out_virt), full_size);
    }
  }

  generic_memory_resource_->simaai_buffer_unmap(out_msg.payload);
  publisher_->publish(std::move(out_msg));
  RCLCPP_INFO(this->get_logger(), "ProcessCvu finished");
  auto t1 = std::chrono::steady_clock::now();
  auto total = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0);
  if (!silent)
    RCLCPP_INFO(this->get_logger(), "ProcessCvu took: %ldms", total.count());

  return true;
}

ProcessCvu::ProcessCvu(const rclcpp::NodeOptions &options)
    : Node("processcvu", options), recv_counter(0) {
  RCLCPP_INFO(this->get_logger(), "ProcessCvu created");

  config_file_path = this->declare_parameter("config_file_path", "");
  package_dir = this->declare_parameter("package_dir", "");
  hardware_target = this->declare_parameter("hardware_target", "");


  config_file_path = resolve_package_resource_path(config_file_path, hardware_target, package_dir);
  auto sub_topic = this->declare_parameter("sub_topic", "simaai/openpose/mla/output");
  auto pub_topic = this->declare_parameter("pub_topic", "simaai/openpose/ev/postproc/output");
  enable_dumps = this->declare_parameter("dump", false);
  subscribe_to_allegro_decoder = this->declare_parameter("subscribe_to_allegro_decoder", false);
  queue_size = static_cast<size_t>(this->declare_parameter<int64_t>("queue_size", 10));
  silent = this->declare_parameter("silent", true);

  node_name = "simaai_processcvu";
  dispatcher = simaaidispatcher::DispatcherFactory::getDispatcher(simaaidispatcher::DispatcherFactory::EVXX);
  if (!dispatcher) throw std::runtime_error("Failed to initialize CVU Dispatcher!");

  if (!init_config_manager() || !processcvu_parse_buffers_memories())
    throw std::runtime_error("Failed to initialize configuration or parse memories");

  generic_memory_resource_ = static_cast<simaai::memory_resource::GenericMemoryResource*>(std::pmr::get_default_resource());

  output_message_info.sizes.fill(0);
  output_message_info.num_segments = 0;
  full_size = populate_output_buffer_info(output_message_info);
  if (full_size == 0)
    throw std::runtime_error("Failed to populate output message info");

  publisher_ = this->create_publisher<simaai_common::msg::SimaaiMessage>(pub_topic, queue_size);

/*  if (subscribe_to_allegro_decoder) {*/
      subscriber_ = this->create_subscription<simaai_common::msg::Image>(
      sub_topic, queue_size, std::bind(&ProcessCvu::run_processcvu, this, std::placeholders::_1));
/*  }
  else {
      subscriber_ = this->create_subscription<simaai_common::msg::SimaaiMessage>(
      sub_topic, queue_size,
      [this](const simaai_common::msg::SimaaiMessage::ConstSharedPtr msg) {
        this->run_processcvu<simaai_common::msg::SimaaiMessage>(msg);
      });
  }*/
}

RCLCPP_COMPONENTS_REGISTER_NODE(ProcessCvu)