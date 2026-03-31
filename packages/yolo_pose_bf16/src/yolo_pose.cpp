#include <csignal>
#include <iostream>

#include "utils.hpp"
#include "process_sima_yolopose.hpp"
#include "allegro_encoder.hpp"
#include "memory_generic.hpp"
#include "yolopose_render.hpp"
#include "simaai/helpers.hpp"

std::atomic<bool> sigint_received{false};

void signalHandler(int signum) {
  std::cout << "Interrupt code signal received: " << signum << ", Shutting down..." << std::endl;
  sigint_received.store(true);
}


int main(int argc, char ** argv)
{
  // register signal handler for SIGTERM and SIGINT
  signal(SIGINT, signalHandler);
  signal(SIGTERM, signalHandler);
  signal(SIGUSR1, signalHandler);
  signal(SIGUSR2, signalHandler);

  rclcpp::init(argc, argv);
  auto simaai_memory_resource =
    std::make_shared<simaai::memory_resource::GenericMemoryResource>();
  std::pmr::set_default_resource(simaai_memory_resource.get());

  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <path_to_params.yaml>\n";
    return 1;
  }

  std::string package_dir = get_ros_project_root();
  std::string hardware_target = get_hardware_target();

  //create logger instance
  rclcpp::Logger my_logger = rclcpp::get_logger("yolo_pose_main");


  std::cout << "Package Dir: " << package_dir << std::endl;
  std::cout << "Hardware target:" << hardware_target << "[end]" <<std::endl;
  if (hardware_target == "unknown") {
    std::cerr << "[ERROR] Hardware target could not be determined from /etc/build. Exiting.\n";
    return 1;
  }


  rclcpp::NodeOptions base_options;
  base_options.use_intra_process_comms(true);
  base_options.arguments({
    "--ros-args",
    "--params-file", argv[1],
    "--ros-args", "-p", "package_dir:=" + package_dir,
    "--ros-args", "-p", "hardware_target:=" + hardware_target
  });

  // Read executor configuration
  auto exec_config_node = std::make_shared<rclcpp::Node>("executor_config", base_options);
  bool use_multithread = exec_config_node->declare_parameter<bool>("multithread", false);
  int64_t thread_count = exec_config_node->declare_parameter<int64_t>("thread_count", 4);
  exec_config_node.reset();

  // Create executor based on configuration
  std::shared_ptr<rclcpp::Executor> exec;
  if (use_multithread) {
    rclcpp::ExecutorOptions opts{};
    exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(opts, thread_count);
    RCLCPP_INFO(my_logger, "Using MultiThreadedExecutor with %ld threads", thread_count);
  } else {
    exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    RCLCPP_INFO(my_logger, "Using SingleThreadedExecutor");
  }

  // Create nodes with proper naming to match YAML configuration
  RCLCPP_INFO(my_logger, "Creating ProcessSimaYoloPose node...");
  auto process_sima_yolopose = std::make_shared<ProcessSimaYoloPose>(base_options);
  
  RCLCPP_INFO(my_logger, "Creating YoloPoseRender node...");
  auto render = std::make_shared<YoloPoseRender>(base_options);
  
  RCLCPP_INFO(my_logger, "Creating AllegroEncoder node...");
  auto allegro_encoder = std::make_shared<AllegroEncoder>(base_options);
  
  // Initialize render with proper size parameters
  // Note: You may need to adjust these sizes based on your actual configuration
  // The decoder output size and MLA output size should match your pipeline
  // int image_size = 1280 * 720 * 3 / 2; // NV12 format: width * height * 1.5
  // int infer_size = 655200; // Adjust based on your MLA output
  
  RCLCPP_INFO(my_logger, "Initializing YoloPoseRender...");
  // if (!render->init(image_size, infer_size)) {
  //   RCLCPP_ERROR(my_logger, "Failed to initialize YoloPoseRender");
  //   rclcpp::shutdown();
  //   return 1;
  // }
    if (!render->init()) {
      RCLCPP_ERROR(my_logger, "Failed to initialize YoloPoseRender");
      rclcpp::shutdown();
      return 1;
    }
  
  // Set the connected render node in decoder for frame synchronization
  process_sima_yolopose->set_connected_render(render.get());
  
  // Add nodes to executor
  RCLCPP_INFO(my_logger, "Adding nodes to executor...");
  exec->add_node(process_sima_yolopose);
  exec->add_node(render);
  exec->add_node(allegro_encoder);

  RCLCPP_INFO(my_logger, "Starting executor spin...");
  
  // Main execution loop
  while (rclcpp::ok() && !sigint_received.load()) {
    exec->spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  
  RCLCPP_INFO(my_logger, "Shutdown sequence initiated...");
  
  exec->cancel();
  rclcpp::shutdown();
  RCLCPP_DEBUG(my_logger, "rclcpp::shutdown called");

  // Remove nodes from executor
  exec->remove_node(process_sima_yolopose);
  RCLCPP_DEBUG(my_logger, "process_sima_yolopose removed");
  
  exec->remove_node(render);
  RCLCPP_DEBUG(my_logger, "render removed");
  
  exec->remove_node(allegro_encoder);
  RCLCPP_DEBUG(my_logger, "allegro_encoder removed");

  // Reset shared pointers to clean up
  process_sima_yolopose.reset();
  RCLCPP_DEBUG(my_logger, "process_sima_yolopose reset");

  render.reset();
  RCLCPP_DEBUG(my_logger, "render reset");

  allegro_encoder.reset();
  RCLCPP_DEBUG(my_logger, "allegro_encoder reset");

  RCLCPP_INFO(my_logger, "Shutdown sequence successful");
  
  return 0;
}
