#ifndef SIMAAI_ROS2_COMMON_MEMORY_INTERFACE_HPP
#define SIMAAI_ROS2_COMMON_MEMORY_INTERFACE_HPP

#include <chrono>
#include <cstdint>
#include <list>
#include <memory>
#include <memory_resource>
#include <stdexcept>
#include <thread>
#include <simaai/simaai_memory.h>
#include <map>
#include "simaai_common/msg/simaai_buffer.hpp"

struct simaai_memory_t;
struct GstSimaaiSegmentMemory;
struct SimaMemMsgType {
  void *buffer = nullptr;
  simaai_memory_t **segments = nullptr;
  GstSimaaiSegmentMemory *gst_mem = nullptr;
  uint8_t num_segments = 0;
  
  void copy_to_ros(simaai_common::msg::SimaaiBuffer &ros_buf) {
    ros_buf.buffer  = (uint64_t)buffer;
    ros_buf.segments = (uint64_t) segments;
    ros_buf.num_segments  = num_segments;
    ros_buf.gst_memory = (uint64_t)gst_mem;
  }
  void copy_from_ros(const simaai_common::msg::SimaaiBuffer &ros_buf) {
    buffer = (void *)ros_buf.buffer;
    segments = (simaai_memory_t **)ros_buf.segments;
    num_segments =  ros_buf.num_segments;
    gst_mem = (GstSimaaiSegmentMemory *)ros_buf.gst_memory;
  }
};
struct ImageMsgType {
    uint64_t frame_id = 0;
    SimaMemMsgType payload_buf;
}; 


namespace simaai
{

namespace memory_resource
{
    constexpr std::uint8_t SIMA_SEGMENT_MAX_NUM = 16;
    struct SimaaiMessageInfo {
      std::array<std::uint32_t, SIMA_SEGMENT_MAX_NUM> sizes;
      std::uint8_t num_segments;
    };
    class MemoryResource : public std::pmr::memory_resource {
        public:
            explicit MemoryResource(std::int32_t memory_target, std::int32_t memory_flags)
                : mem_target_(memory_target), mem_flags_(memory_flags)
            {
                // Intentionally empty
            };
            
            // Does not allow copy/move
            MemoryResource(MemoryResource &other) = delete;
            MemoryResource(MemoryResource &&other) = delete;

            bool simaai_buffer_allocate(SimaMemMsgType & msg, SimaaiMessageInfo & info) {
              simaai_memory_t **segments = simaai_mem_only_allocate(info);
              if (segments == nullptr) {      
                return false;   
              } 
              msg.num_segments = info.num_segments;
              msg.segments = segments;
              void *virt = simaai_memory_map(segments[0]);
              if (!virt) {
                std::cerr << "Failed to map segmented memory!" << std::endl;
                simaai_memory_free_segments(segments, info.num_segments);
              }
              msg.buffer =  virt;             
              return true;
            }

            simaai_memory_t ** simaai_mem_only_allocate(SimaaiMessageInfo & info) {
              std::lock_guard<std::mutex> lock(mutex_);
              if (info.num_segments > SIMA_SEGMENT_MAX_NUM) {
                std::cerr << "Can't allocate more than " << SIMA_SEGMENT_MAX_NUM << " segments! Requested: " << info.num_segments << std::endl;
                return nullptr;
              } else if (info.num_segments == 0) {
                std::cerr << "You must allocate at least one segment! Requested: 0" << std::endl;
                return nullptr;
              }
              simaai_memory_t **segments = simaai_memory_alloc_segments_flags(info.sizes.data(), info.num_segments, mem_target_, mem_flags_);
              //simaai_memory_t *memory = simaai_memory_alloc_flags(info.size, mem_target_, mem_flags_);
              if (segments == nullptr) {
                std::cerr << "Failed to allocate segments in mem allocate only!" << std::endl;
              }
              return segments; 
            }
            bool simaai_buffer_free(const SimaMemMsgType & msg) {
              std::lock_guard<std::mutex> lock(mutex_);
              simaai_memory_unmap(msg.segments[0]);
              simaai_memory_free_segments(msg.segments, msg.num_segments);
              return true;
            }
            bool simaai_buffer_free(const simaai_common::msg::SimaaiBuffer & msg) {
              std::lock_guard<std::mutex> lock(mutex_);
              simaai_memory_t **segs = (simaai_memory_t **)msg.segments;
              simaai_memory_unmap(segs[0]);
              simaai_memory_free_segments(segs, msg.num_segments);
              return true;
            }
            void * simaai_buffer_map(const SimaMemMsgType & msg) {
              std::lock_guard<std::mutex> lock(mutex_);
              simaai_memory_invalidate_cache(msg.segments[0]);
              return simaai_memory_get_virt(msg.segments[0]);
            }
            void * simaai_buffer_map(const simaai_common::msg::SimaaiBuffer & msg) {
              std::lock_guard<std::mutex> lock(mutex_);
              simaai_memory_t **segs = (simaai_memory_t **)msg.segments;
              simaai_memory_invalidate_cache(segs[0]);
              return simaai_memory_get_virt(segs[0]);
            }
            void simaai_buffer_unmap(SimaMemMsgType & msg) {
              std::lock_guard<std::mutex> lock(mutex_);
              simaai_memory_flush_cache(msg.segments[0]);
              return;
            }
        void* do_allocate(std::size_t bytes, [[maybe_unused]] std::size_t alignment) override
        {
          std::cout << "Do nothing: " << bytes << std::endl;
          return nullptr;
        }
            
        void do_deallocate(void* , [[maybe_unused]] std::size_t bytes, [[maybe_unused]] std::size_t alignment) override
        {
          return;
        }
            
        bool do_is_equal([[maybe_unused]] const std::pmr::memory_resource& other) const noexcept override
        {
          // We can't have two equal memory resources
          return false;
        }
        
        private:
            std::int32_t mem_target_;
            std::int32_t mem_flags_;
            std::mutex mutex_;

//            std::map<std::uint64_t, SimaaiSegmentsInfo> allocation_map_;
    };
}
}

#endif