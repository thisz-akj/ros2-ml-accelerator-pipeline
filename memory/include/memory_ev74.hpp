#ifndef SIMAAI_ROS2_COMMON_MEMORY_EV_HPP
#define SIMAAI_ROS2_COMMON_MEMORY_EV_HPP

#include "memory_interface.hpp"

namespace simaai
{
namespace memory_resource
{
    class EVMemoryResource : public MemoryResource {
        public:

        EVMemoryResource() : MemoryResource(SIMAAI_MEM_TARGET_EV74, SIMAAI_MEM_FLAG_CACHED)
        {}

        virtual ~EVMemoryResource() = default;
    };
}
}

#endif