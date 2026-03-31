#ifndef SIMAAI_ROS2_COMMON_MEMORY_GENERIC_HPP
#define SIMAAI_ROS2_COMMON_MEMORY_GENERIC_HPP

#include "memory_interface.hpp"

namespace simaai
{
namespace memory_resource
{
    class GenericMemoryResource : public MemoryResource {
        public:

        GenericMemoryResource() : MemoryResource(SIMAAI_MEM_TARGET_GENERIC, SIMAAI_MEM_FLAG_CACHED)
        {}

        virtual ~GenericMemoryResource() = default;
    };
}
}

#endif