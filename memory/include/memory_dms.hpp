#ifndef SIMAAI_ROS2_COMMON_MEMORY_DMS_HPP
#define SIMAAI_ROS2_COMMON_MEMORY_DMS_HPP

#include "memory_interface.hpp"

namespace simaai
{
namespace memory_resource
{
    class DMS0MemoryResource : public MemoryResource {
        public:

        DMS0MemoryResource() : MemoryResource(SIMAAI_MEM_TARGET_DMS0, SIMAAI_MEM_FLAG_DEFAULT)
        {}

        virtual ~DMS0MemoryResource() = default;
    };

    class DMS1MemoryResource : public MemoryResource {
        public:

        DMS1MemoryResource() : MemoryResource(SIMAAI_MEM_TARGET_DMS1, SIMAAI_MEM_FLAG_DEFAULT)
        {}

        virtual ~DMS1MemoryResource() = default;
    };

    class DMS2MemoryResource : public MemoryResource {
        public:

        DMS2MemoryResource() : MemoryResource(SIMAAI_MEM_TARGET_DMS2, SIMAAI_MEM_FLAG_DEFAULT)
        {}

        virtual ~DMS2MemoryResource() = default;
    };

    class DMS3MemoryResource : public MemoryResource {
        public:

        DMS3MemoryResource() : MemoryResource(SIMAAI_MEM_TARGET_DMS3, SIMAAI_MEM_FLAG_DEFAULT)
        {}

        virtual ~DMS3MemoryResource() = default;
    };

}
}

#endif