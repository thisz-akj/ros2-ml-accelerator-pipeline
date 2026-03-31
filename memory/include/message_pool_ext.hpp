#ifndef SIMAAI_ROS2_MESSAGE_POOL_HPP
#define SIMAAI_ROS2_MESSAGE_POOL_HPP

#include "rclcpp/macros.hpp"
#include "rclcpp/message_memory_strategy.hpp"
#include "rclcpp/visibility_control.hpp"
 
#include <array>
#include <memory>
#include <mutex>

namespace rclcpp
{
namespace strategies
{
namespace message_pool_memory_strategy
{

  template<
  typename MessageT,
  typename Alloc,
  size_t Size,
  typename std::enable_if<
    rosidl_generator_traits::has_fixed_size<MessageT>::value
  >::type * = nullptr
>
class MessagePoolMemoryStrategyExt
  : public message_memory_strategy::MessageMemoryStrategy<MessageT, Alloc>
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(MessagePoolMemoryStrategyExt)

  
  MessagePoolMemoryStrategyExt()
  : next_array_index_(0)
  {
    for (size_t i = 0; i < Size; ++i) {
      pool_[i].msg_ptr_ = std::make_shared<MessageT>();
      pool_[i].used = false;
    }
  }


  std::shared_ptr<MessageT> borrow_message()
  {
    size_t current_index = next_array_index_;
    next_array_index_ = (next_array_index_ + 1) % Size;
    if (pool_[current_index].used) {
      throw std::runtime_error("Tried to access message that was still in use! Abort.");
    }
    pool_[current_index].msg_ptr_->~MessageT();
    new (pool_[current_index].msg_ptr_.get())MessageT;

    pool_[current_index].used = true;
    return pool_[current_index].msg_ptr_;
  }


  void return_message(std::shared_ptr<MessageT> & msg)
  {
    for (size_t i = 0; i < Size; ++i) {
      if (pool_[i].msg_ptr_ == msg) {
        pool_[i].used = false;
        return;
      }
    }
    throw std::runtime_error("Unrecognized message ptr in return_message.");
  }

protected:
  struct PoolMember
  {
    std::shared_ptr<MessageT> msg_ptr_;
    bool used;
  };

  std::array<PoolMember, Size> pool_;
  size_t next_array_index_;
};

}
}
}

#endif