#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <memory>
#include <queue>
#include <mutex>
#include <condition_variable>
/* This is implementation of the C++ queue with "backpressure" blocking access for both reading and writing to the queue. There is a blocking call "receive_blocking" which will wait on conditional until there is a message in the queue. At the same time there is blocking "send" call that will wait until the queue has space for messages, where the size of the queue is configurable. In order to get the best results in pipelined scheduling and lowest latency, the queue sizes in the "sequential chain" should be configured to 1
*/

template <typename MsgT>
class MsgQueue {
private:
    std::queue<MsgT> queue_; // The underlying queue
    mutable std::mutex mutex_; // Mutex for protecting access to the queue
    std::condition_variable empty_cond_var_; // Condition variable for signaling enqueue changes to the queue
    std::condition_variable full_cond_var_; // Condition variable for signaling dequeue changes to the queue
    std::size_t capacity_ = 1;
    std::atomic<bool> stopped_{false};
public:
    MsgQueue (size_t capacity) : capacity_(capacity) {}
    // Enqueue an item
    void send(MsgT &msg) {
        std::unique_lock<std::mutex> lock(mutex_); // Acquire a lock to protect the queue
        full_cond_var_.wait(lock, [this]{ return (queue_.size() < capacity_); }); // Wait until the queue has enough space
        queue_.push(msg); // Move the item into the queue
        empty_cond_var_.notify_one(); // Notify one waiting thread that a new item is available
    }
    void send_nonblocking(MsgT &msg) {
        std::lock_guard<std::mutex> lock(mutex_); // Acquire a lock to protect the queue
        queue_.push(msg); // Move the item into the queue
    }
    bool receive_blocking(MsgT &msg) {
        std::unique_lock<std::mutex> lock(mutex_); // Acquire a unique lock
        empty_cond_var_.wait(lock, [this] { return stopped_ || !queue_.empty(); });
        if (stopped_ && queue_.empty()) {
            return false;
        }
        msg = queue_.front(); // Move the item out of the queue
        queue_.pop(); // Remove the item
        full_cond_var_.notify_one(); // Notify one waiting thread that a new space is available
        return true; // Return the item
    }
    
    bool receive_nonblocking(MsgT &msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) {
            return false;
        }
        msg = queue_.front();
        queue_.pop();
        return true;
    }
    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }
    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

    void stop() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopped_ = true;
        }
        empty_cond_var_.notify_all();
        full_cond_var_.notify_all();
    }
};



#endif // UTILS_HPP_