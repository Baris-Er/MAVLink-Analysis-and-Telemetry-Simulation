#ifndef LOCKFREEQUEUE_H
#define LOCKFREEQUEUE_H

#include <atomic>
#include <vector>
#include <cstddef>  

template<typename T>
class LockFreeQueue {
public:
    // Constructor to initialize the queue with a specified size
    explicit LockFreeQueue(std::size_t size) : buffer(size), head(0), tail(0) {}

    // Method to enqueue an item
    bool enqueue(const T& item) {
        std::size_t current_tail = tail.load(std::memory_order_relaxed);
        std::size_t next_tail = (current_tail + 1) % buffer.size();
        if (next_tail == head.load(std::memory_order_acquire)) {
            return false; 
        }
        buffer[current_tail] = item;
        tail.store(next_tail, std::memory_order_release);
        return true;
    }

    // Method to dequeue an item
    bool dequeue(T& item) {
        std::size_t current_head = head.load(std::memory_order_relaxed);
        if (current_head == tail.load(std::memory_order_acquire)) {
            return false; 
        }
        item = buffer[current_head];
        head.store((current_head + 1) % buffer.size(), std::memory_order_release);
        return true;
    }

private:
    std::vector<T> buffer;
    std::atomic<std::size_t> head, tail;
};

#endif // LOCKFREEQUEUE_H
