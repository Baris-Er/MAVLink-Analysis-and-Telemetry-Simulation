#ifndef DATA_READER_ENCODER_H
#define DATA_READER_ENCODER_H

#include <iostream>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include <chrono>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <csignal>
#include <cstdio>
#include <vector>
#include <cstdint>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "/home/barry/staj_codes/include/common/mavlink.h"

// Constants
#define BUFFER_SIZE 140 //make it smaller if needed
#define SIZE_OF_INT sizeof(int)
#define SHM_SIZE (BUFFER_SIZE * 2 + 2 * SIZE_OF_INT)
#define BUFFER_1_START 0
#define BUFFER_2_START BUFFER_SIZE
#define FLAG_1_OFFSET (BUFFER_SIZE * 2)
#define FLAG_2_OFFSET (FLAG_1_OFFSET + SIZE_OF_INT)

//extern std::atomic<bool> stop_flag;

void signal_handler(int signal);

template<typename T>
class ThreadSafeQueue {
public:
    void push(const T& value);
    void wait_and_pop(T& value);
private:
    std::queue<T> queue;
    std::mutex mtx;
    std::condition_variable cv_not_full;
    std::condition_variable cv_not_empty;
    size_t max_size = 2800;  
};

// Function declarations
void convert_bytes_to_mavlink(const std::vector<uint8_t>& bytes, std::vector<mavlink_message_t>& messages);
void send_mavlink_messages(const std::vector<mavlink_message_t>& messages, int sockfd, const sockaddr_in& server_addr);
void* get_shared_memory();
void read_and_push_buffer(void* shm_ptr, size_t offset, ThreadSafeQueue<std::vector<uint8_t>>& data_queue);
void reader_thread(ThreadSafeQueue<std::vector<uint8_t>>& data_queue, void* shm_ptr);
void converter_thread(ThreadSafeQueue<std::vector<uint8_t>>& data_queue, ThreadSafeQueue<std::vector<mavlink_message_t>>& message_queue);
void sender_thread(ThreadSafeQueue<std::vector<mavlink_message_t>>& message_queue, int sockfd, const sockaddr_in& server_addr);

#endif // DATA_READER_ENCODER_H

