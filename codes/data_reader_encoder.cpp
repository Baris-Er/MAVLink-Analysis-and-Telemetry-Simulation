/*g++ -o data_reader_encoder data_reader_encoder.cpp -I/home/barry/staj_codes -Wno-address-of-packed-member*/

#include </home/barry/staj_codes/data_reader_encoder.h>

/*std::atomic<bool> stop_flag(false);

void signal_handler(int signal) {
    stop_flag = true;
}*/

template<typename T>
void ThreadSafeQueue<T>::push(const T& value) {
    std::unique_lock<std::mutex> lock(mtx);
    cv_not_full.wait(lock, [this]() { return queue.size() < max_size; });
    queue.push(value);
    cv_not_empty.notify_one();
}

template<typename T>
void ThreadSafeQueue<T>::wait_and_pop(T& value) {
    std::unique_lock<std::mutex> lock(mtx);
    cv_not_empty.wait(lock, [this]() { return !queue.empty(); });
    value = queue.front();
    queue.pop();
    cv_not_full.notify_one();
}


sockaddr_in setup_qgc_address() {
    sockaddr_in server_addr;
    std::memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(14550);                             //adress of qgc listening port
    if (inet_pton(AF_INET, "0.0.0.0", &server_addr.sin_addr) <= 0) { 
        perror("Invalid IP address");
        exit(EXIT_FAILURE);
    }
    return server_addr;
}
int setup_socket() {  // Create a udp socket
    
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }

    
    struct sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY; 
    addr.sin_port = htons(5059);       

    
    if (bind(sockfd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("Socket bind failed");
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    return sockfd;
}
const size_t BATCH_SIZE = 100;
void convert_bytes_to_mavlink(const std::vector<uint8_t>& bytes, std::vector<mavlink_message_t>& messages) {
    mavlink_message_t msg;
    mavlink_status_t status;
    std::vector<mavlink_message_t> batch;

    // Iterate over each byte in the buffer
    for (size_t offset = 0; offset < bytes.size(); ++offset) {

        if (mavlink_parse_char(MAVLINK_COMM_0, bytes[offset], &msg, &status)) {
            batch.push_back(msg);
            
            if (batch.size() >= BATCH_SIZE) {
                messages.insert(messages.end(), batch.begin(), batch.end());
                batch.clear();  
            }
        }
    }

    // Add any remaining messages in the batch
    if (!batch.empty()) {
        messages.insert(messages.end(), batch.begin(), batch.end());
    }
}



void send_mavlink_messages(const std::vector<mavlink_message_t>& messages, int sockfd, const sockaddr_in& server_addr) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN * messages.size()];
    size_t offset = 0;

    for (const auto& msg : messages) {
        int len = mavlink_msg_to_send_buffer(buf + offset, &msg);
        offset += len;
    }
    if (sendto(sockfd, buf, offset, 0, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("sendto");
    }
}         

void* get_shared_memory() {
    int fd = shm_open("/mavlink_pro_data", O_RDWR, 0666);
    if (fd == -1) {
        perror("shm_open");
        exit(1);
    }
    void* addr = mmap(NULL, SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (addr == MAP_FAILED) {
        perror("mmap");
        close(fd);
        exit(1);
    }
    close(fd);
    return addr;
}

void read_and_push_buffer(void* shm_ptr, size_t offset, ThreadSafeQueue<std::vector<uint8_t>>& data_queue) {
    size_t flag_offset = (offset == BUFFER_1_START) ? FLAG_1_OFFSET : FLAG_2_OFFSET;

    while (*(int*)(static_cast<char*>(shm_ptr) + flag_offset) != 1) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    uint8_t* buffer = static_cast<uint8_t*>(shm_ptr) + offset;
    std::vector<uint8_t> data(buffer, buffer + BUFFER_SIZE);

    data_queue.push(data);
    std::fill(buffer, buffer + BUFFER_SIZE, 0);
    *(int*)(static_cast<char*>(shm_ptr) + flag_offset) = 0;
}

void reader_thread(ThreadSafeQueue<std::vector<uint8_t>>& data_queue, void* shm_ptr) {
    while (access("/tmp/stop_flag", F_OK) != 0) {
        read_and_push_buffer(shm_ptr, BUFFER_1_START, data_queue);
        //std::this_thread::sleep_for(std::chrono::milliseconds(10));
        read_and_push_buffer(shm_ptr, BUFFER_2_START, data_queue);
        //std::this_thread::sleep_for(std::chrono::milliseconds(10));
        std::cout << "it comes to here reader" << std::endl;

    }
}

void converter_thread(ThreadSafeQueue<std::vector<uint8_t>>& data_queue, ThreadSafeQueue<std::vector<mavlink_message_t>>& message_queue) {
    while (access("/tmp/stop_flag", F_OK) != 0) {
        std::vector<uint8_t> bytes;
        data_queue.wait_and_pop(bytes);
        std::vector<mavlink_message_t> messages;
        convert_bytes_to_mavlink(bytes, messages);
        message_queue.push(messages);
        std::cout << "it comes to here converter" << std::endl;

    }
}

void sender_thread(ThreadSafeQueue<std::vector<mavlink_message_t>>& message_queue, int sockfd, const sockaddr_in& server_addr) {
    while (access("/tmp/stop_flag", F_OK) != 0) {
        std::vector<mavlink_message_t> messages;
        message_queue.wait_and_pop(messages);
        send_mavlink_messages(messages, sockfd, server_addr);
        std::cout << "it comes to here sender" << std::endl;

    }
}

int main() {
    try {
        int sockfd = setup_socket();
        sockaddr_in qgc_addr = setup_qgc_address();
        int sndbuf_size = 8388608; // 8 MB, adjust as needed
        if (setsockopt(sockfd, SOL_SOCKET, SO_SNDBUF, &sndbuf_size, sizeof(sndbuf_size)) < 0) {   //for socket size 
            perror("Setsockopt SO_SNDBUF failed");
        }
        fcntl(sockfd, F_SETFL, O_NONBLOCK);  //maybe making the socket non blocking is better

        void* shm_ptr = get_shared_memory();
        ThreadSafeQueue<std::vector<uint8_t>> data_queue;
        ThreadSafeQueue<std::vector<mavlink_message_t>> message_queue;

        std::thread reader(reader_thread, std::ref(data_queue), shm_ptr);
        std::thread converter(converter_thread, std::ref(data_queue), std::ref(message_queue));
        std::thread sender(sender_thread, std::ref(message_queue), sockfd, std::ref(qgc_addr));

        reader.join();
        converter.join();
        sender.join();

        close(sockfd);

        while (access("/tmp/stop_flag", F_OK) != 0) {     //this looks for creation the stop flag file created by python_mem_writer.py
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        std::remove("/tmp/stop_flag");
        munmap(shm_ptr, SHM_SIZE);         //after python_mem_writer.py closes this frees the memory and gracefully exit
        std::cout << "Unmapping complete, exiting gracefully." << std::endl;

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
}
