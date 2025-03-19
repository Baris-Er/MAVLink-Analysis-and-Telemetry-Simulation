/*g++ -v -o msg_sender msg_sender.cpp message_generator.cpp -I/home/barry/staj_codes/include/common 
-I/home/barry/staj_codes/include/minimal -I/home/barry/staj_codes/include -I/home/barry/staj_codes/include/standard 
-I/home/barry/staj_codes -Wno-address-of-packed-member*/

//this is how i compiled for making sure every header included mavlink_helpers.h needed to included

/*how the program works: different threads push their each message producing methods' data to lockfreequeue
and consumer thread takes those data and sends to qgc*/

#include "LockFreeQueue.h"
#include <thread>
#include <chrono>
#include <iostream>
#include <random>
#include <cstring>       
#include <string>
#include <sys/types.h>   
#include <sys/socket.h>  
#include <netinet/in.h> 
#include <arpa/inet.h>   
#include <unistd.h>     
#include <message_generator.h>    //this is self made it includes createMessages methods
#include <fcntl.h>

std::atomic<bool> access_flag(false); 
/*this flag is checked with every producer thread this is implemented because when multiple createMessages functions run
at the same time it results in sequence num confusion thus results in %3-7 packet loss */
//all producer threads are nearly same consumer is different

void log_message_rate(const std::string& thread_name, int message_count, std::chrono::duration<double> duration) {
    double rate = message_count / duration.count();
    std::cout << thread_name << " - Messages: " << message_count << ", Duration: " << duration.count() << "s, Rate: " << rate << " messages/sec" << std::endl;
}

void producerThread1Hz(LockFreeQueue<std::vector<uint8_t>>& queue) {
    MessageCreator messageCreator;
    int message_count = 0;  //for debugging
    

    while (true) {
        auto start_time = std::chrono::high_resolution_clock::now();

        while (access_flag.load(std::memory_order_acquire)) {  //looks if flag is true
            
            std::this_thread::sleep_for(std::chrono::milliseconds(1)); //waiting other createMessages functions to finish
        }
        
        access_flag.store(true, std::memory_order_release);
        std::vector<std::vector<uint8_t>> messages = messageCreator.createMessagesFor1Hz();
        message_count += messages.size();

        access_flag.store(false, std::memory_order_release);  //making flag false so that other createMessages stop waiting

        for (const auto& message : messages) {
            queue.enqueue(message);  
        }
        auto process_end_time = std::chrono::high_resolution_clock::now();

        std::chrono::duration<double> process_duration = process_end_time - start_time;
        double elapsed_time = process_duration.count(); 
        double target_interval = 1.0 / 1.0; 
        double sleep_duration = target_interval - elapsed_time;

        if (sleep_duration > 0.0) {      //for sleeping the remaining time
            std::this_thread::sleep_for(std::chrono::duration<double>(sleep_duration));
        }


        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = current_time - start_time;   //these are for debugging,logging
        if (duration.count() >= 10.0) {  
            log_message_rate("ProducerThread1Hz", message_count, duration);
            start_time = current_time;
            message_count = 0;
        }
    }
}

void producerThread5Hz(LockFreeQueue<std::vector<uint8_t>>& queue) {
    MessageCreator messageCreator;
    int message_count = 0;
    

    while (true) {
        auto start_time = std::chrono::high_resolution_clock::now();
        while (access_flag.load(std::memory_order_acquire)) {
            
            std::this_thread::sleep_for(std::chrono::milliseconds(1)); 
        }
        
        access_flag.store(true, std::memory_order_release);
        std::vector<std::vector<uint8_t>> messages = messageCreator.createMessagesFor5Hz();
        message_count += messages.size();

        access_flag.store(false, std::memory_order_release);
      
        for (const auto& message : messages) {
            queue.enqueue(message);  
        }
        auto process_end_time = std::chrono::high_resolution_clock::now();

        std::chrono::duration<double> process_duration = process_end_time - start_time;
        double elapsed_time = process_duration.count(); 
        double target_interval = 1.0 / 5.0; 
        double sleep_duration = target_interval - elapsed_time;

        if (sleep_duration > 0.0) {
            std::this_thread::sleep_for(std::chrono::duration<double>(sleep_duration));
        }

        
        //std::this_thread::sleep_for(std::chrono::milliseconds(223));

       
        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = current_time - start_time;
        if (duration.count() >= 10.0) {  
            log_message_rate("ProducerThread5Hz", message_count, duration);
            start_time = current_time;
            message_count = 0;
        }
    }
}

void producerThread10Hz(LockFreeQueue<std::vector<uint8_t>>& queue) {
    MessageCreator messageCreator;
    int message_count = 0;
    

    while (true) {
        auto start_time = std::chrono::high_resolution_clock::now();
        while (access_flag.load(std::memory_order_acquire)) {
            
            std::this_thread::sleep_for(std::chrono::milliseconds(1)); 
        }
       
        access_flag.store(true, std::memory_order_release);
        std::vector<std::vector<uint8_t>> messages = messageCreator.createMessagesFor10Hz();
        message_count += messages.size();

        access_flag.store(false, std::memory_order_release);

        
        for (const auto& message : messages) {
            queue.enqueue(message);  
        }
        auto process_end_time = std::chrono::high_resolution_clock::now();

        std::chrono::duration<double> process_duration = process_end_time - start_time;
        double elapsed_time = process_duration.count(); 
        double target_interval = 1.0 / 10.0; 
        double sleep_duration = target_interval - elapsed_time;

        if (sleep_duration > 0.0) {
            std::this_thread::sleep_for(std::chrono::duration<double>(sleep_duration));
        }

        //std::this_thread::sleep_for(std::chrono::milliseconds(111));

        
        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = current_time - start_time;
        if (duration.count() >= 10.0) {  
            log_message_rate("ProducerThread10Hz", message_count, duration);
            start_time = current_time;
            message_count = 0;
        }
    }
}

void producerThread20Hz(LockFreeQueue<std::vector<uint8_t>>& queue) {
    MessageCreator messageCreator;
    int message_count = 0;

    while (true) {
        auto start_time = std::chrono::high_resolution_clock::now();
        while (access_flag.load(std::memory_order_acquire)) {
            
            std::this_thread::sleep_for(std::chrono::milliseconds(1)); 
        }
        access_flag.store(true, std::memory_order_release);
        std::vector<std::vector<uint8_t>> messages = messageCreator.createMessagesFor20Hz();
        message_count += messages.size();

        access_flag.store(false, std::memory_order_release);
       
        for (const auto& message : messages) {
            queue.enqueue(message);  
        }
        auto process_end_time = std::chrono::high_resolution_clock::now();
        
        
        std::chrono::duration<double> process_duration = process_end_time - start_time;
        double elapsed_time = process_duration.count(); 
        double target_interval = 1.0 /20.0;   
        double sleep_duration = target_interval - elapsed_time;

        if (sleep_duration > 0.0) {
            std::this_thread::sleep_for(std::chrono::duration<double>(sleep_duration));
        }

        
        //std::this_thread::sleep_for(std::chrono::milliseconds(57)); at first these seconds were chosen to not be divisible to each other for maybe relieving the buffer

       
        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = current_time - start_time;
        if (duration.count() >= 10.0) {  
            log_message_rate("ProducerThread20Hz", message_count, duration);
            start_time = current_time;
            message_count = 0;
        }
    }
}
void consumer(LockFreeQueue<std::vector<uint8_t>>& queue, int sockfd, const sockaddr_in& server_addr) {
    using namespace std::chrono;
    
    while (true) {
        auto start = high_resolution_clock::now(); 
        
        std::vector<uint8_t> message;
        if (!queue.dequeue(message)) {
            std::this_thread::sleep_for(std::chrono::microseconds(350)); 
            //line above is necessary for preventing excessive cpu usage because this thread iterates extremely fast.
            //can be configured according to frequency of the messages sent
        } else {
           
            sendto(sockfd, message.data(), message.size(), 0, (struct sockaddr*)&server_addr, sizeof(server_addr));
        }
        
        auto end = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(end - start).count();
        
        
        if (duration > 1000) { 
            std::cout << "Consumer iteration time: " << duration << " microseconds" << std::endl;
        }
    }
}

sockaddr_in setup_qgc_address() {
    sockaddr_in server_addr;
    std::memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(5057);                             //adress of qgc listening port
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
    addr.sin_port = htons(5061);       

    
    if (bind(sockfd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("Socket bind failed");
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    return sockfd;
}

int main() {
    LockFreeQueue<std::vector<uint8_t>> queue(600); 
    int sockfd = setup_socket();
    sockaddr_in qgc_addr = setup_qgc_address();
    int sndbuf_size = 8388608; // 8 MB, adjust as needed
    if (setsockopt(sockfd, SOL_SOCKET, SO_SNDBUF, &sndbuf_size, sizeof(sndbuf_size)) < 0) {   //for socket size 
        perror("Setsockopt SO_SNDBUF failed");
    }
    fcntl(sockfd, F_SETFL, O_NONBLOCK);  //maybe making the socket non blocking is better 

    std::thread producer1(producerThread1Hz, std::ref(queue));
    std::thread producer2(producerThread5Hz, std::ref(queue));
    std::thread producer3(producerThread10Hz, std::ref(queue));
    std::thread producer4(producerThread20Hz, std::ref(queue));
    std::thread consumerThread(consumer, std::ref(queue), sockfd, qgc_addr);

    producer1.join();
    producer2.join();
    producer3.join();
    producer4.join();
    consumerThread.join();

    return 0;
}
