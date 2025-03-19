from pymavlink import mavutil
import asyncio
import os
import mmap
import sys
import struct
import signal

# Constants
px4_pymav_port = 5058
BUFFER_SIZE = 140   # or 560 but 140 works fine
SIZE_OF_INT = struct.calcsize('i')  # Size of integer in bytes
SHM_SIZE = BUFFER_SIZE * 2 + 2 * SIZE_OF_INT  
BUFFER_1_START = 0
BUFFER_2_START = BUFFER_SIZE
FLAG_1_OFFSET = BUFFER_SIZE * 2
FLAG_2_OFFSET = FLAG_1_OFFSET + SIZE_OF_INT
stop_flag_file = '/tmp/stop_flag'

def signal_handler(signum, frame):
    # Create a flag file to indicate termination this file will be read by data_reader_encoder.cpp for memory freeing
    with open(stop_flag_file, 'w') as f:
        f.write('stop')
        print(f"works stop file")
    sys.exit(0)

#there was signal handler here i moved it to main

async def read_messages(connection, queue):
    buffer = bytearray()
    
    while True:
        if os.path.exists(stop_flag_file):
            # exit loop if stop flag is detected
            break
        
        msg = connection.recv_match(blocking=False)
        if not msg:
            await asyncio.sleep(0.01)
            continue
        
        mavlink_bytes = msg.get_msgbuf()  # get MAVLink message as bytes
        if len(mavlink_bytes) == 0:
            print("Warning: Empty MAVLink message received.")
            continue
        
        if len(mavlink_bytes) > BUFFER_SIZE:
            print(f"Error: MAVLink message exceeds buffer size ({len(mavlink_bytes)} > {BUFFER_SIZE})")
            continue
        
        # Add message to buffer
        buffer.extend(mavlink_bytes)
        
        # Check if buffer size has reached BUFFER_SIZE
        if len(buffer) >= BUFFER_SIZE:
            await queue.put(bytes(buffer[:BUFFER_SIZE]))  
            buffer = buffer[BUFFER_SIZE:]  
    # push remaining data in buffer if there's any
    if buffer:
        await queue.put(bytes(buffer))  

def initialize_shared_memory():
    try:
        fd = os.open("/dev/shm/mavlink_pro_data", os.O_CREAT | os.O_RDWR, 0o666)       #think about file
        os.ftruncate(fd, SHM_SIZE)
        shm = mmap.mmap(fd, SHM_SIZE)
        
        shm[FLAG_1_OFFSET:FLAG_1_OFFSET + SIZE_OF_INT] = (0).to_bytes(SIZE_OF_INT, byteorder='little')
        shm[FLAG_2_OFFSET:FLAG_2_OFFSET + SIZE_OF_INT] = (0).to_bytes(SIZE_OF_INT, byteorder='little')
        print(f"memory created succesfully")
        return shm
    except Exception as e:
        print(f"Error initializing shared memory: {e}")
        sys.exit(1)

async def write_to_shared_memory(queue):
    buffer_switch = True
    shm = initialize_shared_memory()  

    while True:
        try:
            if queue.empty():
                print("Queue is empty, sleeping.")
                await asyncio.sleep(0.01)
                continue

            mavlink_bytes = await queue.get()
            offset = BUFFER_1_START if buffer_switch else BUFFER_2_START
            flag_offset = FLAG_1_OFFSET if buffer_switch else FLAG_2_OFFSET

            # Wait until the flag indicates that the buffer is ready
            while int.from_bytes(shm[flag_offset:flag_offset + SIZE_OF_INT], byteorder='little') != 0:
                await asyncio.sleep(0.01)

            bytes_written = 0
            while bytes_written < len(mavlink_bytes):
                remaining_space = BUFFER_SIZE - (offset % BUFFER_SIZE)
                chunk_size = min(remaining_space, len(mavlink_bytes) - bytes_written)

                # Write a chunk of data to the shared memory buffer
                shm[offset:offset + chunk_size] = mavlink_bytes[bytes_written:bytes_written + chunk_size]
                bytes_written += chunk_size
                offset = (offset + chunk_size) % BUFFER_SIZE  # Wrap around if necessary

                # Check if we need to switch buffers
                if (offset % BUFFER_SIZE) == 0:
                    break

            # Set the flag to indicate the buffer is filled
            shm[flag_offset:flag_offset + SIZE_OF_INT] = (1).to_bytes(SIZE_OF_INT, byteorder='little')

            # Switch buffers
            buffer_switch = not buffer_switch

        except Exception as e:
            print(f"Error writing to shared memory: {e}")
            sys.exit(1)
            

async def main():
    try:
        signal.signal(signal.SIGINT, signal_handler)
        connection_string = 'udpin:0.0.0.0:%d' % px4_pymav_port
        pymav_analysis_port = mavutil.mavlink_connection(connection_string)
        print(f"it works until here")
        queue = asyncio.Queue()
        await asyncio.gather(read_messages(pymav_analysis_port, queue), write_to_shared_memory(queue))
        print(f"it should have never reached here")
    except Exception as e:
        print(f"Error in main function: {e}")
        sys.exit(1)

asyncio.run(main())