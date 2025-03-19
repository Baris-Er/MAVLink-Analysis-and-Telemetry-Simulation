import socket

local_qgc_port = 14550
bridge_port = 5057
px4_pymav_port = 5058      #port for decoding mavlink data coming from px4 
px4_local_port = 18570

def create_data_bridge():
    telemetry_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    telemetry_socket.bind(("0.0.0.0", bridge_port))
    print(f"memory created succesfully")
    destination = ("0.0.0.0", px4_pymav_port)
    while True:
        # Receive data
        data, addr = telemetry_socket.recvfrom(1024)    #5057
        telemetry_socket.sendto(data, destination)
         #Check the source of the data
        if addr[1] == local_qgc_port:  
           # Forward data to PX4
           telemetry_socket.sendto(data, ("0.0.0.0", px4_local_port))
        elif addr[1] == px4_local_port:   
            # Forward data to QGC
            telemetry_socket.sendto(data, ("0.0.0.0", local_qgc_port))   
create_data_bridge()