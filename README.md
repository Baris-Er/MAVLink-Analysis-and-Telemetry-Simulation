# MAVLink Analysis and Telemetry Simulation
MAVLink protocol analysis and telemetry simulation toolkit for UAV communications. For more detailed information, please refer to the documentation in the 'docs' folder.

![Project Diagram](images/system_overview.png "System Overview")

In the system depicted in the diagram above, data flows between the PX4 and QGroundControl (QGC) through a series of components and processes. The data from PX4 is received through the Data Bridge port, to MAVLink decoding and encoding application. This process utilizes shared memory accessible by both Python and C++ applications. The encoded MAVLink messages are then sent to the Data Bridge port, which forwards them to QGC.

Conversely, data from QGC is routed back to PX4 through the Data Bridge port, ensuring bidirectional communication. For scenarios where QGC is not used or when testing message transmission, the mavlink_listener.py script can be employed to directly decode incoming messages. This serves as a tool for verifying message integrity and communication in the absence of QGC.

Additionally, the msg_sender.cpp component can be utilized to generate custom telemetry data replacing PX4, which can be directed to QGC or the Data Bridge port or the port that python_mem_writer.py listens for further processing. This setup provides flexibility for testing and validating the system’s functionality under various conditions.

## Key Features

- **MAVLink Encoding/Decoding:** Efficient handling of MAVLink message structures.
- **Telemetry Simulation:** Generate custom or randomized telemetry data with `msg_sender.cpp`.
- **Python & C++ Integration:** Utilize shared memory for real-time data exchange between applications.
- **Flexible Data Bridge:** Seamlessly forward messages between PX4 and QGC.
- **Standalone Testing:** Use `mavlink_listener.py` to decode messages when QGC is not in use.

## Requirements
- In this project, both QGroundControl and PX4 are installed and configured on the **Windows Subsystem for Linux (WSL)**. This is necessary because these tools are primarily designed to run on Linux environments.
- In different configurations, it may be necessary to adjust the PX4 socket's sending address to ensure seamless integration between QGroundControl and our implementation.
