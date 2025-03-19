/*program list:
1-headers
2-randomness function
3-common variables across messages
4-message creation methods
5- 1,5,10 and 20 Hz generator functions */
//which messages will be send in specific frequency can be configured at the end

#include "message_generator.h"
#include "/home/barry/staj_codes/include/common/mavlink.h" // Ensure this path is correct
#include <sstream>
#include <cstring>
#include <vector>
#include <cstdint>
#include <random>
#include <cmath>
#include <algorithm>
//these libraries are needed to create specific message types
#include <mavlink_types.h>
#include <mavlink_msg_heartbeat.h>
#include <mavlink_msg_sys_status.h>
#include <mavlink_msg_battery_status.h>
#include <mavlink_msg_rc_channels.h>
#include <mavlink_msg_estimator_status.h>
#include <mavlink_msg_mount_orientation.h>
#include <mavlink_msg_landing_target.h>
#include <mavlink_msg_adsb_vehicle.h>
#include <mavlink_msg_mission_current.h>
#include <mavlink_msg_gps_raw_int.h>
#include <mavlink_msg_vfr_hud.h>
#include <mavlink_msg_global_position_int.h>
#include <mavlink_msg_scaled_pressure.h>
#include <mavlink_msg_vibration.h>
#include "message_generator.h"
#include <mavlink_msg_actuator_control_target.h>
#include <mavlink_msg_optical_flow_rad.h>
#include <mavlink_msg_distance_sensor.h>
#include <mavlink_msg_local_position_ned.h>
#include <mavlink_msg_servo_output_raw.h>
#include <mavlink_msg_attitude.h>
#include <mavlink_msg_nav_controller_output.h>
#include <mavlink_msg_altitude.h>
#include <mavlink_msg_attitude.h>
#include <mavlink_msg_attitude_quaternion.h>
#include <mavlink_msg_attitude_target.h>
#include <mavlink_msg_current_event_sequence.h>
//#include <mavlink_msg_current_mode.h>    
#include <mavlink_msg_esc_info.h>
#include <mavlink_msg_esc_status.h>
#include <mavlink_msg_extended_sys_state.h>
#include <mavlink_msg_home_position.h>
#include <mavlink_msg_link_node_status.h>
#include <mavlink_msg_mission_item_reached.h>
#include <mavlink_msg_open_drone_id_location.h>
#include <mavlink_msg_open_drone_id_system.h>
#include <mavlink_msg_ping.h>
#include <mavlink_msg_position_target_global_int.h>
#include <mavlink_msg_position_target_local_ned.h>
#include <mavlink_msg_scaled_pressure.h>
#include <mavlink_msg_servo_output_raw.h>
#include <mavlink_msg_adsb_vehicle.h>

/*this part is for randomising the telemetry values*/
float randomFloat(float min, float max) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(min, max);
    return dis(gen);
}

float randomGaussian(float mean, float stddev) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> dis(mean, stddev);
    return dis(gen);
}

float addNoise(float value, float noise_level, float min_value, float max_value) {
    float noise = randomFloat(-noise_level, noise_level);
    float newValue = value + noise;
    return std::clamp(newValue, min_value, max_value);
}

float exponentialDecay(float value, float decay_rate) {
    return value * std::exp(-decay_rate);
}

int randomInt(int min, int max) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(min, max);
    return dis(gen);
}


float randomWalk(float base, float step, float min_value, float max_value) {
    float change = randomFloat(-step, step);
    float newValue = base + change;
    return std::clamp(newValue, min_value, max_value);
}

/*this part is the data synchronisation across different messages to give the same values*/
static float battery_remaining = 100; 
static float gps_lat = 409000000; 
static float gps_lon = 290000000; 
static float gps_alt = 1000; 

static float velocity_x = 0.0f; 
static float velocity_y = 0.0f; 
static float velocity_z = 0.0f; 

/*start of the message functions*/
std::vector<uint8_t> MessageCreator::createHeartbeatMessage() {
    mavlink_message_t msg;
    mavlink_heartbeat_t heartbeat;
    
    heartbeat.type = MAV_TYPE_QUADROTOR;
    heartbeat.autopilot = MAV_AUTOPILOT_PX4;
    heartbeat.base_mode = (MAV_MODE_FLAG_GUIDED_ENABLED | MAV_MODE_FLAG_SAFETY_ARMED);
    heartbeat.custom_mode = 0;
    heartbeat.system_status = MAV_STATE_ACTIVE;
    heartbeat.mavlink_version = 2;
   

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_heartbeat_encode(1, 1, &msg, &heartbeat);
    len = mavlink_msg_to_send_buffer(buffer, &msg);
    return std::vector<uint8_t>(buffer, buffer + len);
}

std::vector<uint8_t> MessageCreator::createSysStatusMessage() {
    mavlink_message_t msg;
    mavlink_sys_status_t sys_status;
    
    sys_status.onboard_control_sensors_present = 0x0FFF;
    sys_status.onboard_control_sensors_enabled = 0x0FFF;
    sys_status.onboard_control_sensors_health = 0x0FFF;
    sys_status.load = 150;
    sys_status.voltage_battery = randomInt(110, 130); 
    sys_status.current_battery = randomInt(450, 550); 
    sys_status.battery_remaining = static_cast<int8_t>(battery_remaining);      
    sys_status.drop_rate_comm = 0;
    sys_status.errors_comm = 0;
    sys_status.errors_count1 = 0;
    sys_status.errors_count2 = 0;
    sys_status.errors_count3 = 0;
    sys_status.errors_count4 = 0;

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_sys_status_encode(1, 1, &msg, &sys_status);
    len = mavlink_msg_to_send_buffer(buffer, &msg);
    return std::vector<uint8_t>(buffer, buffer + len);
}

//static float battery_remaining = 100;
std::vector<uint8_t> MessageCreator::createBatteryStatusMessage() {
    mavlink_message_t msg;
    mavlink_battery_status_t battery_status;
    
    battery_status.id = 0;
    battery_status.battery_function = 1; 
    battery_status.type = 0; 
    battery_status.temperature = randomInt(2000, 4000);
    //std::fill(std::begin(battery_status.voltages), std::end(battery_status.voltages), 0);
    for (int i = 0; i < 10; ++i) {
        battery_status.voltages[i] = randomInt(11000, 12500); 
    }
    battery_remaining = exponentialDecay(battery_remaining, 0.001f); 
    battery_status.current_battery = randomInt(400, 600); 
    battery_status.battery_remaining = static_cast<int8_t>(battery_remaining); 
    battery_status.current_consumed = randomInt(400, 600);
    battery_status.energy_consumed = randomInt(400, 600);
    

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_battery_status_encode(1, 1, &msg, &battery_status);
    len = mavlink_msg_to_send_buffer(buffer, &msg);
    return std::vector<uint8_t>(buffer, buffer + len);
}

std::vector<uint8_t> MessageCreator::createRcChannelsMessage() {
    mavlink_message_t msg;
    mavlink_rc_channels_t rc_channels;
    
    rc_channels.time_boot_ms = 1100;
    rc_channels.chan1_raw = 1500;
    rc_channels.chan2_raw = 1500;
    rc_channels.chan3_raw = 1500;
    rc_channels.chan4_raw = 1500;
    rc_channels.chan5_raw = 1500;
    rc_channels.chan6_raw = 1500;
    rc_channels.chan7_raw = 1500;
    rc_channels.chan8_raw = 1500;
    rc_channels.chan9_raw = 1500;
    rc_channels.chan10_raw = 1500;
    rc_channels.chan11_raw = 1500;
    rc_channels.chan12_raw = 1500;
    rc_channels.chan13_raw = 1500;
    rc_channels.chan14_raw = 1500;
    rc_channels.chan15_raw = 1500;
    rc_channels.chan16_raw = 1500;
    rc_channels.chan17_raw = 1500;
    rc_channels.chan18_raw = 1500;

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_rc_channels_encode(1, 1, &msg, &rc_channels);
    len = mavlink_msg_to_send_buffer(buffer, &msg);
    return std::vector<uint8_t>(buffer, buffer + len);
}

std::vector<uint8_t> MessageCreator::createEstimatorStatusMessage() {
    mavlink_message_t msg;
    mavlink_estimator_status_t estimator_status;

    estimator_status.time_usec = 10;  
    estimator_status.flags = randomInt(0, 0xFFFF);
    estimator_status.vel_ratio = randomFloat(0.0f, 1.0f);
    estimator_status.pos_horiz_ratio = randomFloat(0.0f, 1.0f);
    estimator_status.pos_vert_ratio = randomFloat(0.0f, 1.0f);
    estimator_status.mag_ratio = randomFloat(0.0f, 1.0f);
    estimator_status.hagl_ratio = randomFloat(0.0f, 1.0f);
    estimator_status.tas_ratio = randomFloat(0.0f, 1.0f);
    estimator_status.pos_horiz_accuracy = randomFloat(0.0f, 10.0f); 
    estimator_status.pos_vert_accuracy = randomFloat(0.0f, 10.0f); 

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    
    
    uint16_t len = mavlink_msg_estimator_status_encode(1, 1, &msg, &estimator_status);
    len = mavlink_msg_to_send_buffer(buffer, &msg);
    
    
    return std::vector<uint8_t>(buffer, buffer + len);
}

std::vector<uint8_t> MessageCreator::createLandingTargetMessage() {
    mavlink_message_t msg;
    mavlink_landing_target_t landing_target;
    
    landing_target.time_usec = 1100000;
    landing_target.angle_x = 1.0f;
    landing_target.angle_y = 1.0f;
    landing_target.target_num = 1;
    landing_target.x = 1.0f;
    landing_target.y = 2.0f;
    landing_target.z = 3.0f;
    landing_target.size_x = 1.0f;
    landing_target.size_y = 1.0f;


    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_landing_target_encode(1, 1, &msg, &landing_target);
    len = mavlink_msg_to_send_buffer(buffer, &msg);
    return std::vector<uint8_t>(buffer, buffer + len);
}

/*std::vector<uint8_t> createAdsbVehicleMessage() {
    mavlink_message_t msg;
    mavlink_adsb_vehicle_t adsb_vehicle;

    adsb_vehicle.ICAO_address = 0xABCDEF;           // Example ICAO address
    adsb_vehicle.lat = 374221000;                   // Example latitude in degrees * 1E7
    adsb_vehicle.lon = -1220840000;                 // Example longitude in degrees * 1E7
    adsb_vehicle.altitude_type = ADSB_ALTITUDE_TYPE_GEOMETRIC; // Example altitude type
    adsb_vehicle.altitude = 100000;                 // Example altitude in millimeters (100m ASL)
    adsb_vehicle.heading = 9000;                    // Example heading in centidegrees (90.00 degrees)
    adsb_vehicle.hor_velocity = 250;                // Example horizontal velocity in cm/s (2.5 m/s)
    adsb_vehicle.ver_velocity = -50;                // Example vertical velocity in cm/s (descending 0.5 m/s)
    const char callsign[] = "N12345    ";
    std::memcpy(adsb_vehicle.callsign, callsign, sizeof(adsb_vehicle.callsign));
    adsb_vehicle.emitter_type = ADSB_EMITTER_TYPE_LIGHTER_AIR; // Example emitter type
    adsb_vehicle.tslc = 10;                         // Example time since last communication in seconds
    adsb_vehicle.flags = (ADSB_FLAGS_VALID_COORDS | ADSB_FLAGS_VALID_ALTITUDE); // Example flags
    adsb_vehicle.squawk = 1200;                     // Example squawk code

    // Encode the message
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_adsb_vehicle_encode(1, 1, &msg, &adsb_vehicle);
    len = mavlink_msg_to_send_buffer(buffer, &msg);

     of bytes
    return std::vector<uint8_t>(buffer, buffer + len);
}*/

std::vector<uint8_t> MessageCreator::createMissionCurrentMessage() {
    mavlink_message_t msg;
    mavlink_mission_current_t mission_current;
    
    mission_current.seq = randomInt(0, 10);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_mission_current_encode(1, 1, &msg, &mission_current);
    len = mavlink_msg_to_send_buffer(buffer, &msg);
    return std::vector<uint8_t>(buffer, buffer + len);
}



std::vector<uint8_t> MessageCreator::createGpsRawIntMessage() {
    mavlink_message_t msg;
    mavlink_gps_raw_int_t gps_raw_int;
    
    gps_lat = randomWalk(gps_lat, 1000, 408000000, 410000000); 
    gps_lon = randomWalk(gps_lon, 1000, 289000000, 291000000); 
    //gps_alt is updated in altitude method 

    gps_raw_int.lat = gps_lat + static_cast<int32_t>(randomGaussian(0.0, 1.0) * 1e7); 
    gps_raw_int.lon = gps_lon + static_cast<int32_t>(randomGaussian(0.0, 1.0) * 1e7); 
    gps_raw_int.alt = gps_alt + static_cast<int32_t>(randomGaussian(0.0, 1.0) * 1e3); 

    gps_raw_int.time_usec = randomInt(0, 1000000000);
    gps_raw_int.eph = randomInt(50, 150);
    gps_raw_int.epv = randomInt(50, 150);
    gps_raw_int.vel = static_cast<int16_t>(sqrt(velocity_x * velocity_x + velocity_y * velocity_y) * 100); 
    gps_raw_int.cog = randomInt(0, 36000);
    gps_raw_int.satellites_visible = randomInt(5, 15);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_gps_raw_int_encode(1, 1, &msg, &gps_raw_int);
    len = mavlink_msg_to_send_buffer(buffer, &msg);
    return std::vector<uint8_t>(buffer, buffer + len);
}

std::vector<uint8_t> MessageCreator::createVfrHudMessage() {
    mavlink_message_t msg;
    mavlink_vfr_hud_t vfr_hud;
    
    vfr_hud.airspeed = randomWalk(vfr_hud.airspeed, 0.5f, 0.0f, 50.0f); 
    vfr_hud.groundspeed = randomWalk(vfr_hud.groundspeed, 0.5f, 0.0f, 50.0f);
    vfr_hud.heading = (vfr_hud.heading + randomInt(-5, 5)) % 360; 
    vfr_hud.throttle = randomInt(0, 100); 
    vfr_hud.alt = randomWalk(vfr_hud.alt, 5.0f, 0.0f, 10000.0f);
    vfr_hud.climb = randomFloat(-5.0f, 5.0f); 

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_vfr_hud_encode(1, 1, &msg, &vfr_hud);
    len = mavlink_msg_to_send_buffer(buffer, &msg);
    return std::vector<uint8_t>(buffer, buffer + len);
}

std::vector<uint8_t> MessageCreator::createGlobalPositionIntMessage() {
    mavlink_message_t msg;
    mavlink_global_position_int_t global_position_int;
    
    global_position_int.time_boot_ms = 123456;
    global_position_int.lat = gps_lat; 
    global_position_int.lon = gps_lon;
    global_position_int.alt = gps_alt; 
    global_position_int.relative_alt = gps_alt - 0; 
    global_position_int.vx = static_cast<int16_t>(velocity_x * 100); 
    global_position_int.vy = static_cast<int16_t>(velocity_y * 100);
    global_position_int.vz = static_cast<int16_t>(velocity_z * 100);
    global_position_int.hdg = 90; 

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_global_position_int_encode(1, 1, &msg, &global_position_int);
    len = mavlink_msg_to_send_buffer(buffer, &msg);
    return std::vector<uint8_t>(buffer, buffer + len);
}

std::vector<uint8_t> MessageCreator::createVibrationMessage() {
    mavlink_message_t msg;
    mavlink_vibration_t vibration;
    
    vibration.time_usec = randomInt(0, 1000000);
    vibration.vibration_x = randomFloat(0.0f, 1.0f); 
    vibration.vibration_y = randomFloat(0.0f, 1.0f);
    vibration.vibration_z = randomFloat(0.0f, 1.0f);
    vibration.clipping_0 = randomInt(0, 10); 
    vibration.clipping_1 = randomInt(0, 10);
    vibration.clipping_2 = randomInt(0, 10);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_vibration_encode(1, 1, &msg, &vibration);
    len = mavlink_msg_to_send_buffer(buffer, &msg);
    return std::vector<uint8_t>(buffer, buffer + len);
}



std::vector<uint8_t> MessageCreator::createActuatorControlTargetMessage() {
    mavlink_message_t msg;
    mavlink_actuator_control_target_t actuator_control_target;

    actuator_control_target.group_mlx = 0;
    for (int i = 0; i < 8; ++i) {
        actuator_control_target.controls[i] = randomWalk(actuator_control_target.controls[i], 0.05f, -1.0f, 1.0f); 
    }

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_actuator_control_target_encode(1, 1, &msg, &actuator_control_target);
    len = mavlink_msg_to_send_buffer(buffer, &msg);
    return std::vector<uint8_t>(buffer, buffer + len);
}

std::vector<uint8_t> MessageCreator::createOpticalFlowRadMessage() {
    mavlink_message_t msg;
    mavlink_optical_flow_rad_t optical_flow_rad;

    optical_flow_rad.time_usec = randomInt(0, 1000000);
    optical_flow_rad.integration_time_us = randomInt(0, 100000);
    optical_flow_rad.integrated_x = randomFloat(-0.1f, 0.1f); 
    optical_flow_rad.integrated_y = randomFloat(-0.1f, 0.1f); 
    optical_flow_rad.integrated_xgyro = randomFloat(-0.1f, 0.1f); 
    optical_flow_rad.integrated_ygyro = randomFloat(-0.1f, 0.1f); 
    optical_flow_rad.integrated_zgyro = randomFloat(-0.1f, 0.1f); 
    optical_flow_rad.temperature = randomInt(2000, 4000); 

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_optical_flow_rad_encode(1, 1, &msg, &optical_flow_rad);
    len = mavlink_msg_to_send_buffer(buffer, &msg);
    return std::vector<uint8_t>(buffer, buffer + len);
}

std::vector<uint8_t> MessageCreator::createDistanceSensorMessage() {
    mavlink_message_t msg;
    mavlink_distance_sensor_t distance_sensor;
    distance_sensor.time_boot_ms = randomInt(0, 1000000);
    distance_sensor.min_distance = randomInt(20, 50); 
    distance_sensor.max_distance = randomInt(500, 1000); 
    distance_sensor.current_distance = randomInt(20, 500); 
    distance_sensor.type = MAV_DISTANCE_SENSOR_LASER; 
    distance_sensor.orientation = MAV_SENSOR_ROTATION_NONE; 
    distance_sensor.covariance = randomInt(0, 10); 
    distance_sensor.id = 0;                    
    distance_sensor.horizontal_fov = 0.5f;    
    distance_sensor.vertical_fov = 0.5f;       
    distance_sensor.quaternion[0] = 1.0f;       
    distance_sensor.quaternion[1] = 0.0f;
    distance_sensor.quaternion[2] = 0.0f;
    distance_sensor.quaternion[3] = 0.0f;
    distance_sensor.signal_quality = randomInt(80, 100);      

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_distance_sensor_encode(1, 1, &msg, &distance_sensor);
    len = mavlink_msg_to_send_buffer(buffer, &msg);
    return std::vector<uint8_t>(buffer, buffer + len);
}

std::vector<uint8_t> MessageCreator::createLocalPositionNedMessage() {
    mavlink_message_t msg;
    mavlink_local_position_ned_t local_position_ned;
    
    local_position_ned.time_boot_ms = randomInt(0, 1000000);
    local_position_ned.x = gps_lat / 1e7 * 111320.0f; 
    local_position_ned.y = gps_lon / 1e7 * 111320.0f * cos(gps_lat / 1e7 * M_PI / 180); 
    local_position_ned.z = -gps_alt; 

    velocity_x = randomWalk(velocity_x, 0.1f, -10.0f, 10.0f); 
    velocity_y = randomWalk(velocity_y, 0.1f, -10.0f, 10.0f);
    velocity_z = randomWalk(velocity_z, 0.1f, -10.0f, 10.0f);

    local_position_ned.vx = static_cast<int16_t>(velocity_x * 100); 
    local_position_ned.vy = static_cast<int16_t>(velocity_y * 100);
    local_position_ned.vz = static_cast<int16_t>(velocity_z * 100);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_local_position_ned_encode(1, 1, &msg, &local_position_ned);
    len = mavlink_msg_to_send_buffer(buffer, &msg);
    return std::vector<uint8_t>(buffer, buffer + len);
}

std::vector<uint8_t> MessageCreator::createServoOutputRawMessage() {
    mavlink_message_t msg;
    mavlink_servo_output_raw_t servo_output_raw;

    servo_output_raw.servo1_raw = randomInt(1000, 2000); 
    servo_output_raw.servo2_raw = randomInt(1000, 2000);
    servo_output_raw.servo3_raw = randomInt(1000, 2000);
    servo_output_raw.servo4_raw = randomInt(1000, 2000);
    servo_output_raw.servo5_raw = randomInt(1000, 2000);
    servo_output_raw.servo6_raw = randomInt(1000, 2000);
    servo_output_raw.servo7_raw = randomInt(1000, 2000);
    servo_output_raw.servo8_raw = randomInt(1000, 2000);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_servo_output_raw_encode(1, 1, &msg, &servo_output_raw);
    len = mavlink_msg_to_send_buffer(buffer, &msg);
    return std::vector<uint8_t>(buffer, buffer + len);
}

std::vector<uint8_t> MessageCreator::createNavControllerOutputMessage() {
    mavlink_message_t msg;
    mavlink_nav_controller_output_t nav_controller_output;

    nav_controller_output.nav_roll = static_cast<int16_t>(randomGaussian(0.0f, 10.0f) * 100); 
    nav_controller_output.nav_pitch = static_cast<int16_t>(randomGaussian(0.0f, 10.0f) * 100);
    nav_controller_output.nav_bearing = static_cast<int16_t>(randomGaussian(0.0f, 30.0f) * 100); 
    nav_controller_output.alt_error = static_cast<int16_t>(addNoise(randomFloat(-10.0f, 10.0f), 2.0f, -1000, 1000) * 100); 
    nav_controller_output.aspd_error = static_cast<int16_t>(addNoise(randomFloat(-5.0f, 5.0f), 1.0f, -500, 500) * 100); 
    nav_controller_output.target_bearing = static_cast<int16_t>(randomGaussian(0.0f, 20.0f) * 100); 
    nav_controller_output.wp_dist = static_cast<uint16_t>(randomWalk(500.0f, 50.0f, 0.0f, 10000.0f)); 
    nav_controller_output.xtrack_error = static_cast<int16_t>(randomFloat(-10.0f, 10.0f) * 100); 

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_nav_controller_output_encode(1, 1, &msg, &nav_controller_output);
    len = mavlink_msg_to_send_buffer(buffer, &msg);
    return std::vector<uint8_t>(buffer, buffer + len);
}



std::vector<uint8_t> MessageCreator::createAltitudeMessage() {
    mavlink_message_t msg;
    mavlink_altitude_t altitude;
    
    altitude.time_usec = randomInt(0, 1000000);
    gps_alt = randomWalk(gps_alt, 10, 950, 1050); 
    altitude.altitude_monotonic = gps_alt; 
    altitude.altitude_amsl = gps_alt + randomFloat(-5.0f, 5.0f); 
    altitude.altitude_local = gps_alt - 10; 
    altitude.altitude_relative = gps_alt - 5; 
    altitude.bottom_clearance = randomFloat(5.0f, 50.0f);   

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_altitude_encode(1, 1, &msg, &altitude);
    len = mavlink_msg_to_send_buffer(buffer, &msg);
    return std::vector<uint8_t>(buffer, buffer + len);
}

std::vector<uint8_t> MessageCreator::createAttitudeMessage() {
    mavlink_message_t msg;
    mavlink_attitude_t attitude;
    
    attitude.roll = randomWalk(attitude.roll, 0.01f, -3.14159f, 3.14159f);   
    attitude.pitch = randomWalk(attitude.pitch, 0.01f, -3.14159f, 3.14159f);  
    attitude.yaw = randomWalk(attitude.yaw, 0.01f, -3.14159f, 3.14159f);    
    attitude.rollspeed = randomGaussian(0.0f, 0.01f); 
    attitude.pitchspeed = randomGaussian(0.0f, 0.01f); 
    attitude.yawspeed = randomGaussian(0.0f, 0.01f);   

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_attitude_encode(1, 1, &msg, &attitude);
    len = mavlink_msg_to_send_buffer(buffer, &msg);
    return std::vector<uint8_t>(buffer, buffer + len);
}

std::vector<uint8_t> MessageCreator::createAttitudeQuaternionMessage() {
    mavlink_message_t msg;
    mavlink_attitude_quaternion_t attitude_quaternion;
    
    attitude_quaternion.time_boot_ms = randomInt(0, 1000000);
    attitude_quaternion.q1 = randomFloat(-1.0f, 1.0f);
    attitude_quaternion.q2 = randomFloat(-1.0f, 1.0f);
    attitude_quaternion.q3 = randomFloat(-1.0f, 1.0f);
    attitude_quaternion.q4 = randomFloat(-1.0f, 1.0f);
    attitude_quaternion.rollspeed = randomFloat(-1.0f, 1.0f); 
    attitude_quaternion.pitchspeed = randomFloat(-1.0f, 1.0f); 
    attitude_quaternion.yawspeed = randomFloat(-1.0f, 1.0f); 

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_attitude_quaternion_encode(1, 1, &msg, &attitude_quaternion);
    len = mavlink_msg_to_send_buffer(buffer, &msg);
    return std::vector<uint8_t>(buffer, buffer + len);
}


std::vector<uint8_t> MessageCreator::createCurrentEventSequenceMessage() {
    mavlink_message_t msg;
    mavlink_current_event_sequence_t event_sequence;
    
    event_sequence.sequence = randomInt(0, 100); 

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_current_event_sequence_encode(1, 1, &msg, &event_sequence);
    len = mavlink_msg_to_send_buffer(buffer, &msg);
    return std::vector<uint8_t>(buffer, buffer + len);
}
std::vector<uint8_t> MessageCreator::createExtendedSysStateMessage() {
    mavlink_message_t msg;
    mavlink_extended_sys_state_t extended_sys_state;
    
    extended_sys_state.landed_state = MAV_LANDED_STATE_IN_AIR;
    extended_sys_state.vtol_state = MAV_VTOL_STATE_UNDEFINED;

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_extended_sys_state_encode(1, 1, &msg, &extended_sys_state);
    len = mavlink_msg_to_send_buffer(buffer, &msg);
    return std::vector<uint8_t>(buffer, buffer + len);
}
std::vector<uint8_t> MessageCreator::createHomePositionMessage() {
    mavlink_message_t msg;
    mavlink_home_position_t home_position;
    
    home_position.time_usec = 0; 
    home_position.latitude = 0;   
    home_position.longitude = 0;  
    home_position.altitude =0 ;        
    home_position.x = 0;
    home_position.y = 0;
    home_position.z = 0; 

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_home_position_encode(1, 1, &msg, &home_position);
    len = mavlink_msg_to_send_buffer(buffer, &msg);
    return std::vector<uint8_t>(buffer, buffer + len);
}
std::vector<uint8_t> MessageCreator::createOpenDroneIDLocationMessage() {
    mavlink_message_t msg;
    mavlink_open_drone_id_location_t location;

    location.target_system = 1;       
    location.target_component = 1;    
    
    std::fill(std::begin(location.id_or_mac), std::end(location.id_or_mac), 0);
    location.id_or_mac[0] = 1;              

    location.status = 1; 
    location.direction = 18000; 
    location.speed_horizontal = static_cast<int16_t>(velocity_x * 100); 
    location.speed_vertical = static_cast<int16_t>(velocity_z * 100); 
    location.latitude = static_cast<int32_t>(gps_lat); 
    location.longitude = static_cast<int32_t>(gps_lon); 
    location.altitude_barometric = static_cast<int32_t>(gps_alt * 1000); 
    location.altitude_geodetic = static_cast<int32_t>(gps_alt * 1000); 
    location.height_reference = 0;
    location.height = 100.0f; 
    location.horizontal_accuracy = 3; 
    location.vertical_accuracy = 3; 
    location.barometer_accuracy = 3; 
    location.speed_accuracy = 5; 
    location.timestamp = 3600.0f; 
    location.timestamp_accuracy = 1; 

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    
    uint16_t len = mavlink_msg_open_drone_id_location_encode(1, 1, &msg, &location);
    len = mavlink_msg_to_send_buffer(buffer, &msg);
   
    return std::vector<uint8_t>(buffer, buffer + len);
}

std::vector<uint8_t> MessageCreator::createOpenDroneIDSystemMessage() {
    mavlink_message_t msg;
    mavlink_open_drone_id_system_t system;

    system.target_system = 1;                 
    system.target_component = 1;              

    std::fill(std::begin(system.id_or_mac), std::end(system.id_or_mac), 0);
    system.id_or_mac[0] = 1;                 

    system.operator_location_type = 1;
    system.classification_type = 2; 
    system.operator_latitude = static_cast<int32_t>(gps_lat); 
    system.operator_longitude = static_cast<int32_t>(gps_lon); 
    system.area_count = 5; 
    system.area_radius = 1000; 
    system.area_ceiling = 1500.0f; 
    system.area_floor = 500.0f; 
    system.category_eu = 1; 
    system.class_eu = 1; 
    system.operator_altitude_geo = 1200.0f; 
    system.timestamp = 1638316800; 

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    

    uint16_t len = mavlink_msg_open_drone_id_system_encode(1, 1, &msg, &system);
    len = mavlink_msg_to_send_buffer(buffer, &msg);
    
    return std::vector<uint8_t>(buffer, buffer + len);
}


std::vector<uint8_t> MessageCreator::createPositionTargetGlobalIntMessage() {
    mavlink_message_t msg;
    mavlink_position_target_global_int_t position_target;

    position_target.time_boot_ms = 0; 
   
    position_target.coordinate_frame = MAV_FRAME_GLOBAL_INT; 
    
    
    position_target.type_mask = 0x00FF; 

    position_target.lat_int = static_cast<int32_t>(randomWalk(gps_lat, 1.0f, gps_lat - 100000, gps_lat + 100000)); 
    position_target.lon_int = static_cast<int32_t>(randomWalk(gps_lon, 1.0f, gps_lon - 100000, gps_lon + 100000)); 
    position_target.alt = randomWalk(gps_alt, 10.0f, 500.0f, 3000.0f); 

    
    position_target.vx = randomFloat(-5.0f, 5.0f); 
    position_target.vy = randomFloat(-5.0f, 5.0f); 
    position_target.vz = randomFloat(-5.0f, 5.0f); 

    position_target.afx = 0.0f;
    position_target.afy = 0.0f;
    position_target.afz = 0.0f;

    
    position_target.yaw = randomFloat(0.0f, 6.28f); 
    position_target.yaw_rate = randomFloat(-0.5f, 0.5f); 

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    
    uint16_t len = mavlink_msg_position_target_global_int_encode(1, 1, &msg, &position_target);
    len = mavlink_msg_to_send_buffer(buffer, &msg);
    
    
    return std::vector<uint8_t>(buffer, buffer + len);
}

std::vector<uint8_t> MessageCreator::createPositionTargetLocalNEDMessage() {
    mavlink_message_t msg;
    mavlink_position_target_local_ned_t pos_target_local_ned;
    pos_target_local_ned.time_boot_ms = 0; 
    
   
    pos_target_local_ned.coordinate_frame = MAV_FRAME_LOCAL_NED; 
    
    
    pos_target_local_ned.type_mask = 0x00FF; 

    
    pos_target_local_ned.x = randomWalk(0.0f, 5.0f, -500.0f, 500.0f); 
    pos_target_local_ned.y = randomWalk(0.0f, 5.0f, -500.0f, 500.0f);
    pos_target_local_ned.z = randomWalk(-100.0f, 5.0f, -500.0f, 0.0f); 

    
    pos_target_local_ned.vx = randomFloat(-5.0f, 5.0f); 
    pos_target_local_ned.vy = randomFloat(-5.0f, 5.0f);
    pos_target_local_ned.vz = randomFloat(-5.0f, 5.0f); 

    
    pos_target_local_ned.afx = 0.0f;
    pos_target_local_ned.afy = 0.0f;
    pos_target_local_ned.afz = 0.0f;

    
    pos_target_local_ned.yaw = randomFloat(0.0f, 6.28f); 
    pos_target_local_ned.yaw_rate = randomFloat(-0.5f, 0.5f); 
     

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_position_target_local_ned_encode(1, 1, &msg, &pos_target_local_ned);
    len = mavlink_msg_to_send_buffer(buffer, &msg);
    return std::vector<uint8_t>(buffer, buffer + len);
}
std::vector<uint8_t> MessageCreator::createScaledPressureMessage() {
    mavlink_message_t msg;
    mavlink_scaled_pressure_t scaled_pressure;

    scaled_pressure.time_boot_ms = 123456;       
    scaled_pressure.press_abs = 1013.25f;        
    scaled_pressure.press_diff = 5.0f;           
    scaled_pressure.temperature = 2500;          
    scaled_pressure.temperature_press_diff = 2600; 

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    
    
    uint16_t len = mavlink_msg_scaled_pressure_encode(1, 1, &msg, &scaled_pressure);
    len = mavlink_msg_to_send_buffer(buffer, &msg);
    
    
    return std::vector<uint8_t>(buffer, buffer + len);
}


std::vector<std::vector<uint8_t>> MessageCreator::createMessagesFor1Hz() {
    std::vector<std::vector<uint8_t>> messages;
    messages.push_back(createHeartbeatMessage());
    messages.push_back(createSysStatusMessage());
    messages.push_back(createBatteryStatusMessage());
    messages.push_back(createMissionCurrentMessage());
    messages.push_back(createExtendedSysStateMessage());
    messages.push_back(createCurrentEventSequenceMessage());
    messages.push_back(createHomePositionMessage());
    messages.push_back(createOpenDroneIDSystemMessage());
    return messages;
}
std::vector<std::vector<uint8_t>> MessageCreator::createMessagesFor5Hz() {
    std::vector<std::vector<uint8_t>> messages;
    messages.push_back(createGpsRawIntMessage());
    messages.push_back(createVfrHudMessage());
    messages.push_back(createGlobalPositionIntMessage());
    messages.push_back(createAltitudeMessage());
    messages.push_back(createPositionTargetGlobalIntMessage());
    messages.push_back(createScaledPressureMessage());
    messages.push_back(createEstimatorStatusMessage());
    messages.push_back(createVibrationMessage());
    return messages;
}
std::vector<std::vector<uint8_t>> MessageCreator::createMessagesFor10Hz() {
    std::vector<std::vector<uint8_t>> messages;
    messages.push_back(createAttitudeMessage());
    messages.push_back(createLocalPositionNedMessage());
    messages.push_back(createOpticalFlowRadMessage());
    messages.push_back(createServoOutputRawMessage());
    messages.push_back(createDistanceSensorMessage());
    messages.push_back(createPositionTargetLocalNEDMessage());
    messages.push_back(createNavControllerOutputMessage());
    return messages;
}
std::vector<std::vector<uint8_t>> MessageCreator::createMessagesFor20Hz() {
    std::vector<std::vector<uint8_t>> messages;
    messages.push_back(createActuatorControlTargetMessage());
    messages.push_back(createAttitudeQuaternionMessage());
    return messages;
}

//messages.push_back(createAdsbVehicleMessage());  it was in 10hz


