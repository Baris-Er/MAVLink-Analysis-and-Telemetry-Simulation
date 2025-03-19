#ifndef MESSAGE_CREATOR_H
#define MESSAGE_CREATOR_H

#include <vector>
#include <cstdint>
#include </home/barry/staj_codes/include/mavlink_types.h>

class MessageCreator {
public:
    // Function to create all messages related to their frequency
    static std::vector<std::vector<uint8_t>> createMessagesFor1Hz();
    static std::vector<std::vector<uint8_t>> createMessagesFor5Hz();
    static std::vector<std::vector<uint8_t>> createMessagesFor10Hz();
    static std::vector<std::vector<uint8_t>> createMessagesFor20Hz();

    static std::vector<uint8_t> createHeartbeatMessage();
    static std::vector<uint8_t> createSysStatusMessage();
    static std::vector<uint8_t> createBatteryStatusMessage();
    static std::vector<uint8_t> createRcChannelsMessage();
    static std::vector<uint8_t> createEstimatorStatusMessage();
    static std::vector<uint8_t> createLandingTargetMessage();
    static std::vector<uint8_t> createMissionCurrentMessage();
    static std::vector<uint8_t> createGpsRawIntMessage();
    static std::vector<uint8_t> createVfrHudMessage();
    static std::vector<uint8_t> createGlobalPositionIntMessage();
    static std::vector<uint8_t> createScaledPressureMessage();
    static std::vector<uint8_t> createVibrationMessage();
    static std::vector<uint8_t> createActuatorControlTargetMessage();
    static std::vector<uint8_t> createOpticalFlowRadMessage();
    static std::vector<uint8_t> createDistanceSensorMessage();
    static std::vector<uint8_t> createLocalPositionNedMessage();
    static std::vector<uint8_t> createServoOutputRawMessage();
    static std::vector<uint8_t> createAttitudeMessage();
    static std::vector<uint8_t> createNavControllerOutputMessage();
    //new functions
    static std::vector<uint8_t> createAltitudeMessage();
    static std::vector<uint8_t> createAttitudeQuaternionMessage();
    static std::vector<uint8_t> createCurrentEventSequenceMessage();
    static std::vector<uint8_t> createExtendedSysStateMessage();
    static std::vector<uint8_t> createHomePositionMessage();
    static std::vector<uint8_t> createOpenDroneIDLocationMessage();
    static std::vector<uint8_t> createOpenDroneIDSystemMessage();
    static std::vector<uint8_t> createPositionTargetGlobalIntMessage();
    static std::vector<uint8_t> createPositionTargetLocalNEDMessage(); 
};

#endif // MESSAGE_CREATOR_H

//static std::vector<uint8_t> createEscInfoMessage();
    //static std::vector<uint8_t> createEscStatusMessage();
    //static std::vector<uint8_t> createLinkNodeStatusMessage();   these are not used because they are WIP
    //static std::vector<uint8_t> createAdsbVehicleMessage();