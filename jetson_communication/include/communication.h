#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <cstdint>
#include <cstddef>
#include <thread>
#include <chrono>

#define START_BYTE 0xAA
#define STOP_BYTE  0xBB

#pragma pack(push, 1)
struct MotorState
{
    uint32_t drive_ids;                 // Motor ID
    float    position;                  // Position
    float    velocity;                  // Velocity
    float    torque;                    // Torque
    uint8_t  temperature;               // Temperature
    uint16_t quickStatus;               // Status
};
#pragma pack(pop)

#pragma pack(push, 1)
struct MotorCommand {
    uint32_t drive_ids;                 // Motor ID
    float    Kp;                        // P Gain
    float    Kd;                        // D Gain
    float    target_position;           // Target Position
    float    target_velocity;           // Target Velocity
    float    target_torque;             // Target Torque
};
#pragma pack(pop)

#define MAX_MOTOR_NUM 6

#pragma pack(push, 1)
struct HandshakeMessage 
{
    uint32_t num_motors;
    uint32_t motor_ids[MAX_MOTOR_NUM];
};
#pragma pack(pop)

enum class HandshakeResponse : uint8_t {
    OK = 1,
    ERROR = 0
};


class Communication
{
public:
    virtual ~Communication() = default;

    virtual bool sendCommand(const MotorCommand* commands, size_t num_motor) = 0;
    virtual bool readState(MotorState* states, size_t num_motor) = 0;
    virtual bool checkStart(uint32_t* motor_id, size_t num_motor) = 0;
};

#endif // COMMUNICATION_H
