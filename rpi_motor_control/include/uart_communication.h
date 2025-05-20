#ifndef UART_COMMUNICATION_H
#define UART_COMMUNICATION_H

#include "communication.h"

#include <fcntl.h>
#include <cstdint>
#include <iostream>
#include <termios.h>
#include <cstring> 
#include <unistd.h>
#include <vector>
#include <map>

class UARTCommunication : public Communication
{
    public:
        UARTCommunication(const std::string& device, int baudrate, size_t num_motor);
        ~UARTCommunication();

        bool isOpen() const;

        bool sendState(const MotorState* states, size_t num_motor) override;
        bool readCommand(MotorCommand* commands, size_t num_motor) override;
        bool checkStart(uint32_t* motor_id, size_t num_motor) override;

    private:
        int                     fd_;
        struct termios          tty_;
        static const uint32_t   maxResponseLen = 2000;

        size_t                  payload_size_;
        size_t                  expected_packet_size_;

        bool transmit(const uint8_t *buffer, size_t len);
        bool receive(uint8_t *buffer, int timeout_ms = 500);

        uint16_t crc16(const uint8_t *buffer, size_t len);
};

#endif // UART_COMMUNICATION_H