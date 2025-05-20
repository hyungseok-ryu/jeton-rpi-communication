#ifndef UDP_COMMUNICATION_H
#define UDP_COMMUNICATION_H

#include "communication.h"

#include <string>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

class UDPCommunication : public Communication
{
    public:
        UDPCommunication(const std::string& jetsonIp, int statePort, int cmdPort);
        ~UDPCommunication();
        
        bool sendCommand(const MotorCommand* commands, size_t num_motor) override;
        bool readState(MotorState* states, size_t num_motor) override;
        bool checkStart(uint32_t* motor_id, size_t num_motor) override;

        int getCmdSocket() const;
        int getStateSocket() const;

    private:
        uint16_t crc16(const uint8_t *data, size_t len);
        
        int                 stateSock_    = -1;
        int                 cmdSock_      = -1;
        struct sockaddr_in  jetsonAddr_;
        struct sockaddr_in  rpiAddr_;
};

#endif // UDP_COMMUNICATION_H