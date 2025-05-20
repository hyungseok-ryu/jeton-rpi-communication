#include "udp_communication.h"

UDPCommunication::UDPCommunication(const std::string& ip, int statePort, int cmdPort) 
{
    stateSock_ = socket(AF_INET, SOCK_DGRAM, 0);
    memset(&jetsonAddr_, 0, sizeof(jetsonAddr_));
    jetsonAddr_.sin_family = AF_INET;
    jetsonAddr_.sin_port   = htons(statePort);
    inet_pton(AF_INET, ip.c_str(), &jetsonAddr_.sin_addr);

    cmdSock_ = socket(AF_INET, SOCK_DGRAM, 0);
    memset(&rpiAddr_, 0, sizeof(rpiAddr_));
    rpiAddr_.sin_family      = AF_INET;
    rpiAddr_.sin_addr.s_addr = INADDR_ANY;
    rpiAddr_.sin_port        = htons(cmdPort);
    bind(cmdSock_, (struct sockaddr*)&rpiAddr_, sizeof(rpiAddr_));

    struct timeval tv{0,10000};
    setsockopt(cmdSock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
}

UDPCommunication::~UDPCommunication() 
{
    if (stateSock_ >= 0) close(stateSock_);
    if (cmdSock_ >= 0) close(cmdSock_);
}

uint16_t UDPCommunication::crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xA001;
            else crc >>= 1;
        }
    }
    return crc;
}

bool UDPCommunication::sendState(const MotorState* states, size_t num_motor)
{
    size_t payload_size = sizeof(MotorState) * num_motor;
    size_t packet_size = 1 + 2 + payload_size + 2 + 1; // Start + Length(2) + Data + CRC(2) + Stop

    uint8_t* packet = new uint8_t[packet_size];
    size_t offset = 0;

    packet[offset++] = START_BYTE;
    packet[offset++] = payload_size & 0xFF;
    packet[offset++] = (payload_size >> 8) & 0xFF;

    std::memcpy(&packet[offset], states, payload_size);
    offset += payload_size;

    uint16_t crc = crc16(&packet[1], 2 + payload_size);
    packet[offset++] = crc & 0xFF;
    packet[offset++] = (crc >> 8) & 0xFF;

    packet[offset++] = STOP_BYTE;

    sendto(stateSock_, packet, packet_size, 0, (struct sockaddr*)&jetsonAddr_, sizeof(jetsonAddr_));

    delete[] packet;
    return true;
}

bool UDPCommunication::readCommand(MotorCommand* commands, size_t num_motor)
{
    size_t expected_payload_size = sizeof(MotorCommand) * num_motor;
    size_t expected_packet_size = 1 + 2 + expected_payload_size + 2 + 1;

    uint8_t* packet = new uint8_t[expected_packet_size];
    ssize_t len = recvfrom(cmdSock_, packet, expected_packet_size, 0, nullptr, nullptr);

    if (len <= 0) {
        delete[] packet;
        return false;
    }

    size_t offset = 0;

    if (packet[offset++] != START_BYTE) {
        std::cerr << "[ERROR] Invalid start byte\n";
        delete[] packet;
        return false;
    }

    uint16_t payload_len = packet[offset++];
    payload_len |= (packet[offset++] << 8);

    if (payload_len != expected_payload_size) {
        std::cerr << "[ERROR] Payload length mismatch\n";
        delete[] packet;
        return false;
    }

    std::memcpy(commands, &packet[offset], payload_len);
    offset += payload_len;

    uint16_t received_crc = packet[offset++];
    received_crc |= (packet[offset++] << 8);

    uint16_t computed_crc = crc16(&packet[1], 2 + payload_len);

    if (received_crc != computed_crc) {
        std::cerr << "[ERROR] CRC mismatch\n";
        delete[] packet;
        return false;
    }

    if (packet[offset] != STOP_BYTE) {
        std::cerr << "[ERROR] Invalid stop byte\n";
        delete[] packet;
        return false;
    }

    delete[] packet;
    return true;
}



bool UDPCommunication::checkStart(uint32_t* motor_id, size_t num_motor) 
{
    HandshakeMessage msg;
    sockaddr_in sender_addr;
    socklen_t addrlen = sizeof(sender_addr);

    std::cout << "[HANDSHAKE] Waiting handshake..." << std::endl;

    while (true)
    {
        ssize_t len = recvfrom(cmdSock_, &msg, sizeof(msg), 0, (sockaddr*)&sender_addr, &addrlen);
        if (len == sizeof(msg)) {
            if (msg.num_motors != num_motor) {
                std::cerr << "[HANDSHAKE] Motor count mismatch!\n";
                HandshakeResponse res = HandshakeResponse::ERROR;
                sendto(cmdSock_, &res, sizeof(res), 0, (sockaddr*)&sender_addr, addrlen);
                return false;
            }
    
            for (size_t i = 0; i < num_motor; ++i) {
                if (msg.motor_ids[i] != motor_id[i]) {
                    std::cerr << "[HANDSHAKE] Motor ID mismatch at index " << i << "\n";
                    HandshakeResponse res = HandshakeResponse::ERROR;
                    sendto(cmdSock_, &res, sizeof(res), 0, (sockaddr*)&sender_addr, addrlen);
                    return false;
                }
            }
            HandshakeResponse res = HandshakeResponse::OK;
            sendto(cmdSock_, &res, sizeof(res), 0, (sockaddr*)&sender_addr, addrlen);
            std::cout << "[HANDSHAKE] Validation successful.\n";
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return false;

}

int UDPCommunication::getCmdSocket() const 
{
    return cmdSock_;
}

int UDPCommunication::getStateSocket() const 
{
    return stateSock_;
}