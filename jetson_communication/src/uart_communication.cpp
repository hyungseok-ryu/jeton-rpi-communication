#include "uart_communication.h"

UARTCommunication::UARTCommunication(const std::string& device, int baudrate, size_t num_motor)
{
    payload_size_ = sizeof(MotorState) * num_motor;
    expected_packet_size_ = 1 + 2 + payload_size_ + 2 + 1;

    fd_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ == -1) {
        std::cerr << "Error " << errno << " opening " << device << ": " << strerror(errno) << std::endl;
        exit(EXIT_FAILURE);
    }

    struct termios tty;
    if (tcgetattr(fd_, &tty) != 0) {
        std::cerr << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
        close(fd_);
        exit(EXIT_FAILURE);
    }

    cfmakeraw(&tty);
    cfsetispeed(&tty, baudrate);
    cfsetospeed(&tty, baudrate);
    tty.c_cc[VTIME] = 0;
    tty.c_cc[VMIN]  = 1;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        std::cerr << "Error " << errno << " from tcsetattr: " << strerror(errno) << std::endl;
        close(fd_);
        exit(EXIT_FAILURE);
    }

    usleep(20000);
}

UARTCommunication::~UARTCommunication() {
    if (fd_ >= 0) close(fd_);
}

bool UARTCommunication::isOpen() const {
    return fd_ >= 0;
}

bool UARTCommunication::sendCommand(const MotorCommand* commands, size_t num_motor)
{
    if (!isOpen()) return false;

    size_t payload = sizeof(MotorCommand) * num_motor;
    size_t frame_size = 1 + 2 + payload + 2 + 1;
    std::vector<uint8_t> packet;
    packet.reserve(frame_size);

    packet.push_back(START_BYTE);
    packet.push_back(static_cast<uint8_t>(payload & 0xFF));
    packet.push_back(static_cast<uint8_t>((payload >> 8) & 0xFF));
    
    const uint8_t* data_ptr = reinterpret_cast<const uint8_t*>(commands);
    packet.insert(packet.end(), data_ptr, data_ptr + payload);

    uint16_t crc = crc16(packet.data() + 1, 2 + payload);
    packet.push_back(static_cast<uint8_t>(crc & 0xFF));
    packet.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
    packet.push_back(STOP_BYTE);

    return transmit(packet.data(), packet.size());
}

bool UARTCommunication::readState(MotorState* states, size_t num_motor)
{
    if (!isOpen()) return false;

    size_t payload = sizeof(MotorState) * num_motor;
    size_t frame_size = 1 + 2 + payload + 2 + 1;

    std::vector<uint8_t> packet(frame_size);
    if (!receive(packet.data(), frame_size, 200)) return false;

    size_t offset = 0;
    // START_BYTE 확인
    if (packet[offset++] != START_BYTE) return false;

    // 길이 읽기
    uint16_t len = static_cast<uint16_t>(packet[offset++]) | (static_cast<uint16_t>(packet[offset++]) << 8);
    if (len != payload) return false;

    // 페이로드 복사
    std::memcpy(states, packet.data() + offset, payload);
    offset += payload;

    // CRC 검증
    uint16_t recv_crc = static_cast<uint16_t>(packet[offset++]) | (static_cast<uint16_t>(packet[offset++]) << 8);
    uint16_t calc_crc = crc16(packet.data() + 1, 2 + payload);
    if (recv_crc != calc_crc) return false;

    // STOP_BYTE 확인
    if (packet[offset] != STOP_BYTE) return false;

    return true;
}

bool UARTCommunication::receive(uint8_t* buffer, size_t len, int timeout_ms)
{
    size_t total = 0;
    while (total < len) {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(fd_, &readfds);

        struct timeval tv;
        tv.tv_sec  = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;

        int rv = select(fd_ + 1, &readfds, nullptr, nullptr, &tv);
        if (rv < 0 && errno == EINTR) continue;
        if (rv <= 0) return false;

        ssize_t r = read(fd_, buffer + total, len - total);
        if (r <= 0) return false;
        total += static_cast<size_t>(r);
    }
    return true;
}

bool UARTCommunication::transmit(const uint8_t* data, size_t len)
{
    if (!isOpen()) return false;
    size_t sent = 0;
    while (sent < len) {
        ssize_t w = write(fd_, data + sent, len - sent);
        if (w <= 0) return false;
        sent += static_cast<size_t>(w);
    }
    return true;
}

uint16_t UARTCommunication::crc16(const uint8_t* data, size_t len)
{
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc ^= *data++;
        for (int i = 0; i < 8; ++i) {
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
        }
    }
    return crc;
}

bool UARTCommunication::checkStart(uint32_t* motor_id, size_t num_motor) {
    // UART에서는 핸드셰이크 메시지를 UART 방식에 맞춰 구현해야 합니다.
    // 예시: 단순 무조건 OK 반환 (또는 실제 핸드셰이크 구현 필요)
    std::cout << "[UART HANDSHAKE] checkStart is not implemented. Assuming OK." << std::endl;
    return true;
}