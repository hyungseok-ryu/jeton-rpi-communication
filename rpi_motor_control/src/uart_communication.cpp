#include "uart_communication.h"

UARTCommunication::UARTCommunication(const std::string& device, int baudrate, size_t num_motor)
{
    payload_size_ = sizeof(MotorCommand) * num_motor;
    expected_packet_size_ = 1 + 2 + payload_size_ + 2 + 1;

    fd_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ == -1) {
        std::cerr << "Error " << errno << " opening " << device << ": " << strerror(errno) << std::endl;
        exit(EXIT_FAILURE);
    }

    if (tcgetattr(fd_, &tty_) != 0) {
        std::cerr << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
        close(fd_);
        exit(EXIT_FAILURE);
    }

    cfmakeraw(&tty_);
    cfsetispeed(&tty_, baudrate);
    cfsetospeed(&tty_, baudrate);
    tty_.c_cc[VTIME] = 0;
    tty_.c_cc[VMIN]  = 1;

    if (tcsetattr(fd_, TCSANOW, &tty_) != 0) {
        std::cerr << "Error " << errno << " from tcsetattr: " << strerror(errno) << std::endl;
        close(fd_);
        exit(EXIT_FAILURE);
    }

    usleep(20000);
}

UARTCommunication::~UARTCommunication() {
    if (fd_ != -1) close(fd_);
}

bool UARTCommunication::isOpen() const {
    return fd_ >= 0;
}

bool UARTCommunication::sendState(const MotorState* states, size_t num_motor) {
    if (!isOpen()) return false;

    std::vector<uint8_t> packet;
    packet.reserve(expected_packet_size_);

    packet.push_back(START_BYTE);
    packet.push_back(payload_size_ & 0xFF);
    packet.push_back((payload_size_ >> 8) & 0xFF);

    auto data_ptr = reinterpret_cast<const uint8_t*>(states);
    packet.insert(packet.end(), data_ptr, data_ptr + payload_size_);

    uint16_t crc = crc16(packet.data() + 1, 2 + payload_size_);
    packet.push_back(crc & 0xFF);
    packet.push_back((crc >> 8) & 0xFF);
    packet.push_back(STOP_BYTE);

    return transmit(packet.data(), packet.size());
}

bool UARTCommunication::receive(uint8_t *byte, int timeout_ms) {
    if (fd_ < 0) return false;

    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(fd_, &readfds);

    struct timeval timeout;
    timeout.tv_sec  = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;

    int rv = select(fd_ + 1, &readfds, nullptr, nullptr, &timeout);
    if (rv <= 0) return false;

    return read(fd_, byte, 1) == 1;
}

bool UARTCommunication::readCommand(MotorCommand* commands, size_t num_motor) {
    enum class State { WAIT_START, READ_LEN1, READ_LEN2, READ_PAYLOAD, READ_CRC1, READ_CRC2, READ_STOP };
    State state = State::WAIT_START;

    uint16_t payload_len = 0;
    size_t  idx = 0;
    std::vector<uint8_t> buf;
    buf.reserve(expected_packet_size_);
    uint8_t byte;

    while (true) {
        if (!receive(&byte, 100)) return false;

        switch (state) {
            case State::WAIT_START:
                if (byte == START_BYTE) {
                    buf.clear(); buf.push_back(byte);
                    state = State::READ_LEN1;
                }
                break;

            case State::READ_LEN1:
                buf.push_back(byte);
                payload_len = byte;
                state = State::READ_LEN2;
                break;

            case State::READ_LEN2:
                buf.push_back(byte);
                payload_len |= (uint16_t(byte) << 8);
                if (payload_len != payload_size_) return false;
                idx = 0;
                state = State::READ_PAYLOAD;
                break;

            case State::READ_PAYLOAD:
                buf.push_back(byte);
                if (++idx >= payload_len) state = State::READ_CRC1;
                break;

            case State::READ_CRC1:
                buf.push_back(byte);
                state = State::READ_CRC2;
                break;

            case State::READ_CRC2:
                buf.push_back(byte);
                state = State::READ_STOP;
                break;

            case State::READ_STOP:
                if (byte == STOP_BYTE) buf.push_back(byte);
                else { state = State::WAIT_START; break; }

                uint16_t recv_crc = uint16_t(buf[3 + payload_len]) | (uint16_t(buf[4 + payload_len]) << 8);
                uint16_t calc_crc = crc16(buf.data() + 1, 2 + payload_len);
                if (recv_crc != calc_crc) return false;

                std::memcpy(commands, buf.data() + 3, payload_len);
                return true;
        }
    }
}

bool UARTCommunication::transmit(const uint8_t *data, size_t len) {
    if (fd_ < 0) return false;
    ssize_t sent = 0;
    while (sent < static_cast<ssize_t>(len)) {
        ssize_t w = write(fd_, data + sent, len - sent);
        if (w <= 0) return false;
        sent += w;
    }
    return true;
}

uint16_t UARTCommunication::crc16(const uint8_t *data, size_t len) {
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