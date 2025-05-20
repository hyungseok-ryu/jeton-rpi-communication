#include "motor.h"

Motor::Motor(Communication& comm, const uint32_t* motor_id, size_t num_motor)
: communication_(comm),
  candle_(mab::CAN_BAUD_8M, true, mab::BusType_E::SPI),
  num_motor_(num_motor)
{
    for (size_t i = 0; i < num_motor_; ++i) 
    {
        motor_id_[i] = motor_id[i];
    }

    send_rate_      = 1.0 / 100.0;
    recv_rate_      = 1.0 / 100.0;
    control_rate_   = 1.0 / 500.0;

    send_interval_nsec_     = static_cast<long>(send_rate_ * 1e9);
    recv_interval_nsec_     = static_cast<long>(recv_rate_ * 1e9);
    control_interval_nsec_  = static_cast<long>(control_rate_ * 1e9);

    state_   = new MotorState[num_motor_];
    command_ = new MotorCommand[num_motor_];

    communication_.checkStart(motor_id_, num_motor_);
}

Motor::~Motor()
{
    // Destructor implementation
}

void Motor::initialize()
{
    auto ids = candle_.ping(mab::CAN_BAUD_8M);
    if (ids.empty()) return;

    for (auto& id : ids) candle_.addMd80(id);
    for (auto& md : candle_.md80s) {
        candle_.controlMd80SetEncoderZero(md);
        candle_.controlMd80Mode(md, mab::Md80Mode_E::IMPEDANCE);
        candle_.controlMd80Enable(md, true);
    }
}

void Motor::start()
{   
    running_ = true;

    state_send_thread_  = std::thread(&Motor::sendState, this);
    cmd_recv_thread_    = std::thread(&Motor::receiveCommand, this);
    control_thread_     = std::thread(&Motor::control, this);
}

void Motor::stop()
{
    running_ = false;
    if (state_send_thread_.joinable()) state_send_thread_.join();
    if (cmd_recv_thread_.joinable())  cmd_recv_thread_.join();
    if (control_thread_.joinable()) control_thread_.join();
    candle_.end();
}

void Motor::sendState()
{
    sp.sched_priority = 90;
    sched_setscheduler(0, SCHED_FIFO, &sp);

    candle_.begin();
    struct timespec next_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time);

    while (running_) 
    {
        for (size_t idx = 0; idx < candle_.md80s.size(); ++idx) 
        {
            auto& md = candle_.md80s[idx];
        
            state_[idx].drive_ids    = md.getId();
            state_[idx].position     = md.getPosition();
            state_[idx].velocity     = md.getVelocity();
            state_[idx].torque       = md.getTorque();
            state_[idx].temperature  = md.getTemperature();
            state_[idx].quickStatus  = md.getQuickStatus();
        }
        communication_.sendState(state_, num_motor_);
        
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, nullptr);

        next_time.tv_nsec += static_cast<long>(send_interval_nsec_);
        while (next_time.tv_nsec >= 1000000000L) 
        {
            next_time.tv_nsec -= 1000000000L;
            next_time.tv_sec += 1;
        }
    }
}

void Motor::receiveCommand()
{
    sp.sched_priority = 80;
    sched_setscheduler(0, SCHED_FIFO, &sp);

    struct timespec next_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time);

    while (running_) {
        MotorCommand temp_cmds[MAX_MOTOR_NUM];
        communication_.readCommand(temp_cmds, num_motor_);

        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            for (size_t i = 0; i < num_motor_; ++i) 
            {
                command_[i] = temp_cmds[i];
            }
        }

        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, nullptr);

        next_time.tv_nsec += static_cast<long>(recv_interval_nsec_);
        while (next_time.tv_nsec >= 1000000000L) {
            next_time.tv_nsec -= 1000000000L;
            next_time.tv_sec += 1;
        }
    }
}

void Motor::control()
{
    sp.sched_priority = 95;
    sched_setscheduler(0, SCHED_FIFO, &sp);

    struct timespec next_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time);

    while (running_)
    {
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, nullptr);

        MotorCommand temp_cmds[MAX_MOTOR_NUM];
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            for (size_t i = 0; i < num_motor_; ++i)
                temp_cmds[i] = command_[i];
        }

        for (auto& md : candle_.md80s) 
        {
            uint32_t id = md.getId();

            for (size_t i = 0; i < num_motor_; ++i) 
            {
                if (motor_id_[i] == id) 
                {
                    md.setImpedanceControllerParams(temp_cmds[i].Kp, temp_cmds[i].Kd);
                    md.setTargetPosition(temp_cmds[i].target_position);
                    md.setTargetVelocity(temp_cmds[i].target_velocity);
                    md.setTargetTorque(temp_cmds[i].target_torque);
                    std::cout << "Motor ID: " << id << ", Target Position: " << temp_cmds[i].target_position << std::endl;
                    break;
                }
            }
        }

        next_time.tv_nsec += control_interval_nsec_;
        while (next_time.tv_nsec >= 1'000'000'000L) {
            next_time.tv_nsec -= 1'000'000'000L;
            next_time.tv_sec  += 1;
        }
    }
}

bool Motor::handle_handshake(int cmd_sock, uint32_t* local_motor_ids, uint32_t local_num) {
    HandshakeMessage msg;
    sockaddr_in sender_addr;
    socklen_t addrlen = sizeof(sender_addr);

    std::cout << "[HANDSHAKE] Waiting handshake..." << std::endl;

    while (true)
    {
        ssize_t len = recvfrom(cmd_sock, &msg, sizeof(msg), 0, (sockaddr*)&sender_addr, &addrlen);
        if (len == sizeof(msg)) {
            if (msg.num_motors != local_num) {
                std::cerr << "[HANDSHAKE] Motor count mismatch!\n";
                HandshakeResponse res = HandshakeResponse::ERROR;
                sendto(cmd_sock, &res, sizeof(res), 0, (sockaddr*)&sender_addr, addrlen);
                return false;
            }
    
            for (size_t i = 0; i < local_num; ++i) {
                if (msg.motor_ids[i] != local_motor_ids[i]) {
                    std::cerr << "[HANDSHAKE] Motor ID mismatch at index " << i << "\n";
                    HandshakeResponse res = HandshakeResponse::ERROR;
                    sendto(cmd_sock, &res, sizeof(res), 0, (sockaddr*)&sender_addr, addrlen);
                    return false;
                }
            }
            HandshakeResponse res = HandshakeResponse::OK;
            sendto(cmd_sock, &res, sizeof(res), 0, (sockaddr*)&sender_addr, addrlen);
            std::cout << "[HANDSHAKE] Validation successful.\n";
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    return false;

}
