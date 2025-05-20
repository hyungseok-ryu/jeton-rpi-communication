#include "motor.h"

Motor::Motor(Communication& comm, const uint32_t* motor_id, size_t num_motor)
: communication_(comm),
  num_motor_(num_motor)
{
    for (size_t i = 0; i < num_motor_; ++i) 
    {
        motor_id_[i] = motor_id[i];
    }

    send_rate_      = 1.0 / 100.0;
    recv_rate_      = 1.0 / 100.0;

    send_interval_nsec_     = static_cast<long>(send_rate_ * 1e9);
    recv_interval_nsec_     = static_cast<long>(recv_rate_ * 1e9);

    state_   = new MotorState[num_motor_];
    command_ = new MotorCommand[num_motor_];
}

Motor::~Motor()
{
    // Destructor implementation
}

void Motor::initialize()
{
    communication_.checkStart(motor_id_, num_motor_);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

}

void Motor::start()
{   
    running_ = true;

    std::cout << "[INFO] Motor started.\n";

    cmd_send_thread_  = std::thread(&Motor::sendCommand, this);
    state_recv_thread_    = std::thread(&Motor::receiveState, this);
}

void Motor::stop()
{
    running_ = false;
    if (cmd_send_thread_.joinable()) cmd_send_thread_.join();
    if (state_recv_thread_.joinable())  state_recv_thread_.join();
}

void Motor::setCommand(const MotorCommand* new_cmds, size_t count) 
{
    std::lock_guard<std::mutex> lock(command_mutex_);
    for (size_t i = 0; i < std::min(count, num_motor_); ++i) 
    {
        command_[i] = new_cmds[i];
    }
}

void Motor::sendCommand()
{
    struct timespec next_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time);

    while (running_) 
    {
        MotorCommand temp_cmds[num_motor_];
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            for (size_t i = 0; i < num_motor_; ++i)
                temp_cmds[i] = command_[i];
        }

        last_send_time_ns.store(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                    std::chrono::high_resolution_clock::now().time_since_epoch()).count());

        communication_.sendCommand(temp_cmds, num_motor_);

        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, nullptr);
        next_time.tv_nsec += (long)(send_interval_nsec_);
        while (next_time.tv_nsec >= 1'000'000'000L) {
            next_time.tv_nsec -= 1'000'000'000L;
            next_time.tv_sec += 1;
        }
    }
}

void Motor::receiveState()
{
    struct timespec receive_state_time;
    clock_gettime(CLOCK_MONOTONIC, &receive_state_time);

    while (running_) 
    {
        MotorState temp_state[num_motor_];
        if (communication_.readState(temp_state, num_motor_)) 
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            for (size_t i = 0; i < num_motor_; ++i) 
            {
                state_[i] = temp_state[i];
            }
        
            long long latency_us = (std::chrono::duration_cast<std::chrono::nanoseconds>(
                                        std::chrono::high_resolution_clock::now().time_since_epoch()).count() -
                                        last_send_time_ns.load()) / 1000;
        
            for (int i = 0; i < num_motor_; ++i) 
            {
                std::cout << "[STATE] " << "ID: " << state_[i].drive_ids
                          << ", pos: " << state_[i].position
                          << ", vel: " << state_[i].velocity
                          << ", torque: " << state_[i].torque
                          << ", temp: " << (int)state_[i].temperature
                          << ", status: " << state_[i].quickStatus
                          << ", latency: " << latency_us << " us\n";
            }
        }
        else
        {
            std::cerr << "[WARN] Failed to read MotorState.\n";
        }        

        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &receive_state_time, nullptr);

        receive_state_time.tv_nsec += (long)(recv_interval_nsec_);
        while (receive_state_time.tv_nsec >= 1'000'000'000L) {
            receive_state_time.tv_nsec -= 1'000'000'000L;
            receive_state_time.tv_sec += 1;
        }
    }
}