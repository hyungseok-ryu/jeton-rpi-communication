#pragma once

#include "candle.hpp"

#include "uart_communication.h"
#include "udp_communication.h"

#include <cstdint>
#include <thread>
#include <mutex>
#include <atomic>
#include <sched.h>

class Motor
{
    public:
        Motor(Communication& communication, const uint32_t* motor_id, size_t num_motor);
        ~Motor();

        void initialize();
        void start();
        void stop();
        bool handle_handshake(int cmd_sock, uint32_t* local_motor_ids, uint32_t local_num);

        std::atomic<bool>     running_{true};
    
    private:
        MotorState*         state_;
        MotorCommand*       command_;

        mab::Candle           candle_;
        Communication&        communication_;

        std::thread           state_send_thread_;
        std::thread           cmd_recv_thread_;
        std::thread           control_thread_;
        std::mutex            command_mutex_;

        uint32_t motor_id_[MAX_MOTOR_NUM];
        size_t   num_motor_;

        double send_rate_;
        double recv_rate_;
        double control_rate_;

        long   send_interval_nsec_;
        long   recv_interval_nsec_;
        long   control_interval_nsec_;

        struct sched_param sp;
        
        void sendState();
        void receiveCommand();
        void control();

};