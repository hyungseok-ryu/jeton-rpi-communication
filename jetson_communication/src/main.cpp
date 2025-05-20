#include "udp_communication.h"
#include "uart_communication.h"
#include "communication.h"
#include "motor.h"

#include <csignal>
#include <atomic>
#include <chrono>
#include <thread>
#include <cmath>

#define STATE_PORT_RPI1  5001
#define CMD_PORT_RPI1    6001

#define STATE_PORT_RPI2  5002
#define CMD_PORT_RPI2    6002

#define IP_RPI1        "172.26.178.79"
#define IP_RPI2        "172.26.178.78"

#define UART_DEVICE "/dev/ttyTHS1"
#define BAUDRATE B1000000

#define NUM_MOTOR_RPI1 1
#define NUM_MOTOR_RPI2 1

uint32_t motor_id_RPI1[NUM_MOTOR_RPI1] = {200};
uint32_t motor_id_RPI2[NUM_MOTOR_RPI2] = {200};

UDPCommunication udp_rpi1_comm(IP_RPI1, STATE_PORT_RPI1, CMD_PORT_RPI1);
UDPCommunication udp_rpi2_comm(IP_RPI2, STATE_PORT_RPI2, CMD_PORT_RPI2);

UARTCommunication uart_comm(UART_DEVICE, BAUDRATE, NUM_MOTOR_RPI1);

Motor motor1(udp_rpi1_comm, motor_id_RPI1, NUM_MOTOR_RPI1);
Motor motor2(udp_rpi2_comm, motor_id_RPI2, NUM_MOTOR_RPI2);

std::atomic<bool> keep_running(true);

void signal_handler(int sig)
{
    keep_running = false;
    motor1.stop();
}

int main()
{
    signal(SIGINT, signal_handler);

    motor1.initialize();
    motor1.start();

    float t = 0.0f;
    const double dt = 1.0 / 100.0;
    struct timespec next_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time);

    while (keep_running) 
    {
        MotorCommand cmds[NUM_MOTOR_RPI1];
        for (size_t i = 0; i < NUM_MOTOR_RPI1; ++i) 
        {
            cmds[i].drive_ids = motor_id_RPI1[i];
            cmds[i].Kp = 1.0f;
            cmds[i].Kd = 0.0f;
            cmds[i].target_position = std::sin(t + i) * 2.0f;
            cmds[i].target_velocity = 0.0f;
            cmds[i].target_torque = 0.0f;
        }

        motor1.setCommand(cmds, NUM_MOTOR_RPI1);
        t += dt;

        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, nullptr);
        next_time.tv_nsec += static_cast<long>(dt * 1e9);
        while (next_time.tv_nsec >= 1'000'000'000L) {
            next_time.tv_nsec -= 1'000'000'000L;
            next_time.tv_sec += 1;
        }
    }

    return 0;
}
