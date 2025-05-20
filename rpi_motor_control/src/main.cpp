#include "udp_communication.h"
#include "uart_communication.h"
#include "motor.h"

#include <csignal>
#include <atomic>
#include <chrono>
#include <thread>

#define STATE_PORT_RPI2  5002
#define CMD_PORT_RPI2    6002

#define JETSON_IP   "172.26.178.73"

#define UART_DEVICE "/dev/ttyAMA0"
#define BAUDRATE B1000000

#define NUM_MOTOR   1

uint32_t motor_id[NUM_MOTOR] = {200};

UDPCommunication udp_rpi2_comm(JETSON_IP, STATE_PORT_RPI2, CMD_PORT_RPI2);
UARTCommunication uart_comm(UART_DEVICE, BAUDRATE, NUM_MOTOR);

Motor motor(udp_rpi2_comm, motor_id, NUM_MOTOR);

std::atomic<bool> keep_running(true);

void signal_handler(int sig)
{
    keep_running = false;
    motor.stop();
}

int main()
{
    signal(SIGINT, signal_handler);

    // if (!motor.handle_handshake(comm.getCmdSocket(), motor_id, NUM_MOTOR)) 
    // {
    //     std::cerr << "[ERROR] Handshake failed. Exiting.\n";
    //     close(comm.getCmdSocket());
    //     close(comm.getStateSocket());
    //     return -1;
    // }

    motor.initialize();
    motor.start();

    while (keep_running) 
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    return 0;
}
