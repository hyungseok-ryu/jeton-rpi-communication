#include <iostream>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <string.h>
#include <csignal>
#include <thread>
#include <mutex>
#include <atomic>
#include <cmath>
#include <sched.h>
#include <time.h>
#include <pthread.h>

#include "candle.hpp"

#define STATE_PORT  5005
#define CMD_PORT    6006
#define JETSON_IP   "172.26.178.73"

int state_sock;
int cmd_sock;

struct sockaddr_in jetson_addr;
struct sockaddr_in rpi_addr;

std::atomic<bool> keep_running(true);
float latest_target = 0.0f;
std::mutex target_mutex;

void set_realtime_priority(int priority = 90) {
    struct sched_param sp;
    sp.sched_priority = priority;
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
        perror("Failed to set real-time priority");
    }
}

void signal_handler(int sig) {
    keep_running = false;
}

void init_udp_sender() {
    state_sock = socket(AF_INET, SOCK_DGRAM, 0);
    memset(&jetson_addr, 0, sizeof(jetson_addr));
    jetson_addr.sin_family = AF_INET;
    jetson_addr.sin_port = htons(STATE_PORT);
    inet_pton(AF_INET, JETSON_IP, &jetson_addr.sin_addr);
}

void init_udp_receiver() {
    cmd_sock = socket(AF_INET, SOCK_DGRAM, 0);
    memset(&rpi_addr, 0, sizeof(rpi_addr));
    rpi_addr.sin_family = AF_INET;
    rpi_addr.sin_port = htons(CMD_PORT);
    rpi_addr.sin_addr.s_addr = INADDR_ANY;
    bind(cmd_sock, (struct sockaddr*)&rpi_addr, sizeof(rpi_addr));

    struct timeval tv{0,10000};
    setsockopt(state_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
}

void udp_receive_thread() {
    set_realtime_priority(80); 

    struct timespec prev_time;
    clock_gettime(CLOCK_MONOTONIC, &prev_time);

    while (keep_running) {
        float cmd;
        ssize_t recv_len = recvfrom(cmd_sock, &cmd, sizeof(cmd), 0, nullptr, nullptr);

        struct timespec now_time;
        clock_gettime(CLOCK_MONOTONIC, &now_time);

        long dt_usec = (now_time.tv_sec - prev_time.tv_sec) * 1'000'000L +
                       (now_time.tv_nsec - prev_time.tv_nsec) / 1000L;
        prev_time = now_time;

        if (recv_len == sizeof(cmd) && std::isfinite(cmd)) {
            {
                std::lock_guard<std::mutex> lock(target_mutex);
                latest_target = cmd;
            }

        } else if (recv_len == -1 && (errno != EAGAIN && errno != EWOULDBLOCK)) {
            perror("[UDP RECEIVE] recvfrom failed");
        }


        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
}

void motor_loop_thread() {
    set_realtime_priority(90);

    mab::Candle candle(mab::CAN_BAUD_8M, true, mab::BusType_E::SPI);
    auto ids = candle.ping(mab::CAN_BAUD_8M);
    if (ids.empty()) return;

    for (auto& id : ids) candle.addMd80(id);
    for (auto& md : candle.md80s) {
        candle.controlMd80SetEncoderZero(md);
        candle.controlMd80Mode(md, mab::Md80Mode_E::IMPEDANCE);
        candle.controlMd80Enable(md, true);
    }

    candle.begin();

    const double interval_sec = 1.0 / 400.0;
    struct timespec next_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time);

	struct timespec prev_time;
	clock_gettime(CLOCK_MONOTONIC, &prev_time);

    float t = 0.0f;

    while (keep_running) {
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, nullptr);

		struct timespec now_time;
		clock_gettime(CLOCK_MONOTONIC, &now_time);
	
		long dt_nsec = (now_time.tv_sec - prev_time.tv_sec) * 1'000'000'000L +
					   (now_time.tv_nsec - prev_time.tv_nsec);
		prev_time = now_time;

        float target;
        {
            std::lock_guard<std::mutex> lock(target_mutex);
            target = latest_target;
        }

        t += interval_sec;

        for (auto& md : candle.md80s) {
            md.setTargetPosition(target);
			// md.setTargetPosition(sin(t) * 2.0f);
            float pos = md.getPosition();
            float vel = md.getVelocity();
            float torque = md.getTorque();

            float data[3] = { pos, vel, torque };
            sendto(state_sock, data, sizeof(data), 0,
                   (struct sockaddr*)&jetson_addr, sizeof(jetson_addr));
        }

        next_time.tv_nsec += static_cast<long>(interval_sec * 1e9);
        while (next_time.tv_nsec >= 1000000000L) {
            next_time.tv_nsec -= 1000000000L;
            next_time.tv_sec += 1;
        }
    }

    candle.end();
}

int main() {
    signal(SIGINT, signal_handler);

    init_udp_sender();
    init_udp_receiver();

    std::thread recv_thread(udp_receive_thread);
    std::thread motor_thread(motor_loop_thread);

    recv_thread.join();
    motor_thread.join();

    close(state_sock);
    close(cmd_sock);

    std::cout << "RPi terminated cleanly.\n";
    return 0;
}
