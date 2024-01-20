#include "odrive-can.hpp"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/select.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

const uint16_t ODRIVE_ID_MASK = 0x7E0;

const uint16_t ODRIVE_SET_AXIS_STATE = 0x007;
const uint16_t ODRIVE_GET_ENCODER_ESTIMATES = 0x09;
const uint16_t ODRIVE_SET_INPUT_TORQUE = 0x00E;

OdriveCAN::OdriveCAN(int drive_id, const char *can_interface) {
    this->socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (this->socket_fd == -1) {
        perror("Error creating socket");
        return;
    }

    this->id = drive_id;

    struct ifreq ifr;
    std::strcpy(ifr.ifr_name, can_interface);

    if (ioctl(this->socket_fd, SIOCGIFINDEX, &ifr) == -1) {
        perror("Error assigning interface to socket");
        close(this->socket_fd);
        return;
    }

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd, (struct sockaddr*)&addr, sizeof(addr)) == -1) {
        perror("Error binding socket");
        close(this->socket_fd);
        return;
    }

        // Set the read timeout to 1 second (adjust as needed)
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 10000;
    if (setsockopt(this->socket_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == -1) {
        perror("ERROR SETTING TIMEOUT");
        close(this->socket_fd);
        return;
    }

    struct can_filter filter[1];
    filter[0].can_id = this->id;  // Set the CAN ID (will be masked)
    filter[0].can_mask = ODRIVE_ID_MASK;

    if (setsockopt(this->socket_fd, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0) {
        perror("Error setting CAN filter");
        close(this->socket_fd);
        return;
    }
}

OdriveCAN::~OdriveCAN(){
    this->motor_off();
    close(this->socket_fd);
}

void OdriveCAN::motor_on() {
    std::cout << "ODrive MOTOR ON" << std::endl;

    uint32_t data = 0x08;

    struct can_frame request_frame;
    request_frame.can_id = (this->id << 5) | ODRIVE_SET_AXIS_STATE;
    request_frame.can_dlc = sizeof(uint32_t);
    memcpy(request_frame.data, &data, sizeof(data));

    // Send the CAN request frame
    if (write(socket_fd, &request_frame, sizeof(request_frame)) == -1) {
        perror("Error sending CAN request frame");
        close(socket_fd);
        return;//TODO HANDLE ERROR
    }

    usleep(10000);//somehow, first write is discarted, this is here to counter that
}

void OdriveCAN::motor_off() {
    std::cout << "ODrive MOTOR OFF" << std::endl;

    uint32_t data = 0x01;

    struct can_frame request_frame;
    request_frame.can_id = (this->id << 5) | ODRIVE_SET_AXIS_STATE;
    request_frame.can_dlc = sizeof(uint32_t);
    memcpy(request_frame.data, &data, sizeof(data));

    // Send the CAN request frame
    if (write(socket_fd, &request_frame, sizeof(request_frame)) == -1) {
        perror("Error sending CAN request frame");
        close(socket_fd);
        return;//TODO HANDLE ERROR
    }
}


void OdriveCAN::set_input_torque(float torque){
    struct can_frame request_frame;
    request_frame.can_id = (this->id << 5) | ODRIVE_SET_INPUT_TORQUE;
    request_frame.can_dlc = sizeof(float);
    memcpy(request_frame.data, &torque, sizeof(torque));

    // Send the CAN request frame
    if (write(socket_fd, &request_frame, sizeof(request_frame)) == -1) {
        perror("Error sending CAN request frame");
        close(socket_fd);
        return;//TODO HANDLE ERROR
    }
}

void OdriveCAN::update_motor_state(){
    struct can_frame request_frame;
    request_frame.can_id = (this->id << 5) | ODRIVE_GET_ENCODER_ESTIMATES;
    request_frame.can_dlc = 0;

    // Send the CAN request frame
    if (write(socket_fd, &request_frame, sizeof(request_frame)) == -1) {
        perror("Error sending CAN request frame");
        close(socket_fd);
        return;//TODO HANDLE ERROR
    }


    struct can_frame recv_frame;
    int count = 0;
    while (true) {
        ssize_t nbytes = read(socket_fd, &recv_frame, sizeof(struct can_frame));
        if(nbytes == 0){
            perror("NO ANSWER");
            return;
        }
        if(recv_frame.can_id != request_frame.can_id){
            if(count++ > 15){
                perror("NO ANSWER");
                return;
            }
        }
        if(recv_frame.can_id == request_frame.can_id){
            break;
        }
    }
    float loc_pos, loc_vel;
    memcpy(&loc_pos,&recv_frame.data[0],sizeof(float));
    memcpy(&loc_vel,&recv_frame.data[4],sizeof(float));

    this->pos = loc_pos;
    this->vel = loc_vel;
}

float OdriveCAN::get_pos(){
    return(this->pos);
}

float OdriveCAN::get_vel(){
    return(this->vel);
}
