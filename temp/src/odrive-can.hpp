#ifndef ODRIVE_CAN_HPP
#define ODRIVE_CAN_HPP

#include <cstdint>

class OdriveCAN {
public:
    OdriveCAN(int drive_id, const char *can_interface);
    ~OdriveCAN();

    void motor_on();
    void motor_off();
    void set_input_torque(float torque);
    void update_motor_state();
    void get_pos();
    void get_vel();

    int id;
private:
    int socket_fd;
    float pos = 0;
    float vel = 0; 
};

#endif
