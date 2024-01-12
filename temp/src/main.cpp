#include "odrive-can.hpp"
#include <unistd.h>

int main() {
    OdriveCAN odrive(0,"can0"); // Instantiate OdriveCAN object

    odrive.motor_on();
    
    sleep(1);

    odrive.set_input_torque(0.1);

    sleep(3);

    odrive.motor_off();

    return 0;
}
