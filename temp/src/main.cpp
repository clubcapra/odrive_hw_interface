#include <cstdio>
#include <unistd.h>

#include "odrive-can.hpp"

int main() {
    OdriveCAN odrive(0,"can0"); // Instantiate OdriveCAN object

    odrive.motor_on();

    odrive.set_input_torque(1);

    for(int i = 0; i < 30; i++){
        odrive.update_motor_state();
        printf("P : %f|| V : %f\n",odrive.get_pos(), odrive.get_vel());
        usleep(100000);
    }

    return 0;
}
