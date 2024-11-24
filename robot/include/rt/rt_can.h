/*!
 * @file rt_can.h
 * @brief CAN communication to CANable
 */

#ifndef _rt_can
#define _rt_can

#include "Actuator.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "actuator_command_t.hpp"
#include "actuator_response_t.hpp"
#include <iostream>


#define m1  0x01
#define m2  0x02
#define m3  0x03
#define m4  0x04
#define m5  0x05
#define m6  0x06
#define m7  0x07
#define m8  0x08
#define m9  0x09
#define m10 0x0A
#define m11 0x0B
#define m12 0x0C

class CAN {
    public:
    void init_can();
    void can_send_receive(actuator_command_t* &can_command, actuator_response_t* &can_response);
    void check_safety();
    void stop_can();
    actuator_command_t* get_can_command();
    actuator_response_t *get_can_data();

    private:
    Actuator FR{"can2"}, FL{"can1"}, RR{"can3"}, RL{"can0"};

};





#endif