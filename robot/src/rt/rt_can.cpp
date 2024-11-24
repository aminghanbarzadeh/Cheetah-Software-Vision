#include "rt/rt_can.h"

actuator_command_t* can_cmd;
actuator_response_t* can_data;

void CAN::init_can() {
  
  FR.enable(m1);
  FR.enable(m2);
  FR.enable(m3);
  printf("[IUSTHardware] Leg 1 is enabled!\n");

  FL.enable(m4);
  FL.enable(m5);
  FL.enable(m6);
  printf("[IUSTHardware] Leg 2 is enabled!\n");
  
  RR.enable(m7);
  RR.enable(m8);
  RR.enable(m9);
  printf("[IUSTHardware] Leg 3 is enabled!\n");
  
  RL.enable(m10);
  RL.enable(m11);
  RL.enable(m12);
  printf("[IUSTHardware] Leg 4 is enabled!\n");

  // can_cmd.

}

void CAN::can_send_receive(actuator_command_t* &can_command, actuator_response_t* &can_response) {

  FR.command(m1, can_command[0].position,can_command[0].velocity,can_command[0].kp,can_command[0].kd,can_command[0].tau);
  FL.command(m4, can_command[3].position,can_command[3].velocity,can_command[3].kp,can_command[3].kd,can_command[3].tau);
  RR.command(m7, can_command[6].position,can_command[6].velocity,can_command[6].kp,can_command[6].kd,can_command[6].tau);
  RL.command(m10, can_command[9].position,can_command[9].velocity,can_command[9].kp,can_command[9].kd,can_command[9].tau);

  FR.command(m2, can_command[1].position,can_command[1].velocity,can_command[1].kp,can_command[1].kd,can_command[1].tau);
  FL.command(m5, can_command[4].position,can_command[4].velocity,can_command[4].kp,can_command[4].kd,can_command[4].tau);
  RR.command(m8, can_command[7].position,can_command[7].velocity,can_command[7].kp,can_command[7].kd,can_command[7].tau);
  RL.command(m11, can_command[10].position,can_command[10].velocity,can_command[10].kp,can_command[10].kd,can_command[10].tau);

  FR.command(m3, can_command[2].position,can_command[2].velocity,can_command[2].kp,can_command[2].kd,can_command[2].tau);
  FL.command(m6, can_command[5].position,can_command[5].velocity,can_command[5].kp,can_command[5].kd,can_command[5].tau);
  RR.command(m9, can_command[8].position,can_command[8].velocity,can_command[8].kp,can_command[8].kd,can_command[8].tau);
  RL.command(m12, can_command[11].position,can_command[11].velocity,can_command[11].kp,can_command[11].kd,can_command[11].tau);


  can_response[0].position = FR.cycle_responses[0][1];
  can_response[1].position = FR.cycle_responses[1][1];
  can_response[2].position = FR.cycle_responses[2][1];

  can_response[3].position = FL.cycle_responses[0][1];
  can_response[4].position = FL.cycle_responses[1][1];
  can_response[5].position = FL.cycle_responses[2][1];

  can_response[6].position = RR.cycle_responses[0][1];
  can_response[7].position = RR.cycle_responses[1][1];
  can_response[8].position = RR.cycle_responses[2][1];

  can_response[9].position = RL.cycle_responses[0][1];
  can_response[10].position = RL.cycle_responses[1][1];
  can_response[11].position = RL.cycle_responses[2][1];


  can_response[0].velocity = FR.cycle_responses[0][2];
  can_response[1].velocity = FR.cycle_responses[1][2];
  can_response[2].velocity = FR.cycle_responses[2][2];

  can_response[3].velocity = FL.cycle_responses[0][2];
  can_response[4].velocity = FL.cycle_responses[1][2];
  can_response[5].velocity = FL.cycle_responses[2][2];

  can_response[6].velocity = RR.cycle_responses[0][2];
  can_response[7].velocity = RR.cycle_responses[1][2];
  can_response[8].velocity = RR.cycle_responses[2][2];

  can_response[9].velocity = RL.cycle_responses[0][2];
  can_response[10].velocity = RL.cycle_responses[1][2];
  can_response[11].velocity = RL.cycle_responses[2][2];

  can_response[0].current = FR.cycle_responses[0][3];
  can_response[1].current = FR.cycle_responses[1][3];
  can_response[2].current = FR.cycle_responses[2][3];

  can_response[3].current = FL.cycle_responses[0][3];
  can_response[4].current = FL.cycle_responses[1][3];
  can_response[5].current = FL.cycle_responses[2][3];

  can_response[6].current = RR.cycle_responses[0][3];
  can_response[7].current = RR.cycle_responses[1][3];
  can_response[8].current = RR.cycle_responses[2][3];

  can_response[9].current = RL.cycle_responses[0][3];
  can_response[10].current = RL.cycle_responses[1][3];
  can_response[11].current = RL.cycle_responses[2][3];

  can_cmd = can_command;
  can_data = can_response;

}

void CAN::check_safety() {

    // if()
        // stop_can();
}

void CAN::stop_can() {

  FR.disable(m1);
  FR.disable(m2);
  FR.disable(m3);
  
  FL.disable(m4);
  FL.disable(m5);
  FL.disable(m6);
  
  RR.disable(m7);
  RR.disable(m8);
  RR.disable(m9);
  
  RL.disable(m10);
  RL.disable(m11);
  RL.disable(m12);

}


actuator_command_t* CAN::get_can_command() {
  return can_cmd;
}

actuator_response_t* CAN::get_can_data() {
  return can_data;
}