#include "IUST_Controller.hpp"

void IUST_Controller::initializeController() {
  debug_iter = 0;
}

void IUST_Controller::runController() {

  Mat3<float> kpMat;
  Mat3<float> kdMat;
  //kpMat << 20, 0, 0, 0, 20, 0, 0, 0, 20;
  //kdMat << 2.1, 0, 0, 0, 2.1, 0, 0, 0, 2.1;
  kpMat << userParameters.kp, 0, 0, 0,  userParameters.kp, 0, 0, 0,  userParameters.kp;
  kdMat <<  userParameters.kd, 0, 0, 0, userParameters.kd, 0, 0, 0, userParameters.kd;

  if (debug_iter % 1000 == 0) {
    printf("[INFO]  IUST controller is running!\n");
    printf("Iteration stamp:\t%d\n",(int)debug_iter);
    printf("--------------------_legController--------------------------\n");
    printf("ABAD Q = [%f, %f, %f, %f]\n", _legController->datas[0].q[0], _legController->datas[1].q[0], _legController->datas[2].q[0], _legController->datas[3].q[0]);
    printf("HIP  Q = [%f, %f, %f, %f]\n", _legController->datas[0].q[1], _legController->datas[1].q[1], _legController->datas[2].q[1], _legController->datas[3].q[1]);
    printf("KNEE Q = [%f, %f, %f, %f]\n", _legController->datas[0].q[2], _legController->datas[1].q[2], _legController->datas[2].q[2], _legController->datas[3].q[2]);
    printf("ABAD QD = [%f, %f, %f, %f]\n", _legController->datas[0].qd[0], _legController->datas[1].qd[0], _legController->datas[2].qd[0], _legController->datas[3].qd[0]);
    printf("HIP  QD = [%f, %f, %f, %f]\n", _legController->datas[0].qd[1], _legController->datas[1].qd[1], _legController->datas[2].qd[1], _legController->datas[3].qd[1]);
    printf("KNEE QD = [%f, %f, %f, %f]\n", _legController->datas[0].qd[2], _legController->datas[1].qd[2], _legController->datas[2].qd[2], _legController->datas[3].qd[2]);
    printf("-------------------COMMAND------------------------\n");
    // printf("ABAD DES Q = [%f, %f, %f, %f]\n", _spiCommand.q_des_abad[0],_spiCommand.q_des_abad[1],_spiCommand.q_des_abad[2],_spiCommand.q_des_abad[3]);
    // printf("HIP  DES Q = [%f, %f, %f, %f]\n", _spiCommand.q_des_hip[0],_spiCommand.q_des_hip[1],_spiCommand.q_des_hip[2],_spiCommand.q_des_hip[3]);
    // printf("KNEE DES Q = [%f, %f, %f, %f]\n", _spiCommand.q_des_knee[0],_spiCommand.q_des_knee[1],_spiCommand.q_des_knee[2],_spiCommand.q_des_knee[3]);
    // printf("KP A= [%f, %f, %f, %f]\n", _spiCommand.kp_abad[0],_spiCommand.kp_abad[1],_spiCommand.kp_abad[2],_spiCommand.kp_abad[3]);
    // printf("KP H= [%f, %f, %f, %f]\n", _spiCommand.kp_hip[0],_spiCommand.kp_hip[1],_spiCommand.kp_hip[2],_spiCommand.kp_hip[3]);
    // printf("KP K= [%f, %f, %f, %f]\n", _spiCommand.kp_knee[0],_spiCommand.kp_knee[1],_spiCommand.kp_knee[2],_spiCommand.kp_knee[3]);
    // printf("KD A= [%f, %f, %f, %f]\n", _spiCommand.kd_abad[0],_spiCommand.kd_abad[1],_spiCommand.kd_abad[2],_spiCommand.kd_abad[3]);
    // printf("KD H= [%f, %f, %f, %f]\n", _spiCommand.kd_hip[0],_spiCommand.kd_hip[1],_spiCommand.kd_hip[2],_spiCommand.kd_hip[3]);
    // printf("KD K= [%f, %f, %f, %f]\n", _spiCommand.kd_knee[0],_spiCommand.kd_knee[1],_spiCommand.kd_knee[2],_spiCommand.kd_knee[3]);
    // printf("ABAD DES T = [%f, %f, %f, %f]\n", _spiCommand.tau_abad_ff[0],_spiCommand.tau_abad_ff[1],_spiCommand.tau_abad_ff[2],_spiCommand.tau_abad_ff[3]);
    // printf("HIP  DES T = [%f, %f, %f, %f]\n", _spiCommand.tau_hip_ff[0],_spiCommand.tau_hip_ff[1],_spiCommand.tau_hip_ff[2],_spiCommand.tau_hip_ff[3]);
    // printf("KNEE DES T = [%f, %f, %f, %f]\n", _spiCommand.tau_knee_ff[0],_spiCommand.tau_knee_ff[1],_spiCommand.tau_knee_ff[2],_spiCommand.tau_knee_ff[3]);
    printf("--------------------------------------------------\n\n\n");
  }

  debug_iter++;

}
