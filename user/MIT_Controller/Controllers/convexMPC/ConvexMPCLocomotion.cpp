#include <iostream>
#include <Utilities/Timer.h>
#include <Utilities/Utilities_print.h>

#include "ConvexMPCLocomotion.h"
#include "convexMPC_interface.h"
#include <common/FootstepPlanner/GraphSearch.h>
#include <robot/include/RobotController.h>
#include "Gait.h"

#include <chrono>
#include <thread>
#include <fstream>
#include <cmath>
#include <vector>

#include <lcm/lcm-cpp.hpp>
#include "Network.hpp"
#include "PredictionMsg.hpp"
//#define DRAW_DEBUG_SWINGS
//#define DRAW_DEBUG_PATH
//RobotState robotst;

bool asc = false;
bool descending = false;
bool stair_climb = false;
//const double stair_edges[] = {0.3, 0.55, 0.8, 1.05, 1.3};
const double stair_edge_tolerance = 0.04;
const double adjustment = 0.003;

const int steps = 5;
const double delta = .05;
const double runstairs = .25;
const int cycletime = 8;
const double stair_start = 0.3;
const double bezierHeight = 0.1;
Vec4<double> footposnn;
static lcm::LCM network_h;
Network netwrok_data;


////////////////////
// Controller
////////////////////
std::string log_filename_v_des_x = "/home/aminghanbarzadeh/Cheetah-Software-Vision/config/NN/Thesis_output/v_des_x.txt";
std::string log_filename_v_des_z = "/home/aminghanbarzadeh/Cheetah-Software-Vision/config/NN/Thesis_output/v_des_z.txt";
std::string log_filename_position_x = "/home/aminghanbarzadeh/Cheetah-Software-Vision/config/NN/Thesis_output/position_x.txt";
std::string log_filename_position_z = "/home/aminghanbarzadeh/Cheetah-Software-Vision/config/NN/Thesis_output/position_z.txt";
std::string log_filename_rpy_pitch = "/home/aminghanbarzadeh/Cheetah-Software-Vision/config/NN/Thesis_output/rpy_pitch.txt";
std::string log_filename_f_ff = "/home/aminghanbarzadeh/Cheetah-Software-Vision/config/NN/Thesis_output/f_ff.txt";
std::string log_filename_Pf = "/home/aminghanbarzadeh/Cheetah-Software-Vision/config/NN/Thesis_output/Pf.txt";

//std::chrono::time_point<std::chrono::high_resolution_clock> start_time_global;

class Handler {
public:
    float predictions[4];

    void handleMessage(const lcm::ReceiveBuffer*, const std::string& chan, const PredictionMsg* msg) {
        // Verify if the message is received
        std::cout << "Message received on channel: " << chan << std::endl;

        // Update the predictions
        predictions[0] = msg->leg_1_pred;
        predictions[1] = msg->leg_2_pred;
        predictions[2] = msg->leg_3_pred;
        predictions[3] = msg->leg_4_pred;

        // Print predictions immediately after updating
        std::cout << "Updated predictions: " << predictions[0] << ", " << predictions[1] << ", "
                  << predictions[2] << ", " << predictions[3] << std::endl;
    }

};






// Function to get elapsed time since the start of the simulation
// long long time_diff_global() {
//     return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time_global).count();
// }

void log_velocity_x(const std::string& filename, float time, float v_des_x) {
    std::ofstream log_file(filename, std::ios::app); // Open in append mode
    if (log_file.is_open()) {
        log_file << time << "," << v_des_x << "\n";
        log_file.close();
    } else {
        std::cerr << "Unable to open log file: " << filename << "\n";
    }
}

void log_velocity_z(const std::string& filename, float time, float v_des_z) {
    std::ofstream log_file(filename, std::ios::app); // Open in append mode
    if (log_file.is_open()) {
        log_file << time << "," << v_des_z << "\n";
        log_file.close();
    } else {
        std::cerr << "Unable to open log file: " << filename << "\n";
    }
}

// Log seResult.position[0] to position_x.txt
void log_position_x(const std::string& filename, float time, float position_x) {
    std::ofstream log_file(filename, std::ios::app); // Open in append mode
    if (log_file.is_open()) {
        log_file << time << "," << position_x << "\n";
        log_file.close();
    } else {
        std::cerr << "Unable to open log file: " << filename << "\n";
    }
}

// Log seResult.position[2] to position_z.txt
void log_position_z(const std::string& filename, float time, float position_z) {
    std::ofstream log_file(filename, std::ios::app); // Open in append mode
    if (log_file.is_open()) {
        log_file << time << "," << position_z << "\n";
        log_file.close();
    } else {
        std::cerr << "Unable to open log file: " << filename << "\n";
    }
}

// Log seResult.rpy[1] (pitch) to rpy_pitch.txt
void log_rpy_pitch(const std::string& filename, float time, float rpy_pitch) {
    std::ofstream log_file(filename, std::ios::app); // Open in append mode
    if (log_file.is_open()) {
        log_file << time << "," << rpy_pitch << "\n";
        log_file.close();
    } else {
        std::cerr << "Unable to open log file: " << filename << "\n";
    }
}

void log_f_ff(const std::string& filename, float time, const Vec3<float> f_ff[4]) {
    std::ofstream log_file(filename, std::ios::app);  // Open in append mode
    if (log_file.is_open()) {
        // Write time
        log_file << time << ",";
        
        // Log f_ff for each leg
        for (int i = 0; i < 4; ++i) {
            log_file << f_ff[i].transpose();  // Transpose the Vec3 to write in row format (x, y, z)
            if (i < 3) log_file << ",";       // Separate the legs with commas
        }
        log_file << "\n";  // New line after all legs' data
        log_file.close();
    } else {
        std::cerr << "Unable to open log file: " << filename << "\n";
    }
}

void log_Pf(const std::string& filename, float time, const Vec3<float> Pff) {
    std::ofstream log_file(filename, std::ios::app); // Open in append mode
    if (log_file.is_open()) {
        log_file << time << "," << Pff[0] << "," << Pff[2] << "\n"; // Log time, Pf[0], and Pf[2]
        log_file.close();
    } else {
        std::cerr << "Unable to open log file: " << filename << "\n";
    }
}



void initialize_log(const std::string& filename) {
    std::ofstream log_file(filename, std::ios::out); // Open in write mode
    if (log_file.is_open()) {
        log_file << "horizonLength,cycletime,bezierHeight,delta,runstairs,pitch_ascension,offset,Pf,des_vel\n";
        log_file.close();
    } else {
        std::cerr << "Unable to initialize log file: " << filename << "\n";
    }
}


void log_parameters(const std::string& filename, int horizonLength_, int cycletime_, double bezierHeight_, double delta_, int runstairs_, float pitch_ascension_, float offsetY, const std::vector<Vec3<float>>& allPf, const Vec3<float>& des_vel, const Vec3<float>& position_desired) {
    std::ofstream log_file(filename, std::ios::app); // Open in append mode
    if (log_file.is_open()) {
        log_file << horizonLength_ << ","
                 << cycletime_ << ","
                 << bezierHeight_ << ","
                 << delta_ << ","
                 << runstairs_ << ","
                 << pitch_ascension_ << ","
                 << offsetY << ",";

        // Log all foot positions
        for (const auto& Pf : allPf) {
            log_file << Pf.transpose() << ",";
        }

        // Log desired velocity
        log_file << des_vel.transpose() << ",";
        log_file << position_desired.transpose() << "\n";

        log_file.close();
    } else {
        std::cerr << "Unable to open log file: " << filename << "\n";
    }
}




std::string log_filename = "/home/aminghanbarzadeh/Cheetah-Software-Vision/config/NN/1_.1_5_-.4311.txt"; // Update the path as needed

std::vector<double> generate_stair_edges(int count, double start, double step) {
  std::vector<double> stair_edges;
  for (int i = 0; i < count; ++i) {
    stair_edges.push_back(start + i * step);
  }
  return stair_edges;
}

void determine_z_position(float x, float& z) {
  if (x < stair_start) {
    z = -adjustment;
  } else {
    for (int step = 0; step < steps; ++step) {
      double step_start = stair_start + step * runstairs;
      double step_end = step_start + runstairs;
      if (x >= step_start && x < step_end) {
        z = (1+step) * delta - adjustment;
        break;
      }
    }
  }
}

ConvexMPCLocomotion::ConvexMPCLocomotion(float _dt, int _iterations_between_mpc, MIT_UserParameters* parameters,  float fmax, RobotType &robotType) :
  iterationsBetweenMPC(_iterations_between_mpc),
  horizonLength(2*cycletime),
  dt(_dt),
  trotting(horizonLength, Vec4<int>(0,cycletime,cycletime,0), Vec4<int>(cycletime,cycletime,cycletime,cycletime),"Trotting"),
  bounding(horizonLength, Vec4<int>(5,5,0,0),Vec4<int>(4,4,4,4),"Bounding"),
  //bounding(horizonLength, Vec4<int>(5,5,0,0),Vec4<int>(3,3,3,3),"Bounding"),
  pronking(horizonLength, Vec4<int>(0,0,0,0),Vec4<int>(4,4,4,4),"Pronking"),
  jumping(horizonLength, Vec4<int>(0,0,0,0), Vec4<int>(2,2,2,2), "Jumping"),
  //galloping(horizonLength, Vec4<int>(0,2,7,9),Vec4<int>(6,6,6,6),"Galloping"),
  //galloping(horizonLength, Vec4<int>(0,2,7,9),Vec4<int>(3,3,3,3),"Galloping"),
  galloping(horizonLength, Vec4<int>(0,2,7,9),Vec4<int>(4,4,4,4),"Galloping"),
  standing(horizonLength, Vec4<int>(0,0,0,0),Vec4<int>(10,10,10,10),"Standing"),
  //trotRunning(horizonLength, Vec4<int>(0,5,5,0),Vec4<int>(3,3,3,3),"Trot Running"),
  trotRunning(horizonLength, Vec4<int>(0,5,5,0),Vec4<int>(4,4,4,4),"Trot Running"),
  walking(horizonLength, Vec4<int>(0,3,5,8), Vec4<int>(5,5,5,5), "Walking"),
  walking2(horizonLength, Vec4<int>(0,5,5,0), Vec4<int>(7,7,7,7), "Walking2"),
  pacing(horizonLength, Vec4<int>(5,0,5,0),Vec4<int>(5,5,5,5),"Pacing"),
  random(horizonLength, Vec4<int>(9,13,13,9), 0.4, "Flying nine thirteenths trot"),
//  random(horizonLength, Vec4<int>(0,3,6,9), 0.5, "Flying  trot"),
  random2(horizonLength, Vec4<int>(8,16,16,8), 0.5, "Double Trot"),
  stair_edges(generate_stair_edges(steps, stair_start, runstairs)) // Generate stair edges
{
  _parameters = parameters;
  dtMPC = dt * iterationsBetweenMPC;
  default_iterations_between_mpc = iterationsBetweenMPC;
  printf("[Convex MPC] dt: %.3f iterations: %d, dtMPC: %.3f\n", dt, iterationsBetweenMPC, dtMPC);
  setup_problem(dtMPC, horizonLength, 0.4, fmax);
  //setup_problem(dtMPC, horizonLength, 0.4, 650); // DH
  rpy_comp[0] = 0;
  rpy_comp[1] = 0;
  rpy_comp[2] = 0;
  rpy_int[0] = 0;
  rpy_int[1] = 0;
  rpy_int[2] = 0;

  for(int i = 0; i < 4; i++)
    firstSwing[i] = true;
  if (robotType == RobotType::IUST){
    initMilabSparseMPC();
  }else{
    initSparseMPC();
  }

  pBody_des.setZero();
  vBody_des.setZero();
  aBody_des.setZero();

   //initialize_log(log_filename);
}

void ConvexMPCLocomotion::initialize(){
  for(int i = 0; i < 4; i++) firstSwing[i] = true;
  firstRun = true;
}

void ConvexMPCLocomotion::recompute_timing(int iterations_per_mpc) {
  iterationsBetweenMPC = iterations_per_mpc;
  dtMPC = dt * iterations_per_mpc;
}

// auto current_time() {
//   return std::chrono::high_resolution_clock::now();
  
// }
// auto time_diff(const std::chrono::time_point<std::chrono::high_resolution_clock>& start) {
//   return std::chrono::duration_cast<std::chrono::milliseconds>(current_time() - start).count();
//   //std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(current_time() - start).count() << std::endl;
//   //std::cout << std::chrono::high_resolution_clock::now() << std::endl;
// }
// auto start_time = current_time();

void ConvexMPCLocomotion::_SetupCommand(ControlFSMData<float> & data){
    // if(data._quadruped->_robotType == RobotType::IUST) {
    //     _body_height = 0.375;
    // }else if(data._quadruped->_robotType == RobotType::MINI_CHEETAH){
    //     _body_height = 0.29;
    // }else if(data._quadruped->_robotType == RobotType::CHEETAH_3){
    //     _body_height = 0.45;
    // }else{
    //     assert(false);
    // }

  float x_vel_cmd=0, y_vel_cmd=0, z_vel_cmd = 0;
  float filter(0.1);

  // if(data.controlParameters->use_rc){
  //   const rc_control_settings* rc_cmd = data._desiredStateCommand->rcCommand;
  //   data.userParameters->cmpc_gait = rc_cmd->variable[0];
  //   _yaw_turn_rate = -rc_cmd->omega_des[2];
  //   x_vel_cmd = rc_cmd->v_des[0];
  //   y_vel_cmd = rc_cmd->v_des[1] * 0.5;
  //   z_vel_cmd = rc_cmd->v_des[2];
  //   _body_height += rc_cmd->height_variation * 0.08;
  // }else{
  //   _yaw_turn_rate = data._desiredStateCommand->rightAnalogStick[0]*0.2;
  //   x_vel_cmd = data._desiredStateCommand->leftAnalogStick[1]*0.2;
  //   y_vel_cmd = data._desiredStateCommand->leftAnalogStick[0]*0.2;
    //z_vel_cmd = data._desiredStateCommand->rightAnalogStick[0]*0.2;
    // std::thread timer_thread([&]() {
    //     std::this_thread::sleep_for(std::chrono::seconds(5));
    //     data._desiredStateCommand->ascending_trigger = true;
    //     std::cout << "Parameter set to true after 5 seconds." << std::endl;
    // });
    
    //std::cout <<  << std::endl;
    //if (time_diff(start_time) >= 11000) {
       //data._desiredStateCommand->ascending_trigger = true;
       //std::cout << "Parameter set to true after 5 seconds." << std::endl;
    // //         // Reset the start time to avoid repeatedly setting the parameter
    //   //start_time = current_time();
    //}
    if(data._desiredStateCommand->ascending_trigger || asc){
      
      //z_vel_cmd = x_vel_cmd * 0.364;
      asc = true;
      _pitch_des = pitch_ascension;
      x_vel_cmd = 0.2;

    }
    else if (data._desiredStateCommand->descending_trigger || descending)
    {
      //z_vel_cmd = x_vel_cmd * -0.364;
      descending = true;
      _pitch_des = pitch_descension;
    }
    else if(data._desiredStateCommand->stair_trigger || stair_climb)
    {
      stair_climb = true;
      //x_vel_cmd = 0.1;
    }
    
    else
    {
      _pitch_des = -0.0;
      
    }
    
    if(data._desiredStateCommand->cancel_trigger){
      asc = false;
      descending = false;
      stair_climb = false;
    }
 
    //std::cout << "se++++++++++++++++++++++++++++ " << data._desiredStateCommand->cancel_trigger << std::endl;
  //}
  _x_vel_des = _x_vel_des*(1-filter) + x_vel_cmd*filter;
  _y_vel_des = _y_vel_des*(1-filter) + y_vel_cmd*filter;
  _z_vel_des = _z_vel_des*(1-filter) + z_vel_cmd*filter;
  _yaw_des = data._stateEstimator->getResult().rpy[2] + dt * _yaw_turn_rate;  //TODO
  _roll_des = 0.0;

}
//std::cout << "se++++++++++++++++++++++++++++ " << robotst.ascention_mode << std::endl;
template<>
void ConvexMPCLocomotion::run(ControlFSMData<float>& data) {
  bool omniMode = false;
  lcm::LCM lc;
  if(!lc.good()) {
    std::cerr << "LCM initialization failed!" << std::endl;
    return;
  }

  Handler handler;
  lc.subscribe("prediction_channel", &Handler::handleMessage, &handler);
  lc.handleTimeout(5);
  for (int i = 0; i < 4; i++)
  {
    
    if (handler.predictions[i]!= 0)
    {
      footposnn[i] = handler.predictions[i];
    }
    
  }
  
  // Command Setup
  _SetupCommand(data);
  gaitNumber = data.userParameters->cmpc_gait;
  if(gaitNumber >= 9) {
    gaitNumber -= 9;
    omniMode = true;
  }

  auto& seResult = data._stateEstimator->getResult();

  if (seResult.position[0] > (runstairs*steps) + stair_start + 0.1) //stops the controller(NN Purposes)
  {
    std::ofstream signal_file("/tmp/robot_position_reached");
    signal_file << "done";
    signal_file.close();
    //break; // Exit the loop to stop the controller
  }
  

  // Check if transition to standing
  if(((gaitNumber == 4) && current_gait != 4) || firstRun)
  {
    //std::cout << "se++++++++++++++++++++++++++++ " << seResult.position[2] << std::endl;
    stand_traj[0] = seResult.position[0];
    stand_traj[1] = seResult.position[1];
    stand_traj[2] = seResult.position[2];
    stand_traj[3] = 0;
    stand_traj[4] = 0;
    stand_traj[5] = seResult.rpy[2];
    world_position_desired[0] = stand_traj[0];
    world_position_desired[1] = stand_traj[1];
    world_position_desired[2] = stand_traj[2];
  }
  //std::cout << "bh############################ " << seResult.position[0] << std::endl;
  // pick gait：default gait==9
  Gait* gait = &trotting;
  if(gaitNumber == 1)
    gait = &bounding;
  else if(gaitNumber == 2)
    gait = &pronking;
  else if(gaitNumber == 3)
    gait = &galloping;
  else if(gaitNumber == 4)
    gait = &standing;
  else if(gaitNumber == 5)
    gait = &trotRunning;
  else if(gaitNumber == 6)
    gait = &walking;
  else if(gaitNumber == 7)
    gait = &walking2;
  else if(gaitNumber == 8)
    gait = &pacing;
  current_gait = gaitNumber;

  gait->setIterations(iterationsBetweenMPC, iterationCounter);
  jumping.setIterations(iterationsBetweenMPC, iterationCounter);


  jumping.setIterations(27/2, iterationCounter);

  //printf("[%d] [%d]\n", jumping.get_current_gait_phase(), gait->get_current_gait_phase());
  // check jump trigger
  jump_state.trigger_pressed(jump_state.should_jump(jumping.getCurrentGaitPhase()),
      data._desiredStateCommand->trigger_pressed);


  // bool too_high = seResult.position[2] > 0.29;
  // check jump action
  if(jump_state.should_jump(jumping.getCurrentGaitPhase())) {
    gait = &jumping;
    recompute_timing(27/2);
    _body_height = _body_height_jumping;
    currently_jumping = true;

  } else {
    recompute_timing(default_iterations_between_mpc);
    currently_jumping = false;
  }
  
    // if (data._quadruped->_robotType == RobotType::IUST) {
    //     _body_height = 0.35;
    // } else if (data._quadruped->_robotType == RobotType::MINI_CHEETAH) {
    //     _body_height = 0.29;
    // } else if (data._quadruped->_robotType == RobotType::CHEETAH_3) {
    //     _body_height = 0.45;
    // }
  // integrate position setpoint
  Vec3<float> v_des_robot(_x_vel_des, _y_vel_des, _z_vel_des);
  v_des_robot = seResult.rBody.transpose()*v_des_robot;
  Vec3<float> v_des_world = 
    omniMode ? v_des_robot : seResult.rBody.transpose() * v_des_robot;
  //std::cout << "rbody+!@+#$%+^+%+#&*%(%*&^%$): " << v_des_world << std::endl;

  Vec3<float> v_robot = seResult.vWorld;
  //std::cout << "rbody+!@+#$%+^+%+#&*%(%*&^%$): " << seResult.rBody << std::endl;
  //pretty_print(v_des_world, std::cout, "v des world");




  // Integral-esque pitche and roll compensation
  if(fabs(v_robot[0]) > .2)   //avoid dividing by zero
  {
    rpy_int[1] += dt*(_pitch_des - seResult.rpy[1])/v_robot[0];
  }
  if(fabs(v_robot[1]) > 0.1)
  {
    rpy_int[0] += dt*(_roll_des - seResult.rpy[0])/v_robot[1];
    //std::cout << "rbody+!@+#$%+^+%+#&*%(%*&^%$): " << rpy_int[0] << std::endl;
  }

  rpy_int[0] = fminf(fmaxf(rpy_int[0], -.25), .25); //TODO increase 0.25
  rpy_int[1] = fminf(fmaxf(rpy_int[1], -.25), .25); //IUST changed
  rpy_comp[1] = v_robot[0] * rpy_int[1];//- 0.4; //IUST changed working!!!! 
  rpy_comp[0] = v_robot[1] * rpy_int[0] * (gaitNumber!=8);  //turn off for pronking
  //std::cout << "rbody+!@+#$%+^+%+#&*%(%*&^%$): " << seResult.position[2] << std::endl;

  for(int i = 0; i < 4; i++) {
    pFoot[i] = seResult.position + 
      seResult.rBody.transpose() * (data._quadruped->getHipLocation(i) + 
          data._legController->datas[i].p);
    //std::cout << "pfoot_z: " << data._legController->datas[i].p[2] << std::endl;
  }

  //_body_height = 0.26 + (pFoot[0][2]+pFoot[2][2])/2; //IUST

  if(gait != &standing) {
    world_position_desired += dt * Vec3<float>(v_des_world[0], v_des_world[1], v_des_world[2]);
  }



  // static bool is_start_time_initialized = false; // Ensure it only happens once
  // if (!is_start_time_initialized) {
  //   //start_time_global = std::chrono::high_resolution_clock::now();
  //   is_start_time_initialized = true; // Prevent resetting in the loop
  // }

  // float current_time = static_cast<float>(time_diff_global()) / 1000.0; 
  // log_velocity_x(log_filename_v_des_x, current_time, v_des_world[0]);
  // log_velocity_z(log_filename_v_des_z, current_time, v_des_world[2]);
  // log_position_x(log_filename_position_x, current_time, seResult.position[0]);
  // log_position_z(log_filename_position_z, current_time, seResult.position[2]);
  // log_rpy_pitch(log_filename_rpy_pitch, current_time, seResult.rpy[1]);
  // log_f_ff(log_filename_f_ff, current_time, f_ff);
  
  // some first time initialization
  if(firstRun)
  {
    world_position_desired[0] = seResult.position[0];
    world_position_desired[1] = seResult.position[1];
    world_position_desired[2] = seResult.position[2];
    //world_position_desired[2] = seResult.rpy[2];

    for(int i = 0; i < 4; i++)
    {

      footSwingTrajectories[i].setHeight(0.05);
      footSwingTrajectories[i].setInitialPosition(pFoot[i]);
      footSwingTrajectories[i].setFinalPosition(pFoot[i]);

    }
    firstRun = false;
  }
  // if(seResult.position[2] < 0.25) {
  //   world_position_desired[2] = 0.25;
  // }  
  // foot placement

  for(int l = 0; l < 4; l++)
    swingTimes[l] = gait->getCurrentSwingTime(dtMPC, l);

  float side_sign[4] = {-1, 1, -1, 1};
  float interleave_y[4] = {-0.08, 0.08, 0.02, -0.02};
  //float interleave_gain = -0.13;
  float interleave_gain = -0.2;
  //float v_abs = std::fabs(seResult.vBody[0]);
  float v_abs = std::fabs(v_des_robot[0]);
  //std::cout << "************************: " << seResult.rpy[1] << std::endl;
  Vec4<float> contactStates= gait->getContactState();
  Vec4<float> swingStates = gait->getSwingState();
  //locomo += 0.01;

  std::vector<Vec3<float>> allPf(4); // Vector to store all foot positions
  float offsetY = 0; // Variable to store the second component of the offset
  Vec3<float> des_vel0; // Variable to store desired velocity

  for(int i = 0; i < 4; i++)
  {
    if(firstSwing[i]) {
      swingTimeRemaining[i] = swingTimes[i];
    } else {
      swingTimeRemaining[i] -= dt;
    }
    //if(firstSwing[i]) {
    //footSwingTrajectories[i].setHeight(.05);
    footSwingTrajectories[i].setHeight(bezierHeight);
    Vec3<float> offset;
    if (data._quadruped->_robotType == RobotType::IUST){
        offset << 0, side_sign[i] * 0.125, 0;
    } else{
        offset << 0, side_sign[i] * .105, 0;
    }

    Vec3<float> pRobotFrame = (data._quadruped->getHipLocation(i) + offset);
    pRobotFrame[1] += interleave_y[i] * v_abs * interleave_gain;
    
    
    float stance_time = gait->getCurrentStanceTime(dtMPC, i);
    Vec3<float> pYawCorrected =
      coordinateRotation(CoordinateAxis::Z, -_yaw_turn_rate* stance_time / 2) * pRobotFrame;
    //std::cout << "zramp**************************: " << seResult.position[2]-(seResult.position[0]-0.6)*0.364 << std::endl;
    
// body desired velocity
    Vec3<float> des_vel;
    des_vel[0] = _x_vel_des;
    des_vel[1] = _y_vel_des;
    des_vel[2] = _z_vel_des;
    des_vel = seResult.rBody.transpose() * des_vel;
//Vec3<float> k;
//k[0]=0.1,k[1]=0.0,k[2]=0.0;
    //std::cout << "************************: " << seResult.position[2] << std::endl;
    Vec3<float> Pf = seResult.position + seResult.rBody.transpose() * (pYawCorrected
          + des_vel * swingTimeRemaining[i]);
    
    if (descending)
    {
      Pf[0] -= 0.1;
    }
  std::chrono::time_point<std::chrono::steady_clock> timesteps;
  //LCM publish data
  netwrok_data.cycletime = cycletime;
  netwrok_data.bezierheight = bezierHeight;
  netwrok_data.delta = delta;
  netwrok_data.runstairs = runstairs;
  netwrok_data.pitch_ascension = pitch_ascension;
  netwrok_data.offsety = 0.105;
  netwrok_data.xvelocity = des_vel[0];
  netwrok_data.zvelocity = des_vel[2];

  if (des_vel[0] != 0)
  {
    timesteps = std::chrono::steady_clock::now();
    auto now_in_seconds = std::chrono::duration_cast<std::chrono::seconds>(timesteps.time_since_epoch()).count();

    netwrok_data.timestep = now_in_seconds/((stair_start+0.1+(runstairs*steps))/des_vel[0]);
  }
  
  network_h.publish("inputdatas" , &netwrok_data);  

//      Vec3<float> Pf = seResult.position + seResult.rBody.transpose() * (pFoot[i]
//    + seResult.vWorld * swingTimeRemaining[i];
    //std::cout << "se************************: " << seResult.rBody.transpose() << std::endl;
    //std::cout << "yaw************************: " << pRobotFrame << std::endl;
    //float p_rel_max = 0.35f;
    float p_rel_max = 0.3f;

    // Using the estimated velocity is correct
    //Vec3<float> des_vel_world = seResult.rBody.transpose() * des_vel;
    float pfx_rel = seResult.vWorld[0] * (.5 + _parameters->cmpc_bonus_swing) * stance_time +
      .03f*(seResult.vWorld[0]-v_des_world[0]) +
      (0.5f*seResult.position[2]/9.81f) * (seResult.vWorld[1]*_yaw_turn_rate);
    //std::cout << "************************: " << pfx_rel << std::endl;
    float pfy_rel = seResult.vWorld[1] * .5 * stance_time * dtMPC +
      .03f*(seResult.vWorld[1]-v_des_world[1]) +
      (0.5f*seResult.position[2]/9.81f) * (-seResult.vWorld[0]*_yaw_turn_rate);
    pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
    pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);

    Pf[0] +=  pfx_rel;
    Pf[1] +=  pfy_rel;
    Pf[2] = footposnn[i];
    std::cout << "stance********************: " << footposnn[i]<< std::endl;
    //Pf[2] += pfz_rel;
    // if (contactStates[i] != 0)
    // {
    //   Pf[2] += data._legController->datas[i].p[2];
    //   //std::cout << "stance********************: " << contactStates[i]<< std::endl;
    // }
    // else if (swingStates[i] < 0.5)
    // {
    //   Pf[2] += (data._legController->datas[i].p[2] - (bezierHeight * swingStates[i] * 2));
    //   //std::cout << "liftoff*******************: " << contactStates[i]<< std::endl;
    // }
    // else
    // {
    //   Pf[2] += (data._legController->datas[i].p[2] - (bezierHeight-(bezierHeight * (swingStates[i]-0.5) * 2)));
    //   //std::cout << "comedown*****************: " << contactStates[i]<< std::endl;
    // }
    //Pf[2] += (data._legController->datas[i].p[2] - 0.003); //-0.0005 + pFoot[i][2];//+ (seResult.position[2] - 0.28);
    // if (i == 0)
    // {
    //   std::cout << "contactstates:  "<< -0.0005 + pFoot[i][2] << "  ground:  " << Pf[2] << std::endl;
    // }

    //Pf[2] += (data._legController->datas[i].p[2] - 0.005); //-0.0005 + pFoot[i][2];//+ (seResult.position[2] - 0.28);
    // if (i == 0)
    // {
    //   std::cout << "contactstates:  "<< -0.0005 + pFoot[i][2] << "  ground:  " << Pf[2] << std::endl;
    // }

    // if(asc)
    // {

    //   for (int inumber = 0; inumber < 5; inumber++)
    //   {
    //     if ((Pf[0] - stair_edges[inumber]) < stair_edge_tolerance && (Pf[0] - stair_edges[inumber]) > 0)
    //     {
    //       Pf[0] += stair_edge_tolerance;
    //     }
    //     else if ((Pf[0] - stair_edges[inumber]) > -stair_edge_tolerance && (Pf[0] - stair_edges[inumber]) < 0)
    //     {
    //       Pf[0] -= stair_edge_tolerance;
    //     }
    //   }
    // }

    // if (Pf[0] < 0.3)
    // {
    //   Pf[2] = -adjustment;
      
    // }else  if (0.3 < Pf[0] && Pf[0]< 0.55)
    // {
    //   Pf[2] = delta- adjustment;
      // if (Pf[0] < 0.425)
      // {
      //   Pf[0] = 0.3625;
      // }else
      // {
      //   Pf[0] = 0.4875;
      // }
      
      // if (i==0)
      // {
      //   pitch_ascension = -0.6;
      // }
      
    // }else if (0.55 < Pf[0] && Pf[0] < 0.8)
    // {
    //   Pf[2] = 2*delta - adjustment;
      // if (Pf[0] < 0.675)
      // {
      //   Pf[0] = 0.6125;
      // }else
      // {
      //   Pf[0] = 0.7375;
      // }
      
    //if (i==0)
    //{
    //   pitch_ascension = -0.6;
    //   // }
      
    // }else if (0.8 < Pf[0] && Pf[0]< 1.05)
    // {
    //   Pf[2] = 3*delta - adjustment;
      // if (Pf[0] < 0.925)
      // {
      //   Pf[0] = 0.8625;
      // }else{
        
      //   Pf[0] = 0.9875;
      // }
    //   // if (i==0)
    //   // {
    //   //   pitch_ascension = -0.6;
    //   // }
      
    // }else if (1.05 < Pf[0] && Pf[0]< 1.3)
    // {
    //   Pf[2] = 4*delta - adjustment;
      // if (Pf[0] < 1.175)
      // {
      //   Pf[0] = 1.1125;
      // }else
      // {
      //   Pf[0] = 1.2375;
      // }
    //   //  if (i==0)
    //   //  {
    //   //     pitch_ascension = -0.6;
    //   //  }
      
    // }else if (1.3 < Pf[0] && Pf[0] < 1.55)
    // {
    //   Pf[2] = 5*delta- adjustment;

    // Determine the step based on the foot x position
    // determine_z_position(Pf[0], Pf[2]);

    // if (Pf[0] > 0.2 + stair_start+(steps*runstairs))
    // {
    //   Pf[2] = steps*delta - adjustment;
    //   pitch_ascension = 0;
    //   _x_vel_des = 0.05;

    // }

      // if (Pf[0] < 1.425)
      // {
      //   Pf[0] = 1.3625;
      // }else
      // {
      //   Pf[0] = 1.4875;
      // }
    //   // if (i==0)
    //   // {
    //   //   pitch_ascension = -0.6;
    //   // }
    //}else if (Pf[0] > 1.55)
    //{
      //Pf[2] = 5*delta - adjustment;
    //   //pitch_ascension = -0.55;

    // }else if (Pf[0]>2.35 && Pf[0] < 3.703)
    // {
    //   Pf[2] = 0.36397*(1.353-Pf[0]+2.35);
    // }           
    
    // if (asc)
    // {
    //   for (int inumber = 0; inumber < steps; inumber++)
    //   {
        
    //     if ((Pf[0] - stair_edges[inumber]) < stair_edge_tolerance && (Pf[0] - stair_edges[inumber]) > 0)
    //     {  
    //       Pf[0] += stair_edge_tolerance;
    //     }
    //     else if ((Pf[0] - stair_edges[inumber]) > -stair_edge_tolerance && (Pf[0] - stair_edges[inumber]) < 0)
    //     { 
    //       Pf[0] -= stair_edge_tolerance;
    //     }
    //   }
    // }

    
    //std::cout << "pffff:  "<< Pf[2] << std::endl;
    //std::cout << "pfffx:  "<< Pf[0] << std::endl;
    //std::cout << "pfootx:  "<< pFoot[i] << std::endl;
    std::cout << "z pos:  "<< Pf[2] << std::endl;




    footSwingTrajectories[i].setFinalPosition(Pf);
    if (i == 0 && contactStates[i] != 0)
    {
      //log_Pf(log_filename_Pf, current_time, Pf);
    }
    allPf[i] = Pf; // Store the foot position
    offsetY = std::abs(offset[1]); // Store the absolute value of the second component of the offset
    des_vel0 = des_vel; // Store the desired velocity

//      footSwingTrajectories[i].setHeight(.01);

    
  }
  float bezierHeight1 = bezierHeight;
  int horizonLength1 = horizonLength;
  int cycletime1 = cycletime;
  double delta1 = delta;
  int runstairs1 = runstairs;
  float pitch_ascension0 = pitch_ascension;
  log_parameters(log_filename, horizonLength1, cycletime1, bezierHeight1, delta1, runstairs1, pitch_ascension0, offsetY, allPf, des_vel0, world_position_desired);
  
  // calc gait
  iterationCounter++;

  // load LCM leg swing gains
  Kp << 700, 0, 0,
     0, 700, 0,
     0, 0, 150;
  Kp_stance = Kp;


  Kd << 7, 0, 0,
     0, 7, 0,
     0, 0, 7;
  Kd_stance = Kd;
  // gait
  //Vec4<float> contactStates = gait->getContactState();
  //Vec4<float> swingStates = gait->getSwingState();
  int* mpcTable = gait->getMpcTable();
  updateMPCIfNeeded(mpcTable, data, omniMode);

  //  StateEstimator* se = hw_i->state_estimator;
  Vec4<float> se_contactState(0,0,0,0);

#ifdef DRAW_DEBUG_PATH
  auto* trajectoryDebug = data.visualizationData->addPath();
  if(trajectoryDebug) {
    trajectoryDebug->num_points = 10;
    trajectoryDebug->color = {0.2, 0.2, 0.7, 0.5};
    for(int i = 0; i < 10; i++) {
      trajectoryDebug->position[i][0] = trajAll[12*i + 3];
      trajectoryDebug->position[i][1] = trajAll[12*i + 4];
      trajectoryDebug->position[i][2] = trajAll[12*i + 5];
      auto* ball = data.visualizationData->addSphere();
      ball->radius = 0.01;
      ball->position = trajectoryDebug->position[i];
      ball->color = {1.0, 0.2, 0.2, 0.5};
    }
  }
#endif

  for(int foot = 0; foot < 4; foot++)
  {
    float contactState = contactStates[foot];
    float swingState = swingStates[foot];
    if(swingState > 0) // foot is in swing
    {
      if(firstSwing[foot])
      {
        firstSwing[foot] = false;
        footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
      }

#ifdef DRAW_DEBUG_SWINGS
      auto* debugPath = data.visualizationData->addPath();
      if(debugPath) {
        debugPath->num_points = 100;
        debugPath->color = {0.2,1,0.2,0.5};
        float step = (1.f - swingState) / 100.f;
        for(int i = 0; i < 100; i++) {
          footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState + i * step, swingTimes[foot]);
          debugPath->position[i] = footSwingTrajectories[foot].getPosition();
        }
      }
      auto* finalSphere = data.visualizationData->addSphere();
      if(finalSphere) {
        finalSphere->position = footSwingTrajectories[foot].getPosition();
        finalSphere->radius = 0.02;
        finalSphere->color = {0.6, 0.6, 0.2, 0.7};
      }
      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);
      auto* actualSphere = data.visualizationData->addSphere();
      auto* goalSphere = data.visualizationData->addSphere();
      goalSphere->position = footSwingTrajectories[foot].getPosition();
      actualSphere->position = pFoot[foot];
      goalSphere->radius = 0.02;
      actualSphere->radius = 0.02;
      goalSphere->color = {0.2, 1, 0.2, 0.7};
      actualSphere->color = {0.8, 0.2, 0.2, 0.7};
#endif
      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);


      //      footSwingTrajectories[foot]->updateFF(hw_i->leg_controller->leg_datas[foot].q,
      //                                          hw_i->leg_controller->leg_datas[foot].qd, 0); // velocity dependent friction compensation todo removed
      //hw_i->leg_controller->leg_datas[foot].qd, fsm->main_control_settings.variable[2]);

      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
      //足端相对于hip的位置
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) 
        - data._quadruped->getHipLocation(foot);
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);

      // Update for WBC
      pFoot_des[foot] = pDesFootWorld;
      vFoot_des[foot] = vDesFootWorld;
      aFoot_des[foot] = footSwingTrajectories[foot].getAcceleration();
      
      if(!data.userParameters->use_wbc){
        // Update leg control command regardless of the usage of WBIC
        data._legController->commands[foot].pDes = pDesLeg;
        data._legController->commands[foot].vDes = vDesLeg;
        data._legController->commands[foot].kpCartesian = Kp;
        data._legController->commands[foot].kdCartesian = Kd;
      }
    }
    else // foot is in stance
    {
      firstSwing[foot] = true;

#ifdef DRAW_DEBUG_SWINGS
      auto* actualSphere = data.visualizationData->addSphere();
      actualSphere->position = pFoot[foot];
      actualSphere->radius = 0.02;
      actualSphere->color = {0.2, 0.2, 0.8, 0.7};
#endif

      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
      
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data._quadruped->getHipLocation(foot);
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
      //cout << "Foot " << foot << " relative velocity desired: " << vDesLeg.transpose() << "\n";

      if(!data.userParameters->use_wbc){
        data._legController->commands[foot].pDes = pDesLeg;
        data._legController->commands[foot].vDes = vDesLeg;
        data._legController->commands[foot].kpCartesian = Kp_stance;
        data._legController->commands[foot].kdCartesian = Kd_stance;

        data._legController->commands[foot].forceFeedForward = f_ff[foot];
        data._legController->commands[foot].kdJoint = Mat3<float>::Identity() * 0.2;

        //      footSwingTrajectories[foot]->updateFF(hw_i->leg_controller->leg_datas[foot].q,
        //                                          hw_i->leg_controller->leg_datas[foot].qd, 0); todo removed
        // hw_i->leg_controller->leg_commands[foot].tau_ff += 0*footSwingController[foot]->getTauFF();
      }else{ // Stance foot damping
        data._legController->commands[foot].pDes = pDesLeg;
        data._legController->commands[foot].vDes = vDesLeg;
        data._legController->commands[foot].kpCartesian = 0.*Kp_stance;
        data._legController->commands[foot].kdCartesian = Kd_stance;
      }
      //            cout << "Foot " << foot << " force: " << f_ff[foot].transpose() << "\n";
      se_contactState[foot] = contactState;

      // Update for WBC
      //Fr_des[foot] = -f_ff[foot];
    }
  }

  // se->set_contact_state(se_contactState); todo removed
  data._stateEstimator->setContactPhase(se_contactState);

  // Update For WBC
  pBody_des[0] = world_position_desired[0];
  pBody_des[1] = world_position_desired[1];
  pBody_des[2] = world_position_desired[2];
  //std::cout << "************************: " << world_position_desired[2] << std::endl;
  vBody_des[0] = v_des_world[0];
  vBody_des[1] = v_des_world[1];
  vBody_des[2] = v_des_world[2];

  aBody_des.setZero();

  pBody_RPY_des[0] = 0.;
  if (asc)
  {
    pBody_RPY_des[1] = pitch_ascension;
  }else if (descending)
  {
    pBody_RPY_des[1] = pitch_descension;
  }else{
    pBody_RPY_des[1] = -0.0;
  }
  
  
  pBody_RPY_des[2] = _yaw_des;

  vBody_Ori_des[0] = 0.;
  vBody_Ori_des[1] = 0.;
  vBody_Ori_des[2] = _yaw_turn_rate;

  //contact_state = gait->getContactState();
  contact_state = gait->getContactState();
  // END of WBC Update
  //std::cout << "************************: " << contact_state << std::endl;

}

template<>
void ConvexMPCLocomotion::run(ControlFSMData<double>& data) {
  (void)data;
  printf("call to old CMPC with double!\n");

}

void ConvexMPCLocomotion::updateMPCIfNeeded(int *mpcTable, ControlFSMData<float> &data, bool omniMode) {
  //iterationsBetweenMPC = 30/ =27;
  if((iterationCounter % iterationsBetweenMPC) == 0)
  {
    auto seResult = data._stateEstimator->getResult();
    float* p = seResult.position.data();

    Vec3<float> v_des_robot(_x_vel_des, _y_vel_des,_z_vel_des);
    v_des_robot = seResult.rBody.transpose() * v_des_robot;
    Vec3<float> v_des_world = omniMode ? v_des_robot : seResult.rBody.transpose() * v_des_robot;
    //float trajInitial[12] = {0,0,0, 0,0,.25, 0,0,0,0,0,0};

    //printf("Position error: %.3f, integral %.3f\n", pxy_err[0], x_comp_integral);

    if(current_gait == 4)
    {
      float trajInitial[12] = {
        _roll_des,
        _pitch_des /*-hw_i->state_estimator->se_ground_pitch*/,
        (float)stand_traj[5]/*+(float)stateCommand->data.stateDes[11]*/,
        (float)stand_traj[0]/*+(float)fsm->main_control_settings.p_des[0]*/,
        (float)stand_traj[1]/*+(float)fsm->main_control_settings.p_des[1]*/,
        (float)stand_traj[2]/*fsm->main_control_settings.p_des[2]*/,
        0,0,0,0,0,0};

      for(int i = 0; i < horizonLength; i++)
        for(int j = 0; j < 12; j++)
          trajAll[12*i+j] = trajInitial[j];
    }

    else
    {
      const float max_pos_error = .1;
      float xStart = world_position_desired[0];
      float yStart = world_position_desired[1];
      float zStart = world_position_desired[2];

      if(xStart - p[0] > max_pos_error) xStart = p[0] + max_pos_error;
      if(p[0] - xStart > max_pos_error) xStart = p[0] - max_pos_error;

      if(yStart - p[1] > max_pos_error) yStart = p[1] + max_pos_error;
      if(p[1] - yStart > max_pos_error) yStart = p[1] - max_pos_error;

      if(zStart - p[2] > max_pos_error) zStart = p[2] + max_pos_error;
      if(p[2] - zStart > max_pos_error) zStart = p[2] - max_pos_error;

      world_position_desired[0] = xStart;
      world_position_desired[1] = yStart;
      world_position_desired[2] = zStart;

      float trajInitial[12] =
        {
        (float)rpy_comp[0],    // 0
        (float)rpy_comp[1],    // 1
        _yaw_des,              // 2
        xStart,                                   // 3
        yStart,                                   // 4
        zStart,                      // 5
        0,                                        // 6
        0,                                        // 7
        _yaw_turn_rate,                           // 8
        v_des_world[0],                           // 9
        v_des_world[1],                           // 10
        v_des_world[2]                                         // 11
        };

      for(int i = 0; i < horizonLength; i++)
      {
        for(int j = 0; j < 12; j++)
          trajAll[12*i+j] = trajInitial[j];

        if(i == 0) // start at current position  TODO consider not doing this
        {
          //trajAll[3] = hw_i->state_estimator->se_pBody[0];
          //trajAll[4] = hw_i->state_estimator->se_pBody[1];
          trajAll[2] = seResult.rpy[2];
        }
        else
        {
          trajAll[12*i + 3] = trajAll[12 * (i - 1) + 3] + dtMPC * v_des_world[0];
          trajAll[12*i + 4] = trajAll[12 * (i - 1) + 4] + dtMPC * v_des_world[1];
          trajAll[12*i + 5] = trajAll[12 * (i - 1) + 5] + dtMPC * v_des_world[2];
          trajAll[12*i + 2] = trajAll[12 * (i - 1) + 2] + dtMPC * _yaw_turn_rate;
        }
      }
    }
    Timer solveTimer;

    if(_parameters->cmpc_use_sparse > 0.5) {
      solveSparseMPC(mpcTable, data);
    } else {
      solveDenseMPC(mpcTable, data);
    }
    //printf("TOTAL SOLVE TIME: %.3f\n", solveTimer.getMs());
  }

}

void ConvexMPCLocomotion::solveDenseMPC(int *mpcTable, ControlFSMData<float> &data) {
  auto seResult = data._stateEstimator->getResult();

  //float Q[12] = {0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2};
  float Q[12];
  float alpha;
  if (data._quadruped->_robotType == RobotType::IUST){
      //                   roll pitch yaw x  y  z  wx  wy  wz   vx   vy   vz
      
      float Q_IUST[12] = {1, 1, 1, 2, 2, 50, 0, 0, 1, 1, 1, 1}; //based on milab
      alpha = 8e-6; // milab
      memcpy(Q,Q_IUST,sizeof(Q_IUST));
  } else if (data._quadruped->_robotType == RobotType::MINI_CHEETAH){
      float Q_MINI[12] = {0.25, 0.25, 10, 2, 2, 50, 0, 0, 0.3, 0.2, 0.2, 0.1}; //mini cheetah
      alpha = 4e-5; // mini cheetah
      memcpy(Q,Q_MINI,sizeof(Q_MINI));
  }else{
      float Q_CH3[12] = { 1,   1,    1, 1, 1, 50, 0,  0,  1,   1,   1,   1}; // cheetah 3
      alpha = 1e-6; // cheetah 3
      memcpy(Q,Q_CH3,sizeof(Q_CH3));
  }

  //float Q[12] = {0.25, 0.25, 10, 2, 2, 40, 0, 0, 0.3, 0.2, 0.2, 0.2};
  float yaw = seResult.rpy[2];
  float* weights = Q;
  //float alpha = 4e-5; // make setting eventually
  //float alpha = 4e-7; // make setting eventually: DH
  float* p = seResult.position.data();
  float* v = seResult.vWorld.data();
  float* w = seResult.omegaWorld.data();
  float* q = seResult.orientation.data();
  //std::cout << "position1############ (" << p[2] << ")\n";
  float r[12];
  for(int i = 0; i < 12; i++)
    r[i] = pFoot[i%4][i/4]  - seResult.position[i/4];

  //printf("current posistion: %3.f %.3f %.3f\n", p[0], p[1], p[2]);

  if(alpha > 1e-4) {
    std::cout << "Alpha was set too high (" << alpha << ") adjust to 1e-5\n";
    alpha = 1e-5;
  }

  Vec3<float> pxy_act(p[0], p[1], p[2]);
  Vec3<float> pxy_des(world_position_desired[0], world_position_desired[1], world_position_desired[2]);
  //Vec3<float> pxy_err = pxy_act - pxy_des;
  float pz_err = p[2] - _body_height;
  Vec3<float> vxy(seResult.vWorld[0], seResult.vWorld[1], seResult.vWorld[2]);

  Timer t1;
  dtMPC = dt * iterationsBetweenMPC;
  if(data._quadruped->_robotType == RobotType::IUST) {
      setup_problem(dtMPC,horizonLength,0.4,180);
  }else if(data._quadruped->_robotType == RobotType::MINI_CHEETAH){
      setup_problem(dtMPC,horizonLength,0.4,120);
  }else if(data._quadruped->_robotType == RobotType::CHEETAH_3){
      setup_problem(dtMPC,horizonLength,0.4,650); //DH
  }
  //setup_problem(dtMPC,horizonLength,0.4,650); //DH
  update_x_drag(x_comp_integral);
   if(vxy[0] > 0.3 || vxy[0] < -0.3) {
     //x_comp_integral += _parameters->cmpc_x_drag * pxy_err[0] * dtMPC / vxy[0];
     x_comp_integral += _parameters->cmpc_x_drag * pz_err * dtMPC / vxy[0];
   }

  //printf("pz err: %.3f, pz int: %.3f\n", pz_err, x_comp_integral);

  update_solver_settings(_parameters->jcqp_max_iter, _parameters->jcqp_rho,
      _parameters->jcqp_sigma, _parameters->jcqp_alpha, _parameters->jcqp_terminate, _parameters->use_jcqp);
  //t1.stopPrint("Setup MPC");

  Timer t2;
  //cout << "dtMPC: " << dtMPC << "\n";
  if(data._quadruped->_robotType == RobotType::IUST) {
      milab_flag = true;
  }else milab_flag = false;
  update_problem_data_floats(p,v,q,w,r,yaw,weights,trajAll,alpha,mpcTable,milab_flag);
  //t2.stopPrint("Run MPC");
  //printf("MPC Solve time %f ms\n", t2.getMs());

  for(int leg = 0; leg < 4; leg++)
  {
    Vec3<float> f;
    for(int axis = 0; axis < 3; axis++)
      f[axis] = get_solution(leg*3 + axis);

    //printf("[%d] %7.3f %7.3f %7.3f\n", leg, f[0], f[1], f[2]);

    f_ff[leg] = -seResult.rBody * f;
    // Update for WBC
    Fr_des[leg] = f;
  }
}

void ConvexMPCLocomotion::solveSparseMPC(int *mpcTable, ControlFSMData<float> &data) {
  // X0, contact trajectory, state trajectory, feet, get result!
  (void)mpcTable;
  (void)data;
  auto seResult = data._stateEstimator->getResult();

  std::vector<ContactState> contactStates;
  for(int i = 0; i < horizonLength; i++) {
    contactStates.emplace_back(mpcTable[i*4 + 0], mpcTable[i*4 + 1], mpcTable[i*4 + 2], mpcTable[i*4 + 3]);
  }

  for(int i = 0; i < horizonLength; i++) {
    for(u32 j = 0; j < 12; j++) {
      _sparseTrajectory[i][j] = trajAll[i*12 + j];
    }
  }

  Vec12<float> feet;
  for(u32 foot = 0; foot < 4; foot++) {
    for(u32 axis = 0; axis < 3; axis++) {
      feet[foot*3 + axis] = pFoot[foot][axis] - seResult.position[axis];
    }
  }

  _sparseCMPC.setX0(seResult.position, seResult.vWorld, seResult.orientation, seResult.omegaWorld);
  _sparseCMPC.setContactTrajectory(contactStates.data(), contactStates.size());
  _sparseCMPC.setStateTrajectory(_sparseTrajectory);
  _sparseCMPC.setFeet(feet);
  _sparseCMPC.run();

  Vec12<float> resultForce = _sparseCMPC.getResult();

  for(u32 foot = 0; foot < 4; foot++) {
    Vec3<float> force(resultForce[foot*3], resultForce[foot*3 + 1], resultForce[foot*3 + 2]);
    //printf("[%d] %7.3f %7.3f %7.3f\n", foot, force[0], force[1], force[2]);
    f_ff[foot] = -seResult.rBody * force;
    Fr_des[foot] = force;
  }
}

void ConvexMPCLocomotion::initSparseMPC() {
  Mat3<double> baseInertia;
  baseInertia << 0.07, 0, 0,
              0, 0.26, 0,
              0, 0, 0.242;
  double mass = 9;
  double maxForce = 120;

  std::vector<double> dtTraj;
  for(int i = 0; i < horizonLength; i++) {
    dtTraj.push_back(dtMPC);
  }

  Vec12<double> weights;
  weights << 0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2;
  //weights << 0,0,0,1,1,10,0,0,0,0.2,0.2,0;

  _sparseCMPC.setRobotParameters(baseInertia, mass, maxForce);
  _sparseCMPC.setFriction(0.4);
  _sparseCMPC.setWeights(weights, 4e-5);
  _sparseCMPC.setDtTrajectory(dtTraj);

  _sparseTrajectory.resize(horizonLength);
}
void ConvexMPCLocomotion::initMilabSparseMPC() { //For Milab robot
    Mat3<double> baseInertia;
//    baseInertia << 0.1084, 0, 0,   //28kg
//                    0, 0.834, 0,
//                    0, 0, 0.834;

    baseInertia << 0.0891, 0, 0,   //23kg
                    0, 0.685, 0,
                    0, 0, 0.685;

//    baseInertia << 0.0996, 0, 0,   //25.7kg
//                    0, 0.765, 0,
//                    0, 0, 0.765;

    double mass = 23;//25.7;
    double maxForce = 180;

    std::vector<double> dtTraj;
    for(int i = 0; i < horizonLength; i++) {
        dtTraj.push_back(dtMPC);
    }

    Vec12<double> weights;
    weights <<1,  1 , 1,  2,  2,  50,   0,  0,  1,   1,   1,  1;

    _sparseCMPC.setRobotParameters(baseInertia, mass, maxForce);
    _sparseCMPC.setFriction(0.4);
    _sparseCMPC.setWeights(weights, 8e-6);
    _sparseCMPC.setDtTrajectory(dtTraj);

    _sparseTrajectory.resize(horizonLength);
}
