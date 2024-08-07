#include "MIT_Controller.hpp"
float gholam;

MIT_Controller::MIT_Controller():RobotController(){  }

//#define RC_ESTOP
/**
 * Initializes the Control FSM.
 */
void MIT_Controller::initializeController() {
  // Initialize a new GaitScheduler object
  _gaitScheduler = new GaitScheduler<float>(&userParameters, _controlParameters->controller_dt);

  std::cout << gholam;
  // Initialize a new ContactEstimator object
  //_contactEstimator = new ContactEstimator<double>();
  ////_contactEstimator->initialize();
  
  

  // Initializes the Control FSM with all the required data
  _controlFSM = new ControlFSM<float>(_quadruped, _stateEstimator,
                                      _legController, _gaitScheduler,
                                      _desiredStateCommand, _controlParameters, 
                                      _visualizationData, &userParameters);
}

/**
 * Calculate the commands for the leg controllers using the ControlFSM logic.
 */
void MIT_Controller::runController() {
  // Find the current gait schedule
  _gaitScheduler->step();

  //gholam = _stateEstimate->position[2];
 
  //std::cout << "posbodyZ " << gholam << std::endl;
  //std::cout << "posgroundZ " << _model->_gcLocation[1] << std::endl;
  
  // Find the desired state trajectory
  _desiredStateCommand->convertToStateCommands();

  // Run the Control FSM code
  _controlFSM->runFSM();
}


