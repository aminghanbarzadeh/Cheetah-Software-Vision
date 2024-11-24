#ifndef JPOS_CONTROLLER
#define JPOS_CONTROLLER

#include <RobotController.h>
#include "IustUserParameters.h"

class IUST_Controller:public RobotController{
  public:
    IUST_Controller():RobotController(),_jpos_ini(cheetah::num_act_joint){
    _jpos_ini.setZero();
    }
    virtual ~IUST_Controller(){}

    virtual void initializeController();
    virtual void runController();
    virtual void updateVisualization(){}
    virtual ControlParameters* getUserControlParameters() {
      return &userParameters;
    }

  private:
    int debug_iter;

  protected:
    DVec<float> _jpos_ini;
    IustUserParameters userParameters;
};

#endif
