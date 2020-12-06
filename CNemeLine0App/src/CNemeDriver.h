/*
FILENAME...   CNEMEDriver.h
USAGE...    Motor driver support for the Silverpak 23C and 23CE controller.

Based on some of the motor modules written by:
Mark Rivers
March 1, 2012

-----------------------------
Created by Christian Notthoff
04/12/2020
-----------------------------
Modifications:
-----------------------------
*/

#include <epicsTime.h>
#include <epicsThread.h>

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#define MAX_CNEME_AXES 9

// No controller-specific parameters yet
#define NUM_CNEME_PARAMS 0  

class epicsShareClass CNEMEAxis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  CNEMEAxis(class CNEMEController *pC, int axis);
  void report(FILE *fp, int level);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);
  asynStatus setPosition(double position);
  //asynStatus setClosedLoop(bool closedLoop);
  
  /* These are the methods that are new to this class */
  asynStatus config(int home, int vel);
  
 private:
  CNEMEController *pC_;          /**< Pointer to the asynMotorController to which this axis belongs.
                                  *   Abbreviated because it is used very frequently */
  asynStatus sendAccelAndVelocity(double accel, double velocity);

  int limitmap[3];// high, low, home. if -1 not in use
  bool isInitialised_;
  
  friend class CNEMEController;
};

class epicsShareClass CNEMEController : public asynMotorController {
 public:
  CNEMEController(const char *portName, const char *CNEMEPortName, int numAxes, double movingPollPeriod, double idlePollPeriod);
  
  asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    
  void report(FILE *fp, int level);
  CNEMEAxis* getAxis(asynUser *pasynUser);
  CNEMEAxis* getAxis(int axisNo);
  
  friend class CNEMEAxis;
};
