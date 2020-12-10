/*
FILENAME... CNemeDriver.cpp
USAGE...    Motor driver support for the Silverpak 23C and 23CE controller.

Based on some of the motor modules written by:
Mark Rivers
March 1, 2012

-----------------------------
Created by Christian Notthoff
04/12/2020
-----------------------------
Modifications:
11/12/2020 CN: added some code to stop the driver from flooding the port
11/12/2020 CN: added a parameter to initiate a reset
11/12/2020 CN: added a home position variable home_ (mm).
11/12/2020 CN: added soft limit checks to move
-----------------------------
*/

#define CNDEBUG 1

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsMutex.h>
#include <ellLib.h>
#include <iocsh.h>

#include <asynOctetSyncIO.h>

#include <epicsExport.h>
#include "CNemeDriver.h"

static const char *driverName = "CNemeDriver";

typedef struct CNEMEControllerNode {
  ELLNODE node;
  const char *portName;
  CNEMEController *pController;
} CNEMEControllerNode;

static ELLLIST CNEMEControllerList;
static int CNEMEControllerListInitialized = 0;

#define DEFAULT_START      -5000

/** Creates a new CNemeController object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] CNemePortName     The name of the drvAsynSerialPort that was created previously to connect to the silverpak 23C/CE controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time between polls when any axis is moving 
  * \param[in] idlePollPeriod    The time between polls when no axis is moving 
  */
CNEMEController::CNEMEController(const char *portName, const char *CNEMEPortName, int numAxes, 
                                 double movingPollPeriod, double idlePollPeriod)
  : asynMotorController(portName, numAxes, NUM_CNEME_PARAMS, 
			asynInt32Mask | asynFloat64Mask, 
			asynInt32Mask | asynFloat64Mask,
			//ASYN_CANBLOCK | ASYN_MULTIDEVICE,
			ASYN_CANBLOCK,
			1, // autoconnect
			0, 1)  // Default priority and stack size
{
  int axis;
  asynStatus status;
  static const char *functionName = "CNEMEController::CNEMEController";

  createParam(cnmotorResetString, asynParamInt32, &cnmotorReset_);
  
  CNEMEControllerNode *pNode;
  if (!CNEMEControllerListInitialized) {
    CNEMEControllerListInitialized = 1;
    ellInit(&CNEMEControllerList);
  }
  
  /* Connect to CNEME controller */
  status = pasynOctetSyncIO->connect(CNEMEPortName, 0, &pasynUserController_, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
	      "%s: cannot connect to S23C/moxa controller\n",
	      functionName);
  }
  // We should make sure this portName is not already in the list */
  pNode = (CNEMEControllerNode*) calloc(1, sizeof(CNEMEControllerNode));
  pNode->portName = epicsStrDup(portName);
  pNode->pController = this;
  ellAdd(&CNEMEControllerList, (ELLNODE *)pNode);
  
  if (numAxes < 1 ) numAxes = 1;
  numAxes_ = numAxes;
  if(numAxes > MAX_CNEME_AXES){
    numAxes = MAX_CNEME_AXES;
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
	      "%s: This driver only supports up to %d axis on one serial chain!\n",
	      functionName,MAX_CNEME_AXES);
  }
  for (axis=0; axis<numAxes; axis++) {
    new CNEMEAxis(this, axis);
    setDoubleParam(axis, this->motorPosition_, DEFAULT_START);
    setDoubleParam(axis, this->motorEncoderPosition_, DEFAULT_START);
  }
  
  startPoller(movingPollPeriod, idlePollPeriod, 2);
}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information it calls asynMotorController::report()
  */
void CNEMEController::report(FILE *fp, int level)
{
  fprintf(fp, "Silverpak 23C/CE motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n", 
    this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

  // Call the base class method
  asynMotorController::report(fp, level);
}


asynStatus CNEMEController::writeReadController(){
  //static const char *functionName = "CNEMEController::writeReadController";
  asynStatus status;
  status = asynMotorController::writeReadController();

#ifdef CNDEBUG
  printf("out: %s\n",outString_);
  printf("in: %s\n",inString_);
#endif
  
  char *tmpc=inString_;
  
  if(tmpc[0] != '\0' && !status){
    tmpc+=1;
    sprintf(inString_,"%s",tmpc);
    tmpc=inString_;
    //CNtodo might needs modification if we want to use n16R mode. 
    if( tmpc[2] != '`' && tmpc[2] !='@'){
      sprintf(outString_,"/1TR");
      asynMotorController::writeReadController();
      printf("command error, stopping motor.\n Error was: %d\n",tmpc[2]);
      return asynError;
    }
    while(tmpc[0] != '\0'){
      if(tmpc[0] == 0x03){
	tmpc[0] = '\0';
	break;
      }
      tmpc++;
    }
  }
  return status;
}

asynStatus CNEMEController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  CNEMEAxis *pAxis = this->getAxis(pasynUser);
  static const char *functionName = "CNEMEController::writeInt32";

  /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
   * status at the end, but that's OK */
  pAxis->setIntegerParam(function, value);
  
  if( function == cnmotorReset_){
    double res=0.;
    getDoubleParam(pAxis->axisNo_,motorRecResolution_,&res);
    pAxis->setPosition((int) (pAxis->home_/res));
    pAxis->setIntegerParam(function, 0);
    status = asynSuccess;
  }else if( function == motorStop_){
    //CNtodo not sure if we really need this stop
    sprintf(this->outString_,"/%01dTRn0R",pAxis->axisNo_+1);
    status = this->writeReadController();
  };

  /* Call base class call its method (if we have our parameters check this here) */
  //CNtodo not sure if this should be here at all
  status = asynMotorController::writeInt32(pasynUser, value);
  
  /* Do callbacks so higher layers see any changes */
  pAxis->callParamCallbacks();
  if (status) 
    asynPrint(pasynUser, ASYN_TRACE_ERROR, 
              "%s:%s: error, status=%d function=%d, value=%d\n", 
              driverName, functionName, status, function, value);
  else        
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s: function=%d, value=%d\n", 
              driverName, functionName, function, value);
  return status;
}

/** Returns a pointer to an CNEMEAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
CNEMEAxis* CNEMEController::getAxis(asynUser *pasynUser)
{
  return static_cast<CNEMEAxis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an CNEMEAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number. */
CNEMEAxis* CNEMEController::getAxis(int axisNo)
{
  return static_cast<CNEMEAxis*>(asynMotorController::getAxis(axisNo));
}


// These are the CNEMEAxis methods

/** Creates a new CNEMEAxis object.
  * \param[in] pC Pointer to the CNEMEController to which this axis belongs. 
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  * 
  * Initializes register numbers, etc.
  */
CNEMEAxis::CNEMEAxis(CNEMEController *pC, int axisNo)
  : asynMotorAxis(pC, axisNo),
    pC_(pC)
{
  setIntegerParam(pC_->motorStatusHasEncoder_,1);
  
  limitmap[0]=-1;
  limitmap[1]=-1;
  limitmap[2]=-1;
  isInitialised_=false;
  home_=15.0;
  posMode_=8; //Check manual for valid numbers, no sanity check performed here.
}

/** Reports on status of the axis
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * After printing device-specific information calls asynMotorAxis::report()
  */
void CNEMEAxis::report(FILE *fp, int level)
{
  if (level > 0) {
    fprintf(fp, "  axis %d\n",
            axisNo_);
  }

  // Call the base class method
  asynMotorAxis::report(fp, level);
}

asynStatus CNEMEAxis::sendAccelAndVelocity(double acceleration, double velocity) 
{
  return(asynSuccess);
  //asynStatus status;
  // static const char *functionName = "CNEME::sendAccelAndVelocity";
  //CNtodo
  //return status;
}


/** Set the high limit position of the motor.
  * \param[in] highLimit The new high limit position that should be set in the hardware. Units=steps.*/
asynStatus CNEMEAxis::setHighLimit(double highLimit)
{
  hl_=highLimit;
  return asynSuccess;
}


/** Set the low limit position of the motor.
  * \param[in] lowLimit The new low limit position that should be set in the hardware. Units=steps.*/
asynStatus CNEMEAxis::setLowLimit(double lowLimit)
{
  ll_=lowLimit;
  return asynSuccess;
}

asynStatus CNEMEAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
    static const char *functionName = "CNEMEAxis::move";
#ifdef CNDEBUG
    printf("move:pos=%f, minV=%f, maxV=%f, acc=%f\n",position,minVelocity,maxVelocity,acceleration);
#endif
    
    asynStatus status;
    int pos = (int) fabs(position);
    char* tmpc=NULL;
    
    if(!isInitialised_){
      //setIntegerParam(pC_->motorStatusProblem_,1);
      asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, 
		"%s: motor needs initialisation...\n",
		functionName);
      return asynError;
    }
    
    
    /* Check for soft limits */
    if((position > hl_ || position < ll_) && !relative){
      asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, 
		"%s: move outside softlimits is not allowed.\n",
		functionName);
      return asynError;
    }
    int ismoving=0;
    pC_->getIntegerParam(axisNo_,pC_->motorStatusMoving_,&ismoving);
    if(ismoving){
      setIntegerParam(pC_->motorStatusProblem_,1);
      asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, 
		"%s: motor is already moving need to implement something to handl this...\n",
		functionName);
      return asynError;
    }
    
    // recover from a stop command where /1n0R was issued by setting the motor back to e.g /1n8R (position correction mode)
    if( (relative && fabs(position) > 0) || relative == 0){
      sprintf(pC_->outString_,"/%01dn%dR",axisNo_+1,posMode_);
      status = pC_->writeReadController();
    }
    if(relative){
      if(position > 0){
	//positive direction
	sprintf(pC_->outString_,"/%01dP%dR",axisNo_+1,pos);
      }else if(position < 0){
	//negative direction
	sprintf(pC_->outString_,"/%01dD%dR",axisNo_+1,pos);
      }else{
	// /1P0R and /1D0R are infinit moves!!!
	asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, 
		  "%s: a zero relative move is odd, I will ignore it.\n",
		  functionName);
	return asynError;
      };
    }else{
      sprintf(pC_->outString_,"/%01dA%dR",axisNo_+1,(int) position);
    }
    status = pC_->writeReadController();
    //CNtodo
    if(!status){
      tmpc=pC_->inString_;
      if(tmpc[0] != '/' || tmpc[1] != '0'){
	asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, 
		  "%s: readback error from S23C/moxa controller, action needs to be impl....\n",
		  functionName);
      }else if(tmpc[2] != '`' && tmpc[2] != '@'){
	printf("debug:%s\n",tmpc);
	asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, 
		  "%s: error in S23C/moxa controller command, status bit: %d\n",
		  functionName,tmpc[2]);
      }
    }else{
      //CNtodo how to handel this?
      asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, 
		"%s: error communicating with S23C/moxa controller, action needs to be impl....\n",
		functionName);
    }
    
    setIntegerParam(pC_->motorStatusDone_, 0);
    callParamCallbacks();
    
    asynPrint(pasynUser_, ASYN_TRACE_FLOW, 
	      "%s:%s: Set driver %s, axis %d move to %f, min vel=%f, max vel=%f, accel=%f\n",
	      driverName, functionName, pC_->portName, axisNo_, position, minVelocity, maxVelocity, acceleration );
    return asynSuccess;
}

asynStatus CNEMEAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  static const char *functionName = "CNEMEAxis::home";
  asynPrint(pasynUser_, ASYN_TRACE_ERROR,
    "%s: minVelocity=%f, maxVelocity=%f, acceleration=%f, dir=%d\n to be implemented\n",
	    functionName, minVelocity, maxVelocity, acceleration,forwards);
  return (asynSuccess);
}

asynStatus CNEMEAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status;
  static const char *functionName = "CNEMEAxis::moveVelocity";

  asynPrint(pasynUser_, ASYN_TRACE_ERROR,
    "%s: minVelocity=%f, maxVelocity=%f, acceleration=%f\n not implemented\n",
    functionName, minVelocity, maxVelocity, acceleration);
  
  return(asynSuccess);
  status = sendAccelAndVelocity(acceleration, maxVelocity);
  //CNtodo to be implemented, but I don't like this mode without a limit switch!
  
  return status;
}

asynStatus CNEMEAxis::stop(double acceleration )
{
  asynStatus status;
  //static const char *functionName = "CNEMEAxis::stop";
  sprintf(pC_->outString_,"/%01dTR",axisNo_+1);
  status = pC_->writeReadController();
  sprintf(pC_->outString_,"/%01dn0R",axisNo_+1);
  status = pC_->writeReadController();

  return status;
}

asynStatus CNEMEAxis::setPosition(double position)
{
  asynStatus status;
  static const char *functionName = "CNEMEAxis::setPosition";
  char* tmpc;
  int pos = (int) position;
  /*
  int ismoving=0;
  pC_->getIntegerParam(axisNo_,pC_->motorStatusMoving_,&ismoving);
  if(ismoving){
    setIntegerParam(pC_->motorStatusProblem_,1);
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, 
	      "%s: motor is moving, reseting the zero now is not a good idea...\n",
	      functionName);
    return asynError;
  }
  */

  sprintf(pC_->outString_,"/%01dTR",axisNo_+1);
  status = pC_->writeReadController();
  sprintf(pC_->outString_,"/%01dn0z0R",axisNo_+1);
  status = pC_->writeReadController();
  sprintf(pC_->outString_,"/%01dn0z0R",axisNo_+1);
  status = pC_->writeReadController();
  tmpc=pC_->inString_;
  if(status != asynSuccess) return asynError;
  if(tmpc[2] != '`' && tmpc[2] != '@') return asynError;
  
  sprintf(pC_->outString_,"/%01dh6m60j256V68888aE10000aC100n0R",axisNo_+1);
  status = pC_->writeReadController();
  tmpc=pC_->inString_;
  if(status != asynSuccess) return asynError;
  if(tmpc[2] != '`' && tmpc[2] != '@') return asynError;
  
  sprintf(pC_->outString_,"/%01dz%dn%dRaE10250R",axisNo_+1,pos,posMode_);
  status = pC_->writeReadController();
  
  if(!status){
    tmpc=pC_->inString_;
    if(tmpc[0] != '/' || tmpc[1] != '0'){
      asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, 
	      "%s: readback error from S23C/moxa controller, action needs to be impl....\n",
	      functionName);
    }else{
      //CNtodo may be, maybe not?
      if(tmpc[2] != '`' && tmpc[2] != '@'){
	printf("setPosition readback was:%s\n needs to be implemented",tmpc);
      }
    }
  }else{
    //CNtodo raise a alarm flag if there is something wrong with the motor respponse
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, 
	      "%s: error from S23C/moxa controller, action needs to be impl....\n",
	      functionName);
  }
  
  setDoubleParam (pC_->motorPosition_,         position);
  setDoubleParam (pC_->motorEncoderPosition_,  position);

  asynPrint(pasynUser_, ASYN_TRACE_FLOW,
    "%s: reset position to %f\n",
    functionName, position);
  
  if(!status) isInitialised_ = true;

  callParamCallbacks();
  
  return status;
}

asynStatus CNEMEAxis::config(int home, int vel)
{
  //CNtodo to be imple
  //static const char *functionName = "CNEMEAxis::setPosition";
  //home_ = home;
  //homed_ = 0;
  //posMode_ = 8;
  return asynSuccess;
}

/** Polls the axis.
  * This function reads the motor position, the limit status, the home status, the moving status, 
  * and the drive power-on status. 
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false). */
asynStatus CNEMEAxis::poll(bool *moving)
{
  static const char *functionName = "CNEMEAxis::poll";
  int done=0;
  //int limit;
  double position=-5000;
  asynStatus status;
  char *tmpc=NULL;
  bool mo=false;
  double res=0.;
  
  int con=0;
  pasynManager->isConnected(pC_->pasynUserController_,&con);
  //printf("status %s\n",(con)? "connected":"disconnected");
  //printf("status %s\n",(isInitialised_)? "init":"uninit");
  if(!con || !isInitialised_){
    status = asynError;
    setDoubleParam (pC_->motorEncoderPosition_, DEFAULT_START);
    setDoubleParam (pC_->motorPosition_,        DEFAULT_START);
    setIntegerParam(pC_->motorStatusDone_,      1);
    setIntegerParam(pC_->motorStatusMoving_,    0);
    if(con){
      status = asynSuccess;
      pasynOctetSyncIO->flush(pC_->pasynUserController_);
    }
    *moving = false;
    isInitialised_=false;
    goto skip;
  }
  /*
  if(limitmap[0] != -1 || limitmap[1] != -1 || limitmap[2] != -1){
    // Read the input pin bit pattern
    sprintf(pC_->outString_,"/%01d?4",axisNo_+1);
    status = pC_->writeReadController();
    if (status) goto skip;
    tmpc=pC_->inString_;
    if(tmpc[0] != '/' || tmpc[1] != '0'){
      asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, 
		"%s: readback error from S23C/moxa controller\n",
		functionName);
      status = asynError;
      goto skip;
    }else{
      if(tmpc[2] == '`' || tmpc[2] == '@'){
	// *moving = mo;
	mo = tmpc[2] == '@' ? true:false;
	done = tmpc[2] == '`' ? true:false;
      }
      //printf("hl=%d\n",(int)(tmpc[3] & limitmap[0]));
      //printf("ll=%d\n",(int)(tmpc[3] & limitmap[1]));
      //printf("hh=%d\n",(int)(tmpc[3] & limitmap[2]));
      if(limitmap[0] != -1){//high
	setIntegerParam(pC_->motorStatusHighLimit_, (tmpc[3] & limitmap[0]) ? 1:0);
	if((tmpc[3] & limitmap[0])){
	  stop(0);
	  //double position, int relative, double minVelocity, double maxVelocity, double acceleration
	  move(-100,1,100,100,100);
	  setIntegerParam(pC_->motorStatusProblem_,1);
	}
      }
      if(limitmap[1] != -1){//low
	setIntegerParam(pC_->motorStatusLowLimit_, (tmpc[3] & limitmap[1]) ? 1:0);
	if((tmpc[3] & limitmap[1])){
	  stop(0);
	  move(100,1,100,100,100);
	  setIntegerParam(pC_->motorStatusProblem_,1);
	}
      }
      if(limitmap[2] != -1){//home
	setIntegerParam(pC_->motorStatusAtHome_, (tmpc[3] & limitmap[2]) ? 1:0);
      }
    } 
  }
  */
  
  // Read the current encoder position
  sprintf(pC_->outString_,"/%01d?8",axisNo_+1);
  status = pC_->writeReadController();
  if (status) goto skip;
  tmpc=pC_->inString_;
  if(tmpc[0] != '/' || tmpc[1] != '0'){
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, 
	      "%s: readback error from S23C/moxa controller\n",
	      functionName);
    status = asynError;
    goto skip;
  }else{
    if(tmpc[2] == '`' || tmpc[2] == '@'){
      mo = tmpc[2] == '@' ? true:false;
      done = tmpc[2] == '`' ? true:false;
      tmpc+=3;
      if(tmpc[0]!='\0'){
	position = strtol(tmpc,NULL,10);
	setDoubleParam (pC_->motorEncoderPosition_, position);
      }else{
	//CNtodo do we have to handle this case?
	asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, 
		  "%s: Not able to read encoder position\n",
		  functionName);
	status = asynError;
	goto skip;
      }
    }
  }

  // Read the current motor position
  sprintf(pC_->outString_,"/%01d?0",axisNo_+1);
  status = pC_->writeReadController();
  if (status) goto skip;
  tmpc=pC_->inString_;
  if(tmpc[0] != '/' || tmpc[1] != '0'){
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, 
	      "%s: readback error from S23C/moxa controller\n",
	      functionName);
    status = asynError;
    goto skip;
  }else{
    if(tmpc[2] == '`' || tmpc[2] == '@'){
      mo = tmpc[2] == '@' ? true:false;
      done = tmpc[2] == '`' ? true:false;
      tmpc+=3;
      if(tmpc[0]!='\0'){
	position = strtol(tmpc,NULL,10);
	setDoubleParam (pC_->motorPosition_,        position);
      }else{
	//CNtodo do we have to handle this case?
	asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, 
		  "%s: Not able to read motor position\n",
		  functionName);
	status = asynError;
	goto skip;
      }
    }
  }
  
  setIntegerParam(pC_->motorStatusDone_,      done);
  setIntegerParam(pC_->motorStatusMoving_,   !done);
  *moving = mo;
  
  // Read the limit status
  //setIntegerParam(pC_->motorStatusHighLimit_, limit);
  //setIntegerParam(pC_->motorStatusLowLimit_, limit);
  //setIntegerParam(pC_->motorStatusAtHome_, limit);
  //setIntegerParam(pC_->motorStatusPowerOn_, driveOn/Off);
  //setIntegerParam(pC_->motorStatusDirection_,  ?);

  //pC_->getDoubleParam(pC_->motorRecResolution_,&res);
  //setIntegerParam(pC_->motorStatusHome_,       !(position == (int) (home_/res)));
  //setIntegerParam(pC_->motorStatusHomed_,      (homed_ == 1));

  skip:
  setIntegerParam(pC_->motorStatusProblem_, status ? 1:0);
  callParamCallbacks();
  return status ? asynError : asynSuccess;
}


/** Creates a new CNEMEController object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] CNEMEPortName       The name of the drvAsynIPPPort that was created previously to connect to the CNEME controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving 
  */
extern "C" int CNEMECreateController(const char *portName, const char *CNEMEPortName, int numAxes, 
                                   int movingPollPeriod, int idlePollPeriod)
{
  //CNEMEController *pCNEMEController =
  new CNEMEController(portName, CNEMEPortName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000.);
  //pCNEMEController = NULL;
  return(asynSuccess);
}

extern "C" int CNEMEConfigAxis(const char *portName, int axis, int home, int vel)
{
  CNEMEControllerNode *pNode;
  static const char *functionName = "CNEMEConfigAxis";
  
  // Find this controller
  if (!CNEMEControllerListInitialized) {
    printf("%s:%s: ERROR, controller list not initialized\n",
      driverName, functionName);
    return(-1);
  }
  pNode = (CNEMEControllerNode*)ellFirst(&CNEMEControllerList);
  while(pNode) {
    if (strcmp(pNode->portName, portName) == 0) {
      printf("%s:%s: configuring controller %s axis %d\n",
             driverName, functionName, pNode->portName, axis); 
             pNode->pController->getAxis(axis)->config(home, vel);
      return(0);
    }
    pNode = (CNEMEControllerNode*)ellNext((ELLNODE*)pNode);
  }
  printf("Controller not found\n");
  return(-1);
}

/** Code for iocsh registration */
static const iocshArg CNEMECreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg CNEMECreateControllerArg1 = {"S23C port name", iocshArgString};
static const iocshArg CNEMECreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg CNEMECreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg CNEMECreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const CNEMECreateControllerArgs[] = {&CNEMECreateControllerArg0,
                                                             &CNEMECreateControllerArg1,
                                                             &CNEMECreateControllerArg2,
                                                             &CNEMECreateControllerArg3,
                                                             &CNEMECreateControllerArg4};
static const iocshFuncDef CNEMECreateControllerDef = {"CNEMECreateController", 5, CNEMECreateControllerArgs};

static void CNEMECreateContollerCallFunc(const iocshArgBuf *args)
{
  CNEMECreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static const iocshArg CNEMEConfigAxisArg0 = { "Port name",     iocshArgString};
static const iocshArg CNEMEConfigAxisArg1 = { "Axis #",        iocshArgInt};
static const iocshArg CNEMEConfigAxisArg2 = { "Home position", iocshArgInt};
static const iocshArg CNEMEConfigAxisArg3 = { "Vel",    iocshArgInt};

static const iocshArg *const CNEMEConfigAxisArgs[] = {
  &CNEMEConfigAxisArg0,
  &CNEMEConfigAxisArg1,
  &CNEMEConfigAxisArg2,
  &CNEMEConfigAxisArg3
};
static const iocshFuncDef CNEMEConfigAxisDef ={"CNEMEConfigAxis",4,CNEMEConfigAxisArgs};

static void CNEMEConfigAxisCallFunc(const iocshArgBuf *args)
{
  CNEMEConfigAxis(args[0].sval, args[1].ival, args[2].ival, args[3].ival);
}


static void CNemeRegister(void)
{
  iocshRegister(&CNEMECreateControllerDef, CNEMECreateContollerCallFunc);
  iocshRegister(&CNEMEConfigAxisDef, CNEMEConfigAxisCallFunc);
}

extern "C" {
epicsExportRegistrar(CNemeRegister);
}
