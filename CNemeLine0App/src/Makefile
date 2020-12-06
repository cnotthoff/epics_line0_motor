TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build the IOC application

PROD_IOC = CNemeLine0
# CNemeLine0.dbd will be created and installed
DBD += CNemeLine0.dbd

# CNemeLine0.dbd will be made up from these files:
CNemeLine0_DBD += base.dbd
CNemeLine0_DBD += devCNemeMotor.dbd
CNemeLine0_DBD += motorSupport.dbd
CNemeLine0_DBD += asyn.dbd
CNemeLine0_DBD += drvAsynIPPort.dbd
# Include dbd files from all support applications:
#CNemeLine0_DBD += xxx.dbd

# Add all the support libraries needed by this IOC
#CNemeLine0_LIBS += xxx
CNemeLine0_LIBS += asyn
CNemeLine0_LIBS += motor

# CNemeLine0_registerRecordDeviceDriver.cpp derives from CNemeLine0.dbd
CNemeLine0_SRCS += CNemeLine0_registerRecordDeviceDriver.cpp
CNemeLine0_SRCS += CNemeDriver.cpp

# Build the main IOC entry point on workstation OSs.
CNemeLine0_SRCS_DEFAULT += CNemeLine0Main.cpp
CNemeLine0_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#CNemeLine0_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
CNemeLine0_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

