# CNemeLine0
EPICS ioc for EME Line0 motor with a motor drivers for the following [LIN ENGINEERING](webpage todo) controller: Silverpack 23C and 23CE

CNemeLine0 uses the model 3 driver based on [motor](https://github.com/epics-modules/motor).

This is a development version most stuff is not tested yet. Use at your on risk!


#/1rR recover from overload timeout
#/1auXXXR sets number of retries before overload timeout
#/1n16R enable overload report mode (bit 4) xor with /1n8R position correction mode (bit 3)

#/1N1R (default) uses input (pin 7) for home
#/1n2R enable opto limit mode (bit1) using inputs 3 (pin 7) & 4 (pin 14)