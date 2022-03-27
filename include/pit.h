/*----------------------------------------------------------------------------
       ECS642/ECS714 Lab Work

    Header File for PIT
    
    Functions:
      configurePIT(int channel) - configure channel 0 or 1
      startTimer(int channel)   - start PIT running
      stopTimer(int channel)    - stop it
      setTimer(int channel, uint32_t timeout) - set channel count (no need to stop)
*---------------------------------------------------------------------------*/

#ifndef PIT_H
#define PIT_H
#include <MKL25Z4.h>

// The PIT clock is half the CPU clock
#define PITCLOCK (10485760)

// prototypes
void configurePIT(int channel) ;
void startTimer(int channel) ;
void stopTimer(int channel) ;
void setTimer(int channel, uint32_t timeout) ;

#endif
