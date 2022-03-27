/* ------------------------------------------
       ECS642/ECS714 Lab3

   The LED is displayed at different brightness levels using PWM
   The PIT is used to time the transition between the brightness levels
   A button press switches between two rates (by changing the PIT load value): 
       * a fast one cycles through all the brightness levels in 2 s
       * a slow one takes 10 s
  -------------------------------------------- */

#include <MKL25Z4.h>
#include <stdbool.h>
#include "../include/SysTick.h"
#include "../include/button.h"

#include "../include/pit.h"
#include "../include/TPMPWM.h"
#include "../include/triColorLedPWM.h"


#define BlueInc 1
#define RedInc 2
#define GreenInc 3
#define BlueDec 4
#define RedDec 5
#define GreenDec 6
#define StateW 7
#define StateX 8
#define StateY 9
#define StateZ 10

#define MAXBRIGHTNESS 31
/* --------------------------------------
     Documentation
     =============
     This is a cyclic system with a cycle time of 10ms

     The file has a main function, two tasks
       1. pollB1Task: this polls shield button B1
       2. toggleRateTask: this toggles between a fast and slow rate for changing the LED brightness
     and the PIT interrupt service routine which changes the brightness of 
     one of the LEDs
 -------------------------------------- */
 
/* --------------------------------------------
  Variables for communication
*----------------------------------------------------------------------------*/


bool pressedB1_ev =false;  // set by task1 (polling) and cleared by task 2
bool pressedB2_ev =false;


/*----------------------------------------------------------------------------
  task1pollB1
  
  This task polls button B1. Keep this task.
*----------------------------------------------------------------------------*/
int b1State ;        // Current state - corresponds to position
int b1BounceCount ;
int b2State =BOPEN;        // Current state - corresponds to position
int b2BounceCount ;


void initPollB1Task() {
    b1State = BOPEN ;
    pressedB1_ev = false ; 
    b1BounceCount = 0 ;
    b2BounceCount = 0 ;
}
volatile int R=31;
volatile int G=0;
volatile int B=0;
volatile int CurrState=BlueInc; //assigning current state
int flag=1; //using flag to distinguish between the if statements in both patterns when B2 is pressed

void ChangeColors(){

	if(pressedB2_ev && flag==1){    //checks if B2 is pressed and if yes, then it changes the current state to StateW
		R=0;
		G=0;
		B=0;
		CurrState=StateW;
		flag=0;
		pressedB2_ev=false;
		
	}
	
	
switch(CurrState){	
	case BlueInc:
			if(B==MAXBRIGHTNESS){   //checks if blue is at maxbrightness and if yes, then it moves to next state
				CurrState=RedDec;
			}
			else{
				B++;    //incrementing the value of B
				
			}
			break;
	
	case RedDec:
			if(R==0){   //checks if red is at 0 and if yes, then it moves to next state
				CurrState=GreenInc;
			}
			else{
				R--;    //decrementing the value of R
				
			}
			break;
		
	case GreenInc:  
			if(G==MAXBRIGHTNESS){   //checks if green is at maxbrightness and if yes, then it moves to next state
				CurrState=BlueDec;
			}
			else{
				G++;    //incrementing the value of G
				
			}
			break;
		
	case BlueDec:
			if(B==0){   //checks if blue is at 0 and if yes, then it moves to next state
				CurrState=RedInc;
			}
			else{
				B--;    //decrementing the value of B
				
			}
			break;
		
	case RedInc:
			if(R==MAXBRIGHTNESS){   //checks if red is at maxbrightness and if yes, then it moves to next state
				CurrState=GreenDec;
			}
			else{
				R++;    //incrementing the value of R
				
			}
			break;
		
	case GreenDec:
			if(G==0){   //checks if green is at 0 and if yes, then it moves to next state
				CurrState=BlueInc;
			}
			else{
				G--;    //decrementing the value of G
				
			}
			break;
}
	
}
void ChangeColors1(){
	
	if(pressedB2_ev && flag==0){    //checks if B2 is pressed and if yes, then it changes the current state to BlueInc
		R=31;
		G=0;
		B=0;
		flag=1;
		CurrState=BlueInc;
		pressedB2_ev=false;
	
	}
	
	
switch(CurrState){	
	case StateW:
			if(B==MAXBRIGHTNESS){   //checks if blue is at maxbrightness and if yes, then it moves to next state
				CurrState=StateX;
			}
			else{
				B++;    //incrementing the value of B
				R++;    //incrementing the value of R
				
			}
			break;
	
	case StateX:
			if(B==0){   //checks if blue is at 0 and if yes, then it moves to next state
				CurrState=StateY;
			}
			else{
				B--;    //decrementing the value of B
				G++;    //incrementing the value of G
				
			}
			break;
		
	case StateY:
			if(B==MAXBRIGHTNESS){   //checks if blue is at maxbrightness and if yes, then it moves to next state
				CurrState=StateZ;
			}
			else{
				B++;    //incrementing the value of B
				R--;    //decrementing the value of R
				
			}
			break;
		
	case StateZ:
			if(B==0){   //checks if blue is at 0 and if yes, then it moves to next state
				CurrState=StateW;
			}
			else{
				B--;    //decrementing the value of B
				G--;    //decrementing the value of G
			
			}
			break;

}
	
}



void pollB1Task() {     // Generate signals for a simulated button B1
    if (b1BounceCount > 0) b1BounceCount -- ;
    switch (b1State) {
        case BOPEN:
            if (isPressed(B1MASK)) {
                b1State = BCLOSED ;
                pressedB1_ev = true ; 
            }
          break ;

        case BCLOSED:
            if (!isPressed(B1MASK)) {
                b1State = BBOUNCE ;
                b1BounceCount = BOUNCEDELAY ;
            }
            break ;

        case BBOUNCE:
            if (isPressed(B1MASK)) {
                b1State = BCLOSED ;
            }
            else if (b1BounceCount == 0) {
                b1State = BOPEN ;
            }
            break ;
    }                
}

void pollB2Task() {     // Generate signals for a simulated button B2
    if (b2BounceCount > 0) b2BounceCount -- ;
    switch (b2State) {
        case BOPEN:
            if (isPressed(B2MASK)) {
                b2State = BCLOSED ;
                pressedB2_ev = true ; 
            }
          break ;

        case BCLOSED:
            if (!isPressed(B2MASK)) {
                b2State = BBOUNCE ;
                b2BounceCount = BOUNCEDELAY ;
            }
            break ;

        case BBOUNCE:
            if (isPressed(B2MASK)) {
                b2State = BCLOSED ;
            }
            else if (b2BounceCount == 0) {
                b2State = BOPEN ;
            }
            break ;
    }                
}



/* -------------------------------------
    Programmable Interrupt Timer (PIT) interrupt handler

      Check each channel to see if caused interrupt
      Write 1 to TIF to reset interrupt flag

    Changes the LED brightness. KEEP THIS ISR but changes are needed
   ------------------------------------- */

// PIT load values
// The larger the count, the lower the frequency of interrupts
const uint32_t pitSlowCount = PITCLOCK * 9 / 32 ; // all 32 levels in 10 s
const uint32_t pitFastCount = PITCLOCK * 2 / 32 ; // all 32 levels in 2 s
const uint32_t pitMedCount = PITCLOCK * 5 / 32 ; // all 32 levels in 5 s

// Brightness level
unsigned int bright = 0 ;  // the current brightness

void PIT_IRQHandler(void) {
    // clear pending interrupts
    NVIC_ClearPendingIRQ(PIT_IRQn);

	
	
    if (PIT->CHANNEL[0].TFLG & PIT_TFLG_TIF_MASK) {
        // clear TIF
        PIT->CHANNEL[0].TFLG = PIT_TFLG_TIF_MASK ;

			
        // add code here for channel 0 interrupt
        ChangeColors();      //calling the function ChangeColors()
		ChangeColors1();    //calling the function ChangeColors1()
		setLEDBrightness(Green, G) ;    //calling setLEDBrightness() function for green LED
		setLEDBrightness(Blue, B) ;     //calling setLEDBrightness() function for blue LED
		setLEDBrightness(Red, R) ;      //calling setLEDBrightness() function for red LED
			
        // -- start of demo code    
        /*bright = (bright + 1) % (MAXBRIGHTNESS + 1);        
        setLEDBrightness(Blue, bright) ;*/
        // -- end of demo code
    }

    if (PIT->CHANNEL[1].TFLG & PIT_TFLG_TIF_MASK) {
        // clear TIF
        PIT->CHANNEL[1].TFLG = PIT_TFLG_TIF_MASK ;

        // add code here for channel 1 interrupt
        // -- end of demo code
    }
}

/*----------------------------------------------------------------------------
   Task: toggleRateTask

   Toggle the rate of upadtes to the LEDs on every signal

   KEEP THIS TASK, but changes may be needed
*----------------------------------------------------------------------------*/

#define FAST 0
#define SLOW 1
#define MEDIUM 2

int rateState ;  // this variable holds the current state

// initial state of task
void initToggleRateTask() {
    setTimer(0, pitSlowCount) ;
    rateState = SLOW ;
}

void toggleRateTask() {
    switch (rateState) {

            
        case SLOW:
            if (pressedB1_ev) {                   // signal received
                pressedB1_ev = false ;            // acknowledge
                setTimer(0, pitFastCount) ;  // update PIT
                rateState = MEDIUM ;           // ... the new state
            }
            break ;
						
						
		case MEDIUM:
            if (pressedB1_ev) {                   // signal received
                pressedB1_ev = false ;            // acknowledge
                setTimer(0, pitMedCount) ;  // update PIT
                rateState = FAST ;           // ... the new state
            }
            break ;
						
		case FAST:  
            if (pressedB1_ev) {                   // signal received
                pressedB1_ev = false ;            // acknowledge
                setTimer(0, pitSlowCount) ;  // update PIT
                rateState = SLOW ;           // ... the new state
            }
            break ;
  }
}

/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main (void) {
    configureLEDforPWM() ;            // Configure LEDs for PWM control
    configureButtons(B1MASK, false) ; // ConfigureButtons B1 for polling
	configureButtons(B2MASK, false) ;
    configurePIT(0) ;        // configure PIT channel 0
    configureTPMClock() ;    // clocks to all TPM modules
    configureTPM0forPWM() ;  // configure PWM on TPM0 (blue LED)
    configureTPM2forPWM() ;  // configure PWM on TPM2 (red, green LEDs)
   
    Init_SysTick(1000) ;  // initialse SysTick every 1 ms

    // start everything
    setLEDBrightness(Red, 0) ;
    setLEDBrightness(Green, 0) ;
    setLEDBrightness(Blue, 0) ;

    initPollB1Task() ;       // initialise task state
    initToggleRateTask() ;   // initialise task state
    startTimer(0) ;
    waitSysTickCounter(10) ;  // initialise delay counter
    while (1) {      // this runs forever
        pollB1Task() ;  // Generate signals for a simulated button B1
		pollB2Task();   // Generate signals for a simulated button B2
        toggleRateTask();    // Toggle LED update rate on every press signal
        // delay
        waitSysTickCounter(10) ;  // cycle every 10 ms 
    }
}

