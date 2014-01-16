/*
  AC Light Control
  
  Ryan McLaughlin <ryanjmclaughlin@gmail.com>
  
  The hardware consists of an Triac to act as an A/C switch and
  an opto-isolator to give us a zero-crossing reference.
  The software uses two interrupts to control dimming of the light.
  The first is a hardware interrupt to detect the zero-cross of
  the AC sine wave, the second is software based and always running
  at 1/128 of the AC wave speed. After the zero-cross is detected
  the function check to make sure the proper dimming level has been
  reached and the light is turned on mid-wave, only providing
  partial current and therefore dimming our AC load.
  
  Thanks to http://www.andrewkilpatrick.org/blog/?page_id=445
    and http://www.hoelscher-hi.de/hendrik/english/dimmer.htm
 */

/*
  Modified by Mark Chester <mark@chesterfamily.org>
  
  to use the AC line frequency (half-period) as a reference point
  and fire the triacs based on that plus a count of dimming steps.
  Tracks the line frequency and adjusts accordingly.  Can set up to
  an estimated 512 steps of dimmer resolution.
  
*/

/*
  Modified by James Orland <ptd@caff.cx>
  
  Apply an adjustment to account for the shape of the sinewave - giving linear power adjustment
  Curve from:
  time=arccos(power-1)/Pi
  where t is delay between 0 and 1 and a power of 2 is 100%
  
  The sense of the input to the dimming is also reversed from the previous version by the power mapping - as the analogue input is controlling
  the power out - not the delay time. 1024 in from an analogue input will set maximum power.
*/

#include <TimerOne.h>                                  // http://www.arduino.cc/playground/Code/Timer1

// General
unsigned long int ZeroXTime[4] = {0,0,0,0};            // Timestamp in micros() of the zero crossing interrupts
unsigned long int DimStep;                             // How many micros() in each step of dimming
unsigned long int AvgPeriod;                           // The average line voltage period in micros()
unsigned long int PeriodResync = 3000;                 // Number of milliseconds between line freq measurements
unsigned long int ResetPeriod = PeriodResync;          // The timestamp in millis() when we will measure the period again
unsigned long int DimRes = 256;                        // How many steps of dimmer resolution
volatile unsigned long int DimStepCounter;             // For counting Timer1 interrupts
volatile unsigned long int FireTriac[4] = {0,0,0,0};   // When it's OK to fire the triacs, in counts of DimRes
volatile boolean zero_cross = 0;                       // Tels us we've crossed the zero line
byte TriacPin[4] = {3,5,6,7};                          // Which digital IO pins to use
byte PowerMap[256] = {
 255,245,241,237,235,232,230,228,226,224,223,221,220,218,217,215,214,213,211,210,209,208,207,205,204,203,202,201,200,199,198,197,
 196,195,194,193,192,192,191,190,189,188,187,186,185,185,184,183,182,181,181,180,179,178,177,177,176,175,174,174,173,172,171,171,
 170,169,168,168,167,166,165,165,164,163,163,162,161,161,160,159,158,158,157,156,156,155,154,154,153,152,152,151,150,150,149,148,
 148,147,146,146,145,144,144,143,143,142,141,141,140,139,139,138,137,137,136,135,135,134,134,133,132,132,131,130,130,129,128,128,
 127,127,126,125,125,124,123,123,122,121,121,120,120,119,118,118,117,116,116,115,114,114,113,112,112,111,111,110,109,109,108,107,
 107,106,105,105,104,103,103,102,101,101,100,99,99,98,97,97,96,95,94,94,93,92,92,91,90,90,89,88,87,87,86,85,84,84,83,82,81,81,80,
 79,78,78,77,76,75,74,74,73,72,71,70,70,69,68,67,66,65,64,63,63,62,61,60,59,58,57,56,55,54,53,52,51,50,48,47,46,45,44,42,41,40,38,
 37,35,34,32,31,29,27,25,23,20,18,14,10,0
};

void setup() {                                         // Begin setup
  Timer1.initialize(DimStep);                          // Start up the Timer1 timer
  attachInterrupt(0, zero_cross_detect, FALLING);      // Attach an Interupt to Pin 2 (interupt 0) for Zero Cross Detection
  pinMode(TriacPin[0], OUTPUT);                        // Set the Triac pin as output
  pinMode(TriacPin[1], OUTPUT);                        // Set the Triac pin as output
  pinMode(TriacPin[2], OUTPUT);                        // Set the Triac pin as output
  pinMode(TriacPin[3], OUTPUT);                        // Set the Triac pin as output
  measure_half_period();                               // Initially measure the half period
}                                                      // End setup

void measure_half_period() {
  zero_cross = 0;                                      // Clearing this here increases the accuracy of the measurement
  byte F = 0;                                          // Frequency counter counter  ;)
  while ( F < 4 ) {                                    // This loop takes 4 zero cross samples
    if ( zero_cross ) {                                // Only run if a zero cross is detected
      ZeroXTime[F] = micros();                         // Set the new current zero cross time in micros()
      zero_cross = 0;                                  // Reset zero_cross
      F++;                                             // Bump the counter for the next sample
    }
  }                                                    // Now we calc the length of each DimStep
  DimStep = (((ZeroXTime[1]-ZeroXTime[0]) + (ZeroXTime[2]-ZeroXTime[1]) + (ZeroXTime[3]-ZeroXTime[2])) / 3) / DimRes;
  Timer1.attachInterrupt(fire_triacs, DimStep);        // (Re)Associate fire_triacs() with the Timer1 interrupt and the latest DimStep period
  ResetPeriod = ResetPeriod + PeriodResync;            // Set the next time when we'll measure the half period again
}
  
void zero_cross_detect() {                             // function to be fired at the zero crossing
  zero_cross = 1;                                      // set a variable that's picked up later
  DimStepCounter = 0;                                  // Reset the step counter for the next round of triac firings
}

void fire_triac(int TriacNum) {
  if ( FireTriac[TriacNum] == DimStepCounter ) {              // Is it time to fire?
    
    if(FireTriac[TriacNum]!=DimRes)                           //When the power output is 0% never turn the triac on
      digitalWrite(TriacPin[TriacNum], HIGH);                 // Fire the Triac mid-phase

    if(FireTriac[TriacNum]!=DimRes && FireTriac[TriacNum]!=0) //Only bother to add a timing delay when both digitalWrites are active
      delayMicroseconds(2);

    if(FireTriac[TriacNum]!=0)                                //When the power output is 100% never turn the triac off
      digitalWrite(TriacPin[TriacNum], LOW);                  // Turn off the Triac gate (Triac will not turn off until next zero cross)

  }  
}

void fire_triacs() {                                   // Called every DimStep (Timer1 interrupt, checks FireTriac[n] and fires if it's time

  for(int i=0; i<4; i++)                               // Fire each of the triacs
    fire_triac(i);

  DimStepCounter++;                                    // This counter increments every time fire_triacs runs
}

void loop() {                                          // Main Loop
  if ( millis() >= ResetPeriod ) {                     // Measure the half period every PeriodResync milliseconds to prevent drift
    measure_half_period();
  }
  
  FireTriac[0] = PowerMap[(DimRes * analogRead(0)) / 1024];    // Read input and calc the next triac fire time
  FireTriac[1] = PowerMap[(DimRes * analogRead(1)) / 1024];    // Read input and calc the next triac fire time
  FireTriac[2] = PowerMap[(DimRes * analogRead(0)) / 1024];    // Read input and calc the next triac fire time
  FireTriac[3] = PowerMap[(DimRes * analogRead(1)) / 1024];    // Read input and calc the next triac fire time
DimStepCounter++;
 }
 




