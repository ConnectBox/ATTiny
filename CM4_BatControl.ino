/* Arduino code for  ATTiny88 control of ConnectBox battery pack 
*
*  Created 07/15/21 by DorJamJr
*  Revised 11/17/21 by DorJamJr
*      Changed charge discharge algorithm to run fixed dwell time using highest battery
*      (discharge mode) or lowest battery (charge mode)
*  Revised 01/12/22 by DorJamJr
*      Changed the control algorithm and general functionality by removing the ADC battery
*      voltage read from the ATTiny. Instead, we will use the CM4 to request the voltage of
*      the current battery and then have the CM4 write that voltage to registers 0x21 through
*      0x24 (battery 1 through battery 4, respectively). Thus, all battery voltage 
*      measurements will be by the AXP209 and will bring consistency. Note that the voltages are 
*      actually stored in the batVoltage[] array (lsb = 16 mV) and the registers 0x21 - 0x24
*      are mearly access points of i2c communication to accomplish the writing and reading of the 
*      array values.
*  
*  The battery controller and batteries act as a self contained module.
*  This code manages both battery charging and discharging without the
*  need for control by the CM4. 
*  This code services requests for information from the CM4 re: battery voltages,
*  charge levels and code version.
*  
*  Version 4:
*   A new battery voltage sense scheme is introduced. This was requeired since the original
*   calculations based on the differential readings (ADC0 and ADC1) fails due to the ground
*   offsets introduced by the use of a common Battery + rail and switching in of alternate 
*   batteries as a power source, leading to, in effect, a moving ground for measurement 
*   purposes, and the subsequent pushing of ADC0 and ADC1 measurements outside the allowed
*   range of GND to 1.1 V (the F.S. reference voltage).
*   
* Version 6:  
*   Built for Battery Board version 1.7.3 and beyond. Changed the batEna[] lines to PA0, PA1,
*   PA2 and PA3. This required adding definitions of pin numbers to ATTiny88_pins.h which
*   were 23, 24, 25, and 26 for PA0, PA1, PA2 and PA3, respectively. 
*   
* Version 9:  
*   Major change in logic. We will use the BTx_RD signals to read the voltage on the drain
*   of the battery polarity sense P-FET for each battery. If the voltage is HIGH, then
*   there is a battery in that position. If the voltage is LOW then there is no 
*   battery (or a reversed battery) in that position. We will use that status to cycle
*   through the batteries. When a battery is on, the CM4 will read the battery voltage
*   and return it to an ATTiny register and that voltage will be used by the ATTiny to
*   make decisions as to which battery is to be used. The CM4 code will need to be
*   modified to provide battery voltages to the ATTiny.
*   
* Version A:  
*   Cleanup of code...
*   Battery Logic: see description of function next_bat()
*   
* Version B:  
*   Add watchdog timer code; change loop time to 800 msec (was 80 msec)
*   
* Version C:  
*   Fix watchdog timer code.
*   
* Version D:  
*   Add code to force 8 MHz clock prescale of 8; change battery swap to have 
*   new turn on before old turn off to prevent premature shutdown by AXP209
*   when input voltage dips during swap.
*   
* Version 10:  
*    Minor cleanup from version D. Update calls to batVoltage[] array to use lsb = 16mv
*    (so 8 bits max).
*    
* Version 12:   
*   Add power down extension. Sense when AXP209 calls for power down (INPUT PB0)
*   LOW for two cycles of loop. When PB0 power down request detected, turn PB1 into OUTPUT and 
*   hold high for time set by variable, "offDelay" (default 30 seconds). Allow user to 
*   set variable with I2C write to address 0x36.
*   
* Version 13:  
*   For use with hardware HAT_CM4 version 1.8.3
*   This adds the following functionality: 
*    - sense when AXP209 turns on 5V (PD4)
*    - hardware OR of 5V enable via ATTiny PD6 to hold power on independent of AXP209 (PD6)
*    - sense when AXP209 calls for power off (PD4)
*    - signal CM4 power going down (PD5 drop low)
*    - wait a specified time then drop PD6 to power 5V down
*    - sleep when both 5V and AC_IN are down
*    - wake when AXP209 calls for power OR AC_IN goes high
*    - parallel batteries of like voltage when ATTiny not in sleep mode to
*       minimize losses due to high current through internal battery resistances.
*       
* Version 14:     
*   Update and improve ver 13 to allow AXP209 primary control of power with ATTiny being
*    used to override both power up and power down status, as follows:
*             
*           setup(){
*              pinMode(VCC_OVR,OUTPUT);
*              digitalWrite(VCC_OVR,LOW);  // force hold off
*              powerOn = false;
*           }
*             
*           check_powerUp_request(){
*             if (powerOn==true){ return;}
*             if (AXP_EXTEN == LOW){ return;}  // no request for change so just return
*             else{                            // something is calling for power on
*               if (PB_IN == HIGH){            // PB1 is NOT pushed so continue to hold off
*                 return;
*               }  
*               // make sure low for 2 passes (noise) then...
*                 digitalWrite(VCC_OVR,HIGH);  // hold on
*                 powerOn = true;
*               }  
*             }
*          }   
*          
*          check_powerDown_request(){
*            if (powerOn == false){return;}   // we are off so no action 
*                                             //   we will handle check for AXP_EXTEN == HIGH with powerOn == false in sleepy()
*            if (AXP_EXTEN == HIGH){return;}  // no request for change so just return
*            // else, we are powered but AXP wants to be off so PB1 has been pushed or V is below critical
*            // set timer to wait 30 seconds and then...
*                digitalWrite(VCC_OVR, LOW);          
*                powerOn = false;
*           }     
*           
*           check_sleepy(){
*           // prior to going to sleep, check for special case of (AXP_EXTEN == HIGH) and (powerOn == false)
*           //  which we get to when 5V is off and charger is off, but charger attach caused AXP_EXTEN
*           //  to go HIGH. To fix that we need to hold the PB_IN low until AXP_EXTEN goes low. Then we
*           //  can set the ATTiny outputs to inputs and go to sleep           
*           
*           // Coming out of sleep, we set VCC_OVR to OUTPUT and LOW; powerOn = false;
*           }  
*               
*               
*   
*/

#include <Wire.h>           // for I2C
#include <avr/wdt.h>         // for watchdog timer
#include <avr/sleep.h>
#include <avr/interrupt.h>

#include "ATTiny88_pins.h"  // the "" format looks for this file in the project directory

/// VERSION NUMBER ///
#define VERSION_NUMBER 0x14   // rev level of this code

// I/O to sense presence of charger
#define AC_PRESENT   PC2    // digital read AC_IN line

// Timing constants
// By experiment, the clock is running slow by factor of 8 (8 MHz clock with scale down of 8)
//  so define all time constants are expressed in 8 msec lsb
//  Note that this factor does NOT apply to the watchdog timer (WDT) settings

// DELAY TIMES
//  we have a /8 clock so 125 counts/sec
// minimum delay around the main loop... (25 => 200 msec)
#define LOOP_DELAY      25      // 200 msec (not too long! remember 1 second watchdog!)
int ps_shutdown_delay = 3750;   // 30 sec (make int so there is an opportunity to change it programatically
const int dwellTime = 1500;     // 1500 == 12 secs nominal battery charging time (sec) (each cycle)
int ps_powerup_delay = 250;     // 2 sec

// Time variables to handle next decision time
unsigned long currentTime;
unsigned long targetTime;

// Misc global variables
int CM4_active = 0;     // heartbeat to know if CM4 is active
int currBat = 0;        // battery currently used to power the system
int batCount = 0;       // number of batteries currently connected
int battPresent[4] = {1,0,0,0};     // set to "1" if battery present
int batVoltage[4] = {0, 0, 0, 0};   // Battery voltages sent from CM4; LSB = 16 mV
int batGroup[4] = {0,0,0,0};        // if element is non-zero, indicates bat is part of a battery group
int offDelay = 30;      // delay from AXP209 signal to power off (secs)

unsigned long endPowerUpHoldTime;
int poweringDown = false;
int powerOn = false;                        // indicator that power is on (will be set/cleared as I/O PWR_CTL is written)

// Constants used to select battEna and battSense FETs by reference (>= battery board 1.7.3)
int battEna[4]   = {PA0, PA1, PA2, PA3};    // FETs to use the selected battery   
int battSense[4] = {PD0, PD1, PD2, PD3};    // sense for battery present (HIGH)

// Declarations of I/O
#define HEARTBEAT      PC3
#define AXP_EXTEN      PD4       // AXP209 EXTEN pin (formerly used to directly control 5V)
#define PWR_IRQ        PD5       // signal to CM4 that power down is in process
#define VCC_OVR        PD6       // signal controlling power to 5V converter


/* I2C stuff */
#define I2C_ADDRESS 0x14      // chosen address for this code
byte rcvData;                 // command sent from master
int multi[4] = {0,0,0,0};     // used in handling multi byte i2c communication

// debug stuff
#define CMD_ERROR 0xE8        // constant used in debugging code
int testIncrement = 0;
int loopIncrement = 0;
int probe = 0xee;
int probe1 = 0xe1;
int probe2 = 0xe2;
int probe3 = 0xe3;
int heart = 0;


/* ----------------- SETUP ------------------------------ */

void setup() 
{
  MCUSR = 0;      // clear all interrupt flags (the step you don't usually find in examples)
  wdt_reset();
  cli();          // clear interrupts?
  wdt_disable();  //Disable WDT
  
  // clock prescaler - 0000 = 8 MHZ, 0001 = 4 MHZ, 0010 = 2 MHZ, 0011 = 1 MHZ
  CLKPR = (1<<CLKPCE);  // enable prescaler write
  CLKPR = ((1<<CLKPS1) | (1<<CLKPS0));    // set to 1 MHz
   
  sei();          // enable interrupts

  // housekeeping before enabling the WDT
  // delay (1000);  // why??
  
  analogReference(DEFAULT);       // reference = Vcc => 1023 (0x3FF) counts
  pinMode(HEARTBEAT,OUTPUT);      // create heartbeat
  
  // 5V control signals
  pinMode(AXP_EXTEN,INPUT);       // monitor AXP EXTEN pin for 5V power down control
  pinMode(VCC_OVR,INPUT);         // control signal for 5V on/off
  pinMode(PWR_IRQ,OUTPUT);        // indicator signal for 5V going down
  digitalWrite(PWR_IRQ,HIGH);     // initialize

  // set all sense lines to INPUTS 
  // A reading of HIGH indicates battery present, a LOW indicates battery missing (or backwards)
  for (int n=0; n<4; n++){
    pinMode(battSense[n], INPUT);
  }

  // Set battEna control for Battery 1 (index == 0) to OUTPUT and ON
  //  (This battery is the default battery)
  pinMode(battEna[0], OUTPUT);
  digitalWrite(battEna[0], LOW);
  currBat = 0;

  // now we will enable the remaining battery FETs as OUTPUTs and HIGH (off)
  for (int n = 1; n<4; n++){
    pinMode(battEna[n], OUTPUT);
    digitalWrite(battEna[n], HIGH);
  }

  // Capture current time and calculate the time of the next examination of battery voltages
  currentTime = millis();
  targetTime = currentTime + dwellTime;

  // ADD SETUP FOR I2C
  Wire.begin (I2C_ADDRESS);     // set I2C address
  Wire.onReceive(DataReceive);  // I2C Handlers 
  Wire.onRequest(DataRequest);

  wdt_enable(WDTO_1S);      // enable WDT with a (real) timeout of 1 sec
}


/*
 * ------------  MAIN LOOP ----------------
 * The main loop... at intervals of dwellTime, check for battery presence on all slots, 
 * determine the next battery to be used, connect that battery, reset the time.
 */
 
void loop() 
{
  int nextBat;

  heartbeat();
  check_sleepy();                   // if power is down and no charger, put ATTiny to sleep 
  
  checkForBatteries();              // test for presence of all batteries at top of loop and clear batVoltage[] for missing bat
  
  check_powerDown_request();        // test and begin power down when AXP EXTEN pin is low
  check_powerUp_request();          // test for power up (check AXP_EXTENA for HIGH indicating power on)
 
  currentTime = millis();           // check if our dwell time (12 sec) is done and we need to change batteries
  if (currentTime > targetTime) {   // finished this charge/discharge cycle for current battery
    nextBat = next_bat();           // find next battery we want to use
    batterySelect (nextBat);        // turn on FET for "nextBat" and make it "currBat"
    targetTime = currentTime + dwellTime;    // set target time for next battery check
  }  

  loopIncrement += 1;     // debug...
  delay (LOOP_DELAY);     // slow down loop ...

  wdt_reset();          // reset the WDT... if we fail to get here for 1 seconds, we will restart the ATTiny
}




/* ---------------- SUBROUTINES --------------------------  */

/*
 * heartbeat()
 *  Simple heartbeat on output pin to let us know the ATTiny is looping
 */
void heartbeat(){
  heart += 1;
  heart = heart %8;       // a flicker is sufficient and uses less power
  if (heart == 0){
    digitalWrite(HEARTBEAT, HIGH);
  }
  else{
    digitalWrite(HEARTBEAT, LOW);
  }
 }

/*
 * check_powerDown_request()
 *   Read the AXP_EXTEN line. 
 *     HIGH: no power down request... just return
 *     LOW:  AXP209 sensed power down (either user via pushbutton or supply voltage too low)
 *            so begin power down timer. When timeout, take VCC_OVR low.
 */
void check_powerDown_request(){
  if (powerOn == false){
    return;       // we aren't powered on, so no need to check about powering off... just return    
  }
  if (digitalRead(AXP_EXTEN) == HIGH){
    return;       // AXP_EXTEN high means AXP209 is not calling for a power down... just return
  }
  
  // If we get here, 5V is on and AXP_EXTEN was LOW, so we need to start (or continue) power down countdown
  if (poweringDown == false){       // just entering the power down sequence
    poweringDown = true;
    endPowerUpHoldTime = millis() + ps_shutdown_delay;    // 30 sec
    digitalWrite(PWR_IRQ,LOW);      // signal CM4 we are going down
  }
  
  // else we are in the countdown... check it to see if we are exhausted
  else if (millis() >= endPowerUpHoldTime){  // exhausted             
    digitalWrite(VCC_OVR,LOW);         // turn off the 5V supply
    powerOn = false;
    poweringDown = false;
    digitalWrite(PWR_IRQ,HIGH);      // reset
    pinMode(VCC_OVR,INPUT);          // release for AXP209 to control next power up
  }
}

/*
 * check_powerUp_request()
 *   If power is off, read the AXP_EXTENA line to see if AXP209 calling for power on
 *   If power is on, just return
 */
 void check_powerUp_request(){
  if (powerOn == true){
    return;
  }
  // if we get here, power was off, so see if AXP209 is turning 5V on
  if (digitalRead(AXP_EXTEN) == LOW){
    return;       //  no request to turn on
  }
  // AXP209 wants 5V
  pinMode(VCC_OVR,OUTPUT);   // set over ride pin to OUTPUT
  digitalWrite(VCC_OVR,HIGH);  //  and set HIGH to hold 5V on
  powerOn = true;
 }

/*
 * check_sleepy()
 *  If power is down and we don't have a charger connected, put ATTiny in sleep mode
 *  Note that we want to put all output pins into input mode to save power during sleep
 *   (and then will need to restore them on waking up!)
 */
void check_sleepy(){
  if (powerOn == true){
    return;         // don't sleep if power is on
  }
  if (AC_on() == true){
    return;         // don't sleep if AC is connected
  }
  
  // if we get here, we have neither charging or discharging of batteries, so we can sleep...
  wdt_disable();                // turn off wdt while in sleep
  batterySelect (0);             // select battery 1 before putting selector I/O's into INPUT mode
  // Put all outputs into input mode...
  pinMode(VCC_OVR,INPUT);         // control signal for 5V on/off
  pinMode(PWR_IRQ,INPUT);        // indicator signal for 5V going down
  for (int n = 0; n<4; n++){
    pinMode(battEna[n], INPUT);
  }
  pinMode(HEARTBEAT,INPUT);      // create heartbeat
  
  // do we need to set I2C pins to INPUT state?

  // set our wakeup call to come from PC2 (AC_IN sense) or PD4 (AXP_EXTEN)
  sei();                                  // Global enable all interrupts... same as SREG set bit 7
  // enable interrupt on PD4 (PCINT20) ... AXP_EXTEN
  PCICR |= (1<<PCIE2);                    // enable PCIE2 (PCINT23..16) (See ATTiny88 section 9.2 and 9.3.4)
  PCMSK2 |= (1<<PCINT20);                 //  enable PCINT20 from within the PCIE2 group (see section 9.3.7)
  // enable interrupt on PC2 (PCINT10) ... charger connected (AC_IN)
  PCICR |= (1<<PCIE1);                    // enable PCIE1 (PCINT15..8) (See ATTiny88 section 9.2 and 9.3.4)
  PCMSK1 |= (1<<PCINT10);                 //  enable PCINT10 from within the PCIE2 group (see section 9.3.8)

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);    // set sleep mode (register SMCR bits SM1=1, SM0=0) (see 7.4.1)
  sleep_enable();                         // enable sleep (register SMCR bit SE=1) (see 7.4.1)
  sleep_mode();                           // actually goes to sleep here... zzzz
//  ZZ
//  ZZ
//  ZZ
// WE ARE HERE AT WAKEUP
//  Need to do some housekeeping (similar to setup() function since CM4 will power up (NOT come out of a sleep mode)
  cli();          // clear interrupts ... (same as SREG set bit 7)?
  MCUSR = 0;      // clear MCU Status Register (clears the flag indicating the wdt was the interrupt source 
                  //   (the step you don't usually find in examples)
  PCIFR = 0;      // clear the PC interrupt flag register
  PCICR = 0;      // clear selection of interrupt pins
  SMCR = 0;       // disable sleep
  
  wdt_reset();

  // after wakeup, we need to set up outputs correctly before turning on power
  pinMode(HEARTBEAT,OUTPUT);      // re-enable heartbeat
  pinMode(PWR_IRQ,OUTPUT);        // indicator signal for 5V going down
  digitalWrite(PWR_IRQ,HIGH);     // initialize HIGH (not signalling power going down)
  // now we will enable the remaining battery FETs as OUTPUTs and HIGH (off)
  pinMode(battEna[0], OUTPUT);
  digitalWrite(battEna[0], LOW);  // turn on battery 1
  for (int n = 1; n<4; n++){
    pinMode(battEna[n], OUTPUT);
    digitalWrite(battEna[n], HIGH);  // batteries 2-4 not enabled (initial condition)
  }

  // clear out the batVoltage[] array and let CM4 refill it
  for (int n=0;n<4;n++){
    batVoltage[n] = 0;
  }
  
  sei();                    // enable interrupts
  wdt_enable(WDTO_1S);      // enable WDT with a (real) timeout of 1 sec
}

 
/*
 * checkForBatteries()
 * This function will check all battery positions for presence of battery
 */
void checkForBatteries(){
  int n;
  
  batCount = 0;     // initialize
  for (n=0; n<4; n++){
    if (digitalRead(battSense[n]) == HIGH){
      battPresent[n] = true;
      batCount ++;
    }
    else{
      battPresent[n] = false;
      batVoltage[n] = 0;    // clear the voltage for a battery not present
    }
  }
}  // end of checkForBatteries()


/*
 * next_bat()
 *   Determine the next battery to select based on the following logic:
 *   - IF: any battPresent[] == true with batVoltage[] == 0, select that battery -> LOOP
 *   - ELSE:
 *    - IF charging:
 *     - IF CM4 running:
 *       - Choose battery with lowest voltage -> LOOP
 *     - IF CM4 not running:  
 *       - Choose next available battery -> LOOP
 *   - IF not charging:    
 *     - Choose battery with highest voltage -> LOOP
 */
int next_bat() 
{
  int n, m;
  int nextBat;
  int highest_voltage = 0;
  int lowest_voltage = 0xFF;
  
   // Initial battery reading... pick next battery which is present but has no voltage reading...
  for (n=0; n<4; n++){
    m = (n + 1 + currBat) % 4;    // start with battery AFTER current battery and look through all 4 battery positions
    if (battPresent[m] && (batVoltage[m] == 0)){    // special case... battery present but not yet read by AXP209
        nextBat = m;                                //  so FORCE this battery
        return nextBat; 
    } 
  } 
    
  // all batteries have had their voltage read... proceed with normal selection
  // check to see if we are charging
  if (AC_on()) {               // charging... 
    
    // see if CM4 is active (and so we have active voltage reading of the batteries)
    if (CM4_active == 1) {     //   with CM4 active, batVoltage[] readings are valid... so pick the lowest voltage (valid) battery
      CM4_active = 0;          // reset CM4 heartbeat
      for (n=0; n<4; n++) {
        if ((batVoltage[n] < lowest_voltage) && (batVoltage[n] > 0xC1)) {  // batt > 3100 mV to be selected
          nextBat = n;
          lowest_voltage = batVoltage[n];
        }  
      }
    }
    
    // we are charging but CM4 powered down, so choose next available battery
    else {        
      for (n=0; n<4; n++){
        m = (n + 1 + currBat) % 4;    // choose next battery after current battery
        if (battPresent[m]){          
          nextBat = m; 
          break;                              
        } 
      }      
    }
  }

  // we are NOT charging, but since we are here, the CM4 is powered
  else {                              // discharging... so pick the highest voltage battery
    for (n=0; n<4; n++){
      if (batVoltage[n] > highest_voltage) {
        nextBat = n;
        highest_voltage = batVoltage[n];
      }
    }
  } 
   
 return nextBat;  
}


/*
 * AC_on()
 * Determine if AC_IN line is powered (ie, is charger connected) and return TRUE/FALSE
 * Vref = Vcc (about 4.35V); so LSB = 4.25mV; 4v *2.2K / (2.2K + 10K) = 0.72v;  So for 3.0V count = 127
 */
bool AC_on()       // Verified
{  
  if (digitalRead(AC_PRESENT) == HIGH){
    return true;
  }
  else{
    return false;
  }
}


/*
 * batterySelect (int batt)
 * Turn on FET for select battery position (passing in the nextBat reference)
 * Interrupts are suspended just before the switch over and enabled just after to 
 *  insure that we aren't servicing an interrupt while both FETs are on
 * Note that the digitalWrite(currBat, HIGH) and digitalWrite(nextBat,LOW) occur
 *  intentionally request turning OFF the current battery before turning ON 
 *  the next battery. In practice, there is still a slight overlap (usec's)
 *  of the switching due to capacitive effects of FET gates.
 */
/*
 * FEATURE ADD:
 *  We want to enable all batteries that are within 2 counts (32 mV) of the selected 
 *   battery. Perhaps we can do that here? I don't think we need to be concerned with
 *   charging/discharging (we already looked at that when we chose this battery) so
 *   just check for battery readings within 2 counts of the selected battery??
 *   How will the battery readings get properly saved with CM4 doing one reading of a 
 *   group of batteries? Perhaps a new variable batGroup[] with the 4 elements representing
 *   the currently selected group? Set that var here (when we turn them on) and then
 *   take the group into account when we get a write to I2C register 0x21 - 0x24??
 */
 
void batterySelect (int batt)
{
  noInterrupts();                     // turn off interrupts while we change FETs
  if (batt != currBat) {
    digitalWrite (battEna[batt], LOW);     //  turn on new battery
    batGroup[batt] = 1;             // add new selection to batGroup[] (it may be the only one)
    digitalWrite (battEna[currBat], HIGH); //   then turn off currBat (70 usec overlap)
    batGroup[currBat] = 0;             // remove previous selection from batGroup[] (it may be added again)
  }
  // else we are not changing batteries so leave status quo
  for (int n=1; n<4; n++){
    if (abs(batVoltage[(batt+n)%4] - batVoltage[batt]) <= 2){
      // turn on FET and add to batGroup[] any battery within 2 counts of same voltage (32mV) 
      batGroup[n] = 1;                  // add to group and
      digitalWrite(battEna[n], LOW);    //  turn on FET
    }
    else{
      batGroup[n] = 0;                  // remove from group and
      digitalWrite(battEna[n], HIGH);   //  turn off FET
    }
  }
  interrupts();                       // turn interrupts back on
  currBat = batt;                     // the FET is turned on, so now "nextBat" becomes "currBat"
}



/* I2C communication handlers 
* 
* For the slave, data transfer is handled by interrupt servicing. We just need two 
*  global values, rcvData and sendData which are accessed duing I2C communication.
*  
*  The DataReceive() function handles one (or more) bytes of data from the master.
*  The interpretation is up to us. They could be data requests, commands, etc.
*  We have both reads (one byte communication) and writes (two byte communication) 
*  implemented in this code. 
*/


/* DataReceive()
 * 
 * Services an I2C communication from the master. For one byte commands ("read request") the 
 *  sent data is saved as rcvData (the register the master wants to read) ; for two byte 
 *  commands ("write request") the sent data is saved as rcvReg (the regist to write) and 
 *  rcvVal (the value the master wants written to register rcvReg). Note that this code must 
 *  explictly handle the write requests for any registers for which the write function is
 *  to be accepted. A write to a non-enabled write register will be ignored. At the moment, only
 *  the battery voltage registers (0x21, 0x22, 0x23 and 0x24) are enabled for write.
 *  
 *  Battery Voltage Math: we need to have CM4 send battery voltages in mv up to 4100 (0x1004)
 *  If we limit our voltages to 4096mV, then the CM4 can divide the mv of a battery by 16, ship that
 *  number as the battery voltage, then on this end we can multiply by 16 to get the 
 *  battery voltage in mv. (Actually 255*16 = 4080 so if actual V is greater, we just send 0xFF (255))
 *  Example: Battery V = 3700 mv, we do 3700 / 16 =231 (0xE7) so we send 0xE7 
 *  SO... Battery voltage stored in mV with resolution of 16 mV lets us use a single byte for storage.  
 */
 
void DataReceive(int numBytes)
{
  int n;
  int rcvReg, rcvVal;

  if (numBytes == 1){    // this is a normal request for a read
     while(Wire.available())  {
       rcvData = Wire.read();
    }
  }

  if (numBytes == 2){         // this is a register write
    rcvReg = Wire.read();     // the register to write
    multi[0] = rcvReg;
    rcvVal = Wire.read();     // the value to write
    multi[1]= rcvVal;
    rcvData = rcvReg;         // set up for possible request to retrieve the value written

  // Handle the battery voltage write by putting the value to the appropriate batVoltage array member
    if ((rcvReg >= 0x21) && (rcvReg <= 0x24)){
      batVoltage[rcvReg - 0x21] = rcvVal;   // use value sent (lsb = 16mV)
    //  and if part of batGroup[], fill in batVoltage[] for all members of group
      if (batGroup[rcvReg - 0x21] == 1){  
        for (int n=0; n<4; n++){        
          if (batGroup[n] == 1){
            batVoltage[n] = rcvVal;
          }
        }
      }
    }

    
    if (rcvReg == 0x36){      // power down delay (secs)
      offDelay = rcvVal;
    }
  }       // else... we have a normal request from the CM4
}

/*
 * The DataRequest() function sends data back to the master, usually in response to
 *  a request for data from the master. So here we will interpret the rcvData, formulate
 *  a proper response and call the Wire.write function.
 *  
 *  Note: all voltage responses in mV
 *  
 *  Current command table: 
 *  COMMAND        
 *   0x10    Request for revision of this code
 *   
 *   0x21    Read / Write battery voltage of Battery 1
 *   0x22    Read / Write battery voltage of Battery 2
 *   0x23    Read / Write battery voltage of Battery 3
 *   0x24    Read / Write battery voltage of Battery 4
 *   
 *   0x29    Read voltage of Charger 5V line (LSB) (mV)
 *   0x2A    Read voltage of Charger 5V line (USB) (mV)
 *   
 *   0x30    Read number of batteries attached
 *   0x31    Read number of the battery currently in use [1 -> 4]
 *   0x32    Read battery positions populated (binary, bit 0-3 => battery 1 -> 4)
 *   
 *   0x40   Heartbeat... set to 1 each time CM4 reads register 0x31 
 *   
 *   0x50 and higher used for testing and debug 
 *  
 */
 
void DataRequest()
{
  int sendData;
  int n;
  
  switch(rcvData) {
    case 0x10:                  // Version Number
      sendData = VERSION_NUMBER;
      break;

    case 0x21:    // single byte = (battery voltage battery 1) / 16
      sendData = batVoltage[0];
      break;  

    case 0x22:    // single byte = (battery voltage battery 2) / 16
      sendData = batVoltage[1];
      break;  

    case 0x23:    // single byte = (battery voltage battery 3) / 16
      sendData = batVoltage[2];
      break;  

    case 0x24:    // single byte = (battery voltage battery 4) / 16
      sendData = batVoltage[3];
      break;  


    case 0x30:                  // Battery Count
      sendData = batCount;
      break;   
        
    case 0x31:                  // Current Battery
      sendData = currBat + 1;   // (battery number is zero based)
      CM4_active = 1;           // heartbeat... set to 1 each time CM4 does a read
      break;

    case 0x32:    // sendData = binary representation of battPresent array; bit 0 == battPresent[0], etc
      sendData = 0;
      for (n=0; n<4; n++){
            sendData *= 2;    // batCoded shift left
            if (battPresent[3-n] == true){
              sendData += 1;
            }
      }
      break;

    case 0x36:        // power down delay register
      sendData = offDelay;    // secs
      break;

    case 0x37:        // report time remaining in power up hold
      if (poweringDown){
        sendData = (endPowerUpHoldTime - millis())/125;        // 1000 msec/sec / 8 (clock scaling)
      }
      else{
        sendData = 0xFF;
      }
      break;   

    case 0x40:                  // heartbeat
      sendData = CM4_active;
      break;  

// debug I2C with these calls
    case 0x50:
      testIncrement += 1;
      sendData = testIncrement;
      break;  
    case 0x51:
      sendData = loopIncrement;
      break;  
    case 0x52:
      sendData = dwellTime;
      break; 

    case 0x53:
      sendData = probe1 & 0xFF;
      break;
    case 0x54:
      sendData = probe1 >> 8;
      break;
    case 0x55:
      sendData = probe2 & 0xFF;
      break;
    case 0x56:
      sendData = probe2 >> 8;
      break;
    case 0x57:
      sendData = probe3 & 0xFF;
      break;
    case 0x58:
      sendData = probe3 >> 8;
      break;

    // debug stuff
    case 0x61:
      sendData = multi[0];
      break;  
    case 0x62:
      sendData = multi[1];
      break;  
    case 0x63:
      sendData = multi[2];
      break;  
    case 0x64:
      sendData = multi[3];
      break;  
      
    case 0x70:    // current time
      sendData = currentTime & 0xff;
      break;
    case 0x71:
      sendData = currentTime >> 8;
      sendData = sendData & 0xff;
      break;
    case 0x72:
      sendData = currentTime >> 16;
      sendData = sendData & 0xff;
      break;
    case 0x73:
      sendData = currentTime >> 24;
      sendData = sendData & 0xff;
      break;
          
    case 0x74:    // dwell time
      sendData = dwellTime & 0xff;
      break;
    case 0x75:
      sendData = dwellTime >> 8;
      sendData = sendData & 0xff;
      break;
    case 0x76:
      sendData = dwellTime >> 16;
      sendData = sendData & 0xff;
      break;
    case 0x77:
      sendData = dwellTime >> 24;
      sendData = sendData & 0xff;
      break;
      
    case 0x78:    // Target time
      sendData = targetTime & 0xff;
      break;
    case 0x79:
      sendData = targetTime >> 8;
      sendData = sendData & 0xff;
      break;
    case 0x7A:
      sendData = targetTime >> 16;
      sendData = sendData & 0xff;
      break;
    case 0x7B:
      sendData = targetTime >> 24;
      sendData = sendData & 0xff;
      break;

    case 0x80:
      sendData = probe & 0xff;
      break;  
    case 0x81:
      sendData = probe >> 8;
      break;  
        
    default:                // Command Not Recognized
      sendData = CMD_ERROR;   
      break;
  }  
  Wire.write(sendData);
  //Wire.endTransmission();   // master doesn't like this... causes i2cdetect to fail...
  
}
