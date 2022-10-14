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
*/

#include <Wire.h>           // for I2C
#include<avr/wdt.h>         // for watchdog timer
#include "ATTiny88_pins.h"  // the "" format looks for this file in the project directory

/// VERSION NUMBER ///
#define VERSION_NUMBER 0x12   // rev level of this code

// ADCs used to read battery voltages differentially
#define VCC_SENSE   ADC2    // ADC to read AC_IN line in CB (5V => AD2 of 0.90 V)

// Timing constants
// By experiment, the clock is running slow by factor of 8 (8 MHz clock with scale down of 8)
//  so define all time constants are expressed in 8 msec lsb
//  Note that this factor does NOT apply to the watchdog timer (WDT) settings

// minimum delay around the main loop... (25 => 200 msec)
//  NOTE: don't make this too long! We have a 1 second watchdog running!
#define LOOP_DELAY       25   

// Time variables to handle next decision time
unsigned long currentTime;
const int dwellTime = 1500;   // 1500 == 12 secs nominal battery charging time (sec) (each cycle)
unsigned long targetTime;

// Misc global variables
int Charger_Vcc = 0;    // AC_IN voltage measurement
int CM4_active = 0;     // heartbeat to know if CM4 is active
int currBat = 0;        // battery currently used to power the system
int batCount = 0;       // number of batteries currently connected
int battPresent[4] = {1,0,0,0};     // set to "1" if battery present
int batVoltage[4] = {0, 0, 0, 0};   // Battery voltages sent from CM4; LSB = 16 mV
int offDelay = 30;      // delay from AXP209 signal to power off (secs)
int lastPB0 = LOW;        // set up to find falling edge of PB0
unsigned long endPowerUpHoldTime;
int poweringDown = false;

// Constants used to select battEna and battSense FETs by reference (>= battery board 1.7.3)
int battEna[4]   = {PA0, PA1, PA2, PA3};    // FETs to use the selected battery   
int battSense[4] = {PD0, PD1, PD2, PD3};    // sense for battery present (HIGH)

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
int heartbeat = 0;

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
  
  analogReference(DEFAULT);    // reference = Vcc => 1023 (0x3FF) counts

  // set all sense lines to INPUTS 
  // A reading of HIGH indicates battery present, a LOW indicates battery missing (or backwards)
  for (int n=0; n<4; n++){
    pinMode(battSense[n], INPUT);
  }

  pinMode(PC3,OUTPUT);      // create heartbeat

  pinMode(PB0, INPUT);      // monitor AXP209 EXT_ENA signal
  pinMode(PB1, INPUT);      // override shutdown signal on falling PB0
  pinMode(PB2, OUTPUT);
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
 * The main loop... at intervals of dwellTime, check for battery presence on all slots, 
 * determine the next battery to be used, connect that battery, reset the time.
 */
void loop() 
{
  int nextBat;

  heartbeat += 1;
  heartbeat = heartbeat %2;
  digitalWrite(PC3, heartbeat);

  check_powerDown_request();

  checkForBatteries();              // test for presence of all batteries at top of loop and clear batVoltage[] for missing bat
  
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


/*
 *  check_powerDown_request()
 *  This function monitors PB0 for falling edge and if found, will output PB1 HIGH
 *   for period of time equal to variable "offDelay".
 */

void check_powerDown_request(){
  int PB0_state;

  PB0_state = digitalRead(PB0);
  currentTime = millis();

  if (poweringDown){      // in a powering down sequence
    // check timer
    if (currentTime >= endPowerUpHoldTime){
      pinMode(PB1, INPUT); // release override shutdown and let RC begin 
      poweringDown = false;      
    }
  }
  // Test to see if AXP209 requesting power down
  else {
    if ((lastPB0 == HIGH) && (PB0_state == LOW)) {
      // AXP209 calling for shutdown
      pinMode(PB1, OUTPUT);
      digitalWrite(PB1,HIGH);
      poweringDown = true; 
      endPowerUpHoldTime = currentTime + 125*offDelay;   // 1000 / clock scaling (8) to get sec -> msec   
    }
  }
  lastPB0 = PB0_state;
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
      CM4_active = 0;          // reset heartbeat
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
  Charger_Vcc = (analogRead(VCC_SENSE));  // 127 calculates to charger == 3.0 V
  if (Charger_Vcc > 127) {
     return (true);
  }
  return (false);
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
void batterySelect (int batt)
{
  noInterrupts();                     // turn off interrupts while we change FETs
  if (batt != currBat) {
    digitalWrite (battEna[batt], LOW);     //  turn on new battery
    digitalWrite (battEna[currBat], HIGH); //   then turn off currBat (70 usec overlap)

  }
  // else we are not changing batteries so leave status quo
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

    case 0x29:                  // Charger Voltage Low 8 bits... 
      sendData = Charger_Vcc & 0xFF;  // Ref (approx): for AC_IN == 5V, Vcc => 0x0D4
      break;
    case 0x2A:                  // Charger Voltage Upper 8 bits
      sendData = Charger_Vcc >> 8;
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
