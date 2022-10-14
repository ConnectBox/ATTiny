/* Arduino code for  ATTiny88 control of ConnectBox battery pack 
*
*  Initial creation 07/15/21 by DorJamJr
*    
*   Rev 0x16 by DorJamJr... 
*    - batteries in parallel now all show same voltage (registers 0x21 - 0x24) 
*    - clean up of registers for reading internal state of code. The following are now the
*       main registers of interest:
*         - 0x10 - revision of this code
*         - 0x21 thru 0x24 - voltage reading of batteries 1 - 4, respectively (lsb = 16mV)
*         - 0x30 - battery count
*         - 0x31 - currently selected battery (may be part of a parallel group)
*         - 0x32 - bitmap of batteries present (bit 0 = battery 1, bit 1 = batterry 2, etc)
*         - 0x33 - bitmap of batteries in the current group (ie, paralleled)
*         - 0x37 - time remaining in power down sequence (lsb = 8 msec)
*         - 0x40 - CM4 active heartbeat (set when CM4 writes voltage of current battery)
*         
*         - 0x50 and following - various probes to aid in debugging
*         
*    - NOT IMPLEMENTED in this rev:      
*      - The check_sleepy() function is disabled (requires further work) so the ATTiny will not
*         go to sleep when power is shut down (continues to draw power from the battery bank)
*         
*   Rev 0x17 by DorJamJr      
*     - Fix power up to occur only when PB is held for 2 seconds (so WON'T power on by just plugging in the charger)
*     - Add 2.5 Hz clock on pin 8 of ATTiny for debug
*     - Renew reading of battery using ADC0 but this time, we only read the CONNECTED battery (or bank) so we
*        don't have the problem of a non-ground reference
*     - Use ADC0 battery readings to modify charge times of batteries during unpowered charging (gives low charge  
*        batteries more charge time each loop). This calculation in made in function next_Bat() about line #486. 
*        The current algorithm is: 
*              dwellAdjust = constrain((avgVBatt - batVoltageATT[nextBat]) * 3,-80,200)
*        where dwellAdjust is the the percentage of dwellTime added to or subtracted from the nominal dwelllTime      
*        (so a dwellAdjust result of +35 will give a dwellTime 35% longer than the default)
*     - Added a software UART function which sends an ASCII stream out of PB1 (pin 13). The ASCII stream consists of 
*        "@@@" followed by the contents of the 4 batVoltageATT[] registers (separated by two spaces). This is useful
*        for reading these registers when the CM4 is unpowered. Connecting a logic analyzer to that pin and setting the 
*        analyzer for 9600 baud, lsb=first, no invert, pre-trigger 5% will give valid display of this data.
*        Note: This function takes about 20% of the memory, so a compile directive is included to control its inclusion
*        in the build. If it is not desired to include this function, find the following line and comment it out:
*            #define ENABLE_SERIAL_DEBUG
*     - Add I2C reading of the batVoltageATT[] array using registers 0x41 - 0x44 (lsb = 20mV)
*        Note that these readings are ONLY USED during the powered down and charging state of the machine, but they
*        are read whenever the ATTiny is powered.
*        
*   Rev 0x18 by DorJamJr     
*     - Change clock speed and all timing constants to 8 MHz to fix occassional i2c communication issues
*     
*   Rev 0x19 by DorJamJr  
*     - remove facility to re-read all bats when changing from charger to non-charger or vice versa
*     - add the "weld" concept... once batteries are put in parallel they are welded together unless a battery is
*        removed or replaced.
*     - add use of EEPROM to keep track of power state of CM4. Then use that info to restore power condition on wdt   
*        or reload of code by avrdude
*        
*   Rev 0x1A by DorJamJr     
*     - remove use of EEPROM  (broke i2c periodically !?)
*     - use state of AXP_EXTEN to determine power on state on reboot of ATTiny (wdt or avrdude)
*     
*   Rev 0x1B by DorJamJr   
*     - Remove "#define ENABLE_SERIAL_DEBUG" which was causing i2c reads to fail with a constant return value
*     
*   Rev 0x1C by DorJamJr  
*     - Move code block in check_powerDown_request() to ensure power down completes
*     - Reset batteries 2-4 voltage to 0 and show as "not present" at power on. This forces all battery FETs (except battery 1)
*        to turn OFF and allows the circuit/code to correctly determine battery presence. User must NOT change batteries
*        while power is on... possible fried circuit if low (or reversed) battery inserted in a welded position.
*     
*   Rev 0x1D by DorJamJr  
*     - change reset of batteries 2-4 to when power off occurs. This leaves logic in place to detect battery polarity
*        during power off, when the user might be changing batteries
*     - Add function force_batteryCycle() to reset battery arrays and force cycle through of all batteries. This can  
*        be triggered by a read of register 0x40 and allcow the CM4 to cycle through all batteries to learn voltages
*        
*   Rev 0x1E by DorJamJr     
*     - fixed missing "break;" after case 0x32
*     - changed return value of case 0x40 (reset battery arrays) to equal loop_increment so CM4 will know if we have
*        i2c communication hang
*      - Add case 0x34 (returns AC_on() result)
*      - Change variable lowest_voltage start value to 1000 (was 900) (lsb now 4.6 mv, not 5 mv)
*      
*   Rev 0x1F by DorJamJr
*     - Add 100 msec delay before all i2c read() and write functions ("breathe"). 
*       This does seem to fix the i2c error where all data reads are a single constant.
*     
*      
*/

#include <Wire.h>           // for I2C
#include <avr/wdt.h>         // for watchdog timer
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <SoftwareSerial.h>

#include "ATTiny88_pins.h"  // the "" format looks for this file in the project directory

/// VERSION NUMBER ///
#define VERSION_NUMBER 0x1F   // rev level of this code


// Timing constants
// By experiment, the clock is running slow by factor of 8 (8 MHz clock with scale down of 8)
//  so define all time constants are expressed in 8 msec lsb
//  Note that this factor does NOT apply to the watchdog timer (WDT) settings

// DELAY TIMES ... the following based on 8 MHz clock (no pre-scaling)
#define LOOP_DELAY      200      // 200 msec (not too long! remember 1 second watchdog!)
int ps_shutdown_delay = 20000;   // 20 sec (make int so there is an opportunity to change it programatically
const int dwellTime   = 12000;   // 12 secs nominal battery charging time (sec) (each cycle)
int dwellAdjust  = 0;            // adjustment of dwell time to run durning unpowered charging... based on batVoltageATT[] value
int ps_powerup_delay = 2000;     // 2 sec

// Time variables to handle next decision time
unsigned long currentTime;
unsigned long targetTime;
unsigned long powerup_targetTime;

// Misc global variables
int currBat = 0;        // battery currently used to power the system
int batCount = 0;       // number of batteries currently connected
int battPresent[4] = {1,0,0,0};     // set to "1" if battery present
int battBitmap = 0x00;
int batVoltageCM4[4] = {0, 0, 0, 0};   // Battery voltages sent from CM4; LSB = 16 mV
int batVoltageATT[4] = {0, 0, 0, 0};   // Battery voltages read by ATTiny; LSB = 4.94 mV
bool readNextLoop = false;
int batGroup[4] = {0,0,0,0};        // if element is non-zero, indicates bat is part of a battery group
int groupBitmap = 0x00;             // bitmap of current battery group

unsigned long endPowerUpHoldTime;
int poweringDown = false;
int powerOn = false;                        // indicator that power is on (will be set/cleared as I/O PWR_CTL is written)
int powerRequestInProgress = false;

// Constants used to select battEna and battSense FETs by reference (>= battery board 1.7.3)
int battEna[4]   = {PA0, PA1, PA2, PA3};    // FETs to use the selected battery   
int battSense[4] = {PD0, PD1, PD2, PD3};    // sense for battery present (HIGH)

// I/O to sense presence of charger
#define AC_PRESENT   PC2    // digital read AC_IN line

// Declarations of I/O
#define HEARTBEAT      PC3
#define AXP_EXTEN      PD4       // AXP209 EXTEN pin (formerly used to directly control 5V)
#define PWR_IRQ        PD5       // signal to CM4 that power down is in process
#define VCC_OVR        PD6       // signal controlling power to 5V converter
#define PB_IN          PD7       // push button in
#define FASTBEAT       PB7      // debug... toggles every time around loop
#define CATCH_RST      PC1      // debug... pulse when restart
#define BAT_VOLTAGE    ADC0     // read the voltage on the battery(ies) in use (divider 36K, 10K => 5mV lsb)

// #define ENABLE_SERIAL_DEBUG
#ifdef ENABLE_SERIAL_DEBUG
  #define rxPin          PB0      // pin 12... serial data in (not used... but required for SoftwareSerial)
  #define txPin          PB1      // pin 13... serial data out (for logic analyzer)
  SoftwareSerial mySerial = SoftwareSerial(rxPin, txPin);
//  String stringV;
#endif  

/* I2C stuff */
#define I2C_ADDRESS 0x14      // chosen address for this code
byte rcvData;                 // command sent from master
int multi[4] = {0,0,0,0};     // used in handling multi byte i2c communication

// debug stuff
#define CMD_ERROR 0xE8        // constant used in debugging code
int testIncrement = 0;
int loopIncrement = 0;
int probe = 0xAE;
int probe1 = 0xA1;
int probe2 = 0xA2;
int probe3 = 0xA3;
int probe4 = 0xA4;
int heart = 0;
int heart_rate = 1;



/* ----------------- SETUP ------------------------------ */

void setup() 
{
  MCUSR = 0;      // clear all interrupt flags (the step you don't usually find in examples)
  wdt_reset();
  cli();          // clear interrupts?
  wdt_disable();  //Disable WDT
  
  // clock prescaler - 0000 = 8 MHZ, 0001 = 4 MHZ, 0010 = 2 MHZ, 0011 = 1 MHZ
  CLKPR = 0x80;     // enable prescaler write
  CLKPR = 0x00;     // set to 8 MHZ 

 // restore state of power before last reset (wdt or avrdude)
  pinMode(VCC_OVR,OUTPUT);         // control signal for 5V on/off
  pinMode(AXP_EXTEN,INPUT);       // monitor AXP EXTEN pin for 5V power down control
  if (digitalRead(AXP_EXTEN) == HIGH){   // AXP209 calling for power on
    digitalWrite(VCC_OVR,HIGH);   // power on
    powerOn = true;               // set power on flag
  }
  else{
    digitalWrite(VCC_OVR,LOW);   // power off
    powerOn = false;             // set power on flag    
  }
      
  sei();          // enable interrupts

  // housekeeping before enabling the WDT  
  analogReference(INTERNAL);       // reference = 1.1V => 1023 (0x3FF) counts
  pinMode(HEARTBEAT,OUTPUT);      // create heartbeat
  pinMode(FASTBEAT,OUTPUT);
  pinMode(CATCH_RST,OUTPUT);
  digitalWrite(CATCH_RST,HIGH);
  
  // 5V control signals    
  pinMode(PWR_IRQ,OUTPUT);        // indicator signal for 5V going down
  digitalWrite(PWR_IRQ,HIGH);     // initialize
  pinMode(PB_IN,INPUT);           // signal from pushbutton controlling power requests to AXP209

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
  digitalWrite(CATCH_RST,LOW);

#ifdef ENABLE_SERIAL_DEBUG
  // SoftwareSerial setup
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  mySerial.begin(9600);  // baud of 9600 
#endif
}



/*
 * ------------  MAIN LOOP ----------------
 * The main loop... at intervals of dwellTime, check for battery presence on all slots, 
 * determine the next battery to be used, connect that battery, reset the time.
 */
 
void loop() 
{
  int nextBat;
  int batReading;
  

  heartbeat();
  //check_sleepy();                   // if power is down and no charger, put ATTiny to sleep 
  
  checkForBatteries();              // test for presence of all batteries at top of loop and clear batVoltageATT[] for missing bat
  
  check_powerDown_request();        // test and begin power down when AXP EXTEN pin is low
  check_powerUp_request();          // test for power up (check AXP_EXTEN for HIGH indicating power on)
 
  currentTime = millis();           // check if our dwell time (12 sec) is done and we need to change batteries
  if (currentTime > targetTime) {   // finished this charge/discharge cycle for current battery
    readNextLoop = true;            // read the voltage on the active battery the next loop around
    nextBat = next_bat();           // find next battery we want to use
    batterySelect (nextBat);        // turn on FET for "nextBat" and make it "currBat"
    targetTime = currentTime + dwellTime + dwellAdjust;    // set target time for next battery check

    // debug... write bat voltage ATT array to serial pin txPin
#ifdef ENABLE_SERIAL_DEBUG
    mySerial.write("@@@"); // 0x40 (3 times) (recognizeable pattern to aid setting up logic analyzer)
    for (int n = 0; n<4; n++){
      mySerial.print(batVoltageATT[n]);     
      mySerial.write("  ");
    }
#endif
    
  }
  // read the current battery voltage when not in change cycle
  if (readNextLoop == true) {   // we have a new battery so read and store the voltage
    readNextLoop = false;       //  reset flag so we aren't reading on every loop
    batReading = analogRead(BAT_VOLTAGE);   // lsb = 5mV
    batVoltageATT[currBat] = batReading;    // lsb = 5mV 
    
    // also store this voltage for all batteries in batGroup[] IF this bat is part of bat group
    if (batGroup[currBat] == 1){
      for (int n=0; n<4; n++){
        if (batGroup[n] == 1){
          batVoltageATT[n] = batReading;  // weld the group together by making their voltage array readings identical
        }
      }
    }
  }

  loopIncrement += 1;     // debug...
  digitalWrite(FASTBEAT,loopIncrement%2);

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
  if ((heart == 0) or ((poweringDown == true) and (heart == 2))){
    digitalWrite(HEARTBEAT, HIGH);
  }

  else{
    digitalWrite(HEARTBEAT, LOW);    
  }
 }



/*
 * checkForBatteries()
 * This function will check all battery positions for presence of battery
 * If a battery is missing, the associated voltage reading in batVoltageATT[] array 
 *  will be set to zero
 */
void checkForBatteries(){
  int n;
  int changedBatt;
  int x;

  batCount = 0;     // initialize
  for (n=0; n<4; n++){
    if (digitalRead(battSense[n]) == HIGH){
      battPresent[n] = true;
      batCount ++;
    }
    else{         // battery missing, clear presence and voltage array for that bat
      battPresent[n] = false;
      batVoltageATT[n] = 0;    // clear the voltage for a battery not present
    }
  }

  // Note: an added battery just sensed will still have a previous voltage of 0 in the voltage array
  
  // battBitmap is bitmap of batteries present... formerly calculated at i2c register 0x32
  battBitmap = 0;
  x=0;
  for (n=0; n<4; n++){
    battBitmap *= 2;    // batCoded shift left
    if (battPresent[3-n] == true){
      battBitmap += 1;
      x++;
    }
  }
}  // end of checkForBatteries()



/*
 * check_powerDown_request()
 *   Read the AXP_EXTEN line. 
 *     HIGH: no power down request... just return
 *     LOW:  AXP209 sensed power down (either user via pushbutton or supply voltage too low)
 *            so begin power down timer. When timeout, take VCC_OVR low.
 */
void check_powerDown_request(){
  
  if (poweringDown == true and millis() >= endPowerUpHoldTime){
    digitalWrite(VCC_OVR,LOW);         // turn off the 5V supply
    powerOn = false;
    poweringDown = false;
    digitalWrite(PWR_IRQ,HIGH);     // reset
    digitalWrite(PB_IN, HIGH);      // release hold on PB_IN
    pinMode(PB_IN, INPUT);          //  and tristate PB_IN
    // We do the following to allow battery present circuit to detect batteries which have been removed
    //  This will remove all batteries from the weld... ("welded" batteries cannot be detected as missing!)
    //  User MUST NOT remove and reinsert batteries when the unit is running... could fry stuff!!
    force_batteryCycle();
    return;
  }
  
  if (powerOn == false){
    return;       // we aren't powered on, so no need to check about powering off... just return    
  }
  if (digitalRead(AXP_EXTEN) == HIGH){
    return;       // AXP_EXTEN high means AXP209 is not calling for a power down... just return
  }
  
  // If we get here, 5V is on and AXP_EXTEN was LOW, so we need to start (or continue) power down countdown
  if (poweringDown == false){       // just entering the power down sequence
    poweringDown = true;
    endPowerUpHoldTime = millis() + ps_shutdown_delay;    // 20 sec
    pinMode(PB_IN,OUTPUT);
    digitalWrite(PB_IN, LOW);     // hold PB signal low to prevent false retrigger of power up while powering down
    digitalWrite(PWR_IRQ,LOW);      // signal CM4 we are going down
  }
}



/*
 * check_powerUp_request()
 *   If power is off, read the AXP_EXTEN line to see if AXP209 calling for power on
 *   If power is on, just return
 */
 void check_powerUp_request(){
  if (powerOn == true){
    return;
  }
  // if we get here, power was off, so see if AXP209 wants to turn 5V on
  if (digitalRead(AXP_EXTEN) == LOW){
    return;       //  no request to turn on
  }

  // AXP209 wants 5V... test to make sure it is a PB request (not a request due to charger being attached)
  currentTime = millis();           
  if (digitalRead(PB_IN) == HIGH){          // don't power on unless PB is pushed
    powerRequestInProgress = false;
    return;
  }
  else{                                     // PB is low... go into the power up request timing
    if (powerRequestInProgress == false){   // just entering the request
      powerup_targetTime = currentTime + ps_powerup_delay;  // set target for power up timer
      powerRequestInProgress = true;
      return;
    }
    else{                                   // we are in a power up request... see if we have reached our target time
      if (currentTime > powerup_targetTime){
        digitalWrite(VCC_OVR,HIGH);  //  and set HIGH to hold 5V on
        powerOn = true;      
        powerRequestInProgress = false;
      }
    }  
  }
} 



/*
 * next_bat()
 *   Determine the next battery to select based on the following logic:
 *   - IF: any battPresent[] == true with batVoltageATT[] == 0, select that battery -> LOOP
 *   - ELSE:
 *    - IF charging:
 *       - Choose battery with lowest voltage -> LOOP
 *   - IF not charging:    
 *       - Choose battery with highest voltage -> LOOP
 *     
 *  Note: This routine returns a single battery number BUT will return battery number +4 if eligible for multiBattery selection  
 */
int next_bat() 
{
  int n, m;
  int nextBat;
  int highest_voltage = 0;
  int lowest_voltage = 1000;   // lsb = 4.6mV => 4.6V 
  bool found_battery = false;
  long avgVBatt = 0;
  int validBatts = 0;

  dwellAdjust = 0;    // default unless we are unpowered and charging
   
  // Initial battery reading... pick next battery which is present but has no voltage reading...
  //  Note: we want JUST THIS battery to be selected since we will figure out its voltage during this cycle
  //   and don't want the reading corrupted by parallel batteries... so return the real battery number (not +4)
  for (n=0; n<4; n++){
    m = (n + 1 + currBat) % 4;    // start with battery AFTER current battery and look through all 4 battery positions
    if (battPresent[m] && (batVoltageATT[m] == 0)){    // special case... battery present but not yet read by ATTiny
        nextBat = m;                                //  so FORCE this battery
        return nextBat;     // only case where we will force single (non parallel) battery combination
    } 
  } 
    
  // all batteries have had their voltages read... proceed with normal selection
  //  which will choose the lowest number battery meeting the criteria (so if equal readings,
  //  meeting the voltage criteria (lowest/highest voltage) the lowest number battery is selected) 

  // check to see if we are charging
  if (AC_on()) {     // charging... so pick the lowest battery
    for (n=0; n<4; n++) {
      if ((batVoltageATT[n] < lowest_voltage) && (batVoltageATT[n] > 580)) {  // lsb = 5mv => batt > 2900 mV to be selected
        nextBat = n;
        lowest_voltage = batVoltageATT[n];
      }
    }
  }

  else {                      // discharging, so pick the highest voltage battery
    for (n=0; n<4; n++){
      if (batVoltageATT[n] > highest_voltage) {
        nextBat = n;
        highest_voltage = batVoltageATT[n];
      }
    }
  }

  if (powerOn == true){
   // if we got here, the selection can be multiple batteries, so return nextBat +4 to signal that to batterySelect()     
     return nextBat + 4;  
   }
   else{
     return nextBat;   // don't allow welding if power isn't on... (the time when users can change batteries)
   }
}



/*
 * batterySelect (int batt)
 * Turn on FET for select battery position (passing in the nextBat reference)
 * Interrupts are suspended just before the switch over and enabled just after to 
 *  insure that we aren't servicing an interrupt while both FETs are on
 *  
 * Note that the determination of batteries to be on (grouping) is done before any 
 *  actual switching of batteries. Once the set of on batteries is determined, interrupts
 *  are suspended, the "ON" batteries are turned on, then the "OFF" batteries are turned
 *  off and finally, interrupts are enabled. 
 */
 
void batterySelect (int batt)
{
  int n;
  bool grouping_ok;

  if (batt >= 4){
    grouping_ok = true;
    batt -= 4;    // restore actual battery number
  }
  else{
    grouping_ok = false;
  }

  // clear the group
  for (n=0;n<4;n++){
    batGroup[n] = 0;
  }
  batGroup[batt] = 1;             // add new selection to batGroup[] (it may be the only one)
  
  if (grouping_ok == true){      // the multiple battery function... 
  // add to batGroup[] any battery within 6 lsb of the same voltage (lsb = 5mV) 
    for (n=1; n<4; n++){
      if (abs (batVoltageATT[(batt+n)%4] - batVoltageATT[batt])<=6){    // within 30 mV
        batGroup[(batt+n)%4] = 1;                  // add to group and     // 0x33
      }
    }
  }

  // build up the bitmap
  groupBitmap = 0;
  for (n=0; n<4; n++){
    groupBitmap *= 2;    // batCoded shift left
    if (batGroup[3-n] == true){
      groupBitmap += 1;
    }
  }
  
  noInterrupts();                     // turn off interrupts while we change FETs
  // turn on FETs for batteries with "1" in the batGroup[]
  for (n=0; n<4; n++){
    if (batGroup[n] == 1){
      digitalWrite (battEna[n], LOW);     //  turn on battery in the batGroup[]
    }
  }
  for (n=0; n<4; n++){
    if (batGroup[n] == 0){
      digitalWrite (battEna[n], HIGH);     //  turn off battery not in the batGroup[]
    }
  }
  interrupts();                       // turn interrupts back on
  currBat = batt;                     // the FET is turned on, so now "nextBat" becomes "currBat"
}



/*
 * AC_on()
 * Determine if AC_IN line is powered (ie, is charger connected) and return TRUE/FALSE
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
 * force_batteryCycle()
 *   Clears the battery registers to force selection and read of all batteries present.
 *   This is called at power down to allow user to safely change batteries. Also called
 *   in response to a read of register 0x40 (allowing the CM4 to trigger this function).
 */
void force_batteryCycle(){
    for (int n=0; n<4; n++){
      battPresent[n] = false;
      batVoltageATT[n] = 0;    // clear all the voltages and battery present arrays
    }
    digitalWrite (battEna[0], LOW);      //  turn on  battery 1
    digitalWrite (battEna[1], HIGH);     //  turn off battery 2
    digitalWrite (battEna[2], HIGH);     //  turn off battery 3
    digitalWrite (battEna[3], HIGH);     //  turn off battery 4
 }



/*
 * check_sleepy()
 *  If power is down and we don't have a charger connected, put ATTiny in sleep mode
 *  Note that we want to put all output pins into input mode to save power during sleep
 *   (and then will need to restore them on waking up!)
 *  ALSO NOTE: This function is not currently used but is left in place in case we want 
 *   to try to make it work in the future.
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
  pinMode(VCC_OVR,OUTPUT);         // control signal for 5V on/off
  digitalWrite(VCC_OVR,LOW);      // hold power off until we test for power on request
  powerOn = false;
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

  // Not really using this CM4 data anymore for ATTiny... just for CM4 to get for display on OLED
  // clear out the batVoltageCM4[] array and let CM4 refill it
  for (int n=0;n<4;n++){
    batVoltageCM4[n] = 0;
  }
  
  sei();                    // enable interrupts
  wdt_enable(WDTO_1S);      // enable WDT with a (real) timeout of 1 sec
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
       delay(100);        // breathe??
       rcvData = Wire.read();
    }
  }

  if (numBytes == 2){         // this is a register write
    delay(100);           // breathe??
    rcvReg = Wire.read();     // the register to write
    multi[0] = rcvReg;
    delay(100);           // breathe??
    rcvVal = Wire.read();     // the value to write
    multi[1]= rcvVal;
    rcvData = rcvReg;         // set up for possible request to retrieve the value written

  // Handle the battery voltage write by putting the value to the appropriate batVoltageCM4 array member
    if ((rcvReg >= 0x21) && (rcvReg <= 0x24)){
      batVoltageCM4[rcvReg - 0x21] = rcvVal;   // use value sent (lsb = 16mV)
    //  and if part of batGroup[], fill in batVoltageCM4[] for all members of group
      if (batGroup[rcvReg - 0x21] == 1){  
        for (int n=0; n<4; n++){        
          if (batGroup[n] == 1){
            batVoltageCM4[n] = rcvVal;
          }
        }
      }   
    }
  }       // else... we have a normal request from the CM4
}



/*
 * The DataRequest() function sends data back to the master, usually in response to
 *  a request for data from the master. So here we will interpret the rcvData, formulate
 *  a proper response and call the Wire.write function.  
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
      sendData = batVoltageCM4[0];
      break;  

    case 0x22:    // single byte = (battery voltage battery 2) / 16
      sendData = batVoltageCM4[1];
      break;  

    case 0x23:    // single byte = (battery voltage battery 3) / 16
      sendData = batVoltageCM4[2];
      break;  

    case 0x24:    // single byte = (battery voltage battery 4) / 16
      sendData = batVoltageCM4[3];
      break;  


    case 0x30:                  // Battery Count
      sendData = batCount;
      break;   
        
    case 0x31:                  // Current Battery
      sendData = currBat + 1;   // (battery number is zero based)
      break;

    case 0x32:    // sendData = binary representation of battPresent array; bit 0 == battPresent[0], etc
      sendData = battBitmap;
      break;
      
    case 0x33:    // sendData = binary representation of batGroup array; bit 0 == batt1 in array, etc
      sendData = groupBitmap;
      break;
      
    case 0x34:
       sendData = AC_on();
       break;  


    case 0x37:        // report time remaining in power up hold
      if (poweringDown){
        sendData = (endPowerUpHoldTime - millis());        // 1000 msec/sec / 8 (clock scaling) (scaling removed)
      }
      else{
        sendData = 0xFF;
      }
      break;   

    case 0x40:                  // Request from CM4 to reset battery arrays so all bat voltages can
      force_batteryCycle();           // Forces batteries to cycle so CM4 can read voltages
      sendData = loopIncrement;       // increments every 200 msec
      break;

    case 0x41:    // Bat 1 voltage as read by ATTiny ADC0, lsb = 5mV 
      sendData = ((batVoltageATT[0])>>2) & 0xFF;  // lsb = 20mV
      break;   

    case 0x42:    // Bat 2 voltage as read by ATTiny ADC0, lsb = 5mV
      sendData = ((batVoltageATT[1])>>2) & 0xFF; // lsb = 20mV
      break;   

    case 0x43:    // Bat 3 voltage as read by ATTiny ADC0, lsb = 5mV
      sendData = ((batVoltageATT[2])>>2) & 0xFF; // lsb = 20mV
      break;   

    case 0x44:    // Bat 4 voltage as read by ATTiny ADC0, lsb = 5V
      sendData = ((batVoltageATT[3])>>2) & 0xFF; // lsb = 20mV
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
    case 0x59:
      sendData = probe3 & 0xFF;
      break;
    case 0x5A:
      sendData = probe3 >> 8;
      break;

    // debug stuff
    case 0x61:
      sendData = battPresent[0];
      break;  
    case 0x62:
      sendData = battPresent[1];
      break;  
    case 0x63:
      sendData = battPresent[2];
      break;  
    case 0x64:
      sendData = battPresent[3];
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

    case 0x88:
      sendData = probe >> 8;
      break;  

    case 0x99:    // reset the voltage readings
      for (int n=0; n<4; n++){
        batVoltageATT[n] = 0;
      }
      break;
             
    default:                // Command Not Recognized
      sendData = CMD_ERROR;   
      break;
  } 
  delay(100);               // breathe?? 
  Wire.write(sendData);
  //Wire.endTransmission();   // master doesn't like this... causes i2cdetect to fail...
  
}
