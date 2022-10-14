/* From https://microcontrollerslab.com/attiny88-microcontroller-pinout-programming-datasheet-features-applications/ */

/* Defines common names of pins to their GPIO number 
*  
* It appears that we just use the NUMBER not the "GPIO" part 
* (Note that although the net site shows these as GPIO numbers
*  they actually correspond to pin #'s for the 28 pin DIP version.)
*/

/* CLOCK PINS */
#define CLKI 09
#define CLKO 14

/* DIGITAL IO */
#define PB0  14  // 8 (?)
#define PB1  15
#define PB2  16
#define PB3  17
#define PB4  18
#define PB5  19 // 13 (?)
#define PB6  09
#define PB7  10

#define PC0  23
#define PC1  24
#define PC2  25
#define PC3  26
#define PC4  27
#define PC5  28
#define PC7  21

#define PD0  02 
#define PD1  03
#define PD2  04
#define PD3  05
#define PD4  06
#define PD5  11 // 5 (?)
#define PD6  12
#define PD7  13 // 7 (?)

/* INTERRUPT PINS */
#define INT0  04
#define INT0  05

/* SPI COMMUNICATIONS */
#define MOSI  17
#define MISO  18
#define SCK   19
#define SS'   26    // slave select pin - can use others...

/* I2C COMMUNICATION */
#define SCL  28
#define SDA  27

/* TIMERS */
#define T0  06
#define T1  11

/* TIMER/COUNTER OUTPUT COMPARE */
#define ICP1  14    // input
#define OC1A  15    // output 1
#define OC1B  16    // output 2

/* ANALOG COMPARATOR INPT PINS */
#define AIN0  12    
#define AIN1  13
    
/* ANALOG TO DIGITAL INPUT PINS */
#define ADC0 23
#define ADC1 24
#define ADC2 25
#define ADC3 26
#define ADC4 27
#define ADC5 28

/* AVCC PINS (required for ADC to operate) */
#define AVCC 20

/* RESET PIN */
#define RESET' 01
