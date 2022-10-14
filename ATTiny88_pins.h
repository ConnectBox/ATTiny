/* Defines common names of pins to their program "pin" designator 
 *  
 *  These agree with the ATMEGA328 scheme and were experimentally verified
 *  
 *  In summary, PD0  -> PD7  map to "pin"  0 -> 7
 *              PB0  -> PB7  map to "pin"  8 -> 15
 *              PC0  -> PC5  map to "pin" 17 -> 22
 *              ADC0 -> ADC5 map to "pin" A0 -> A5
 */


/* DIGITAL IO *//* Physical pin DIP28 -- 32QFN*/
#define PB0  8          //      14        12 
#define PB1  9          //      15        13
#define PB2  10         //      16        14
#define PB3  11         //      17        15
#define PB4  12         //      18        16
#define PB5  13         //      19        17
#define PB6  14         //       9         7
#define PB7  15         //      10         8

#define PC0  17         //      23        23
#define PC1  18         //      24        24
#define PC2  19         //      25        25
#define PC3  20         //      26        26
#define PC4  21         //      27        27
#define PC5  22         //      28        28

#define PD0  0          //       2        30
#define PD1  1          //       3        31
#define PD2  2          //       4        32
#define PD3  3          //       5         1
#define PD4  4          //       6         2
#define PD5  5          //      11         9
#define PD6  6          //      12        10
#define PD7  7          //      13        11

#define PA0  23         //      --        19
#define PA1  24         //      --        22
#define PA2  25         //      --         3
#define PA3  26         //      --         6


   
/* ANALOG TO DIGITAL INPUT PINS */
#define ADC0 A0         //      23        23
#define ADC1 A1         //      24        24
#define ADC2 A2         //      25        25
#define ADC3 A3         //      26        26
#define ADC4 A4         //      27        27
#define ADC5 A5         //      28        28
