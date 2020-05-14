// Firmware for the CIWS Residential Datalogger
// Arduino IDE ver. 1.8.9
// Utah Water Research Lab
// Updated: 11/05/2019 (changed to be compatible with Pololu LIS3MDL sensor board)
// Daniel Henshaw and Josh Tracy
// Note: F("String") keeps string literals in program memory and out of RAM. Saves RAM space. Very good. Don't remove the F. I know it looks funny. But don't do it. Really. The program might crash if you do. And then you'll have dishonor on yourself, dishonor on your cow, and you'll find out your cricket ain't lucky.
// Note: Be sure to process resulting data file as ASCII characters, not Unicode. 
/*******************************************************************************************\
* Hardware Description
*                     ___________________   TWI                       ___
*                    |                   |<------------------------->[___] RTC  
*                    |                   |                   |        ___
*                    |                   |                    ------>|___| Magnetometer
*  ________          |                   |  SPI                       _______
* |        |         |                   |<------------------------->|       | SD
* |        |<------->|                   |<-----[] Activate Serial   |______/
* |________|         |___________________|  GPIO
*  Serial             Controller
* 
* Serial:          Serial interface for user interaction with the logger
* Controller:      Arduino Pro/Pro Mini (used for SD library and lower power options)
* SD:              External SD Card storage for the logger.
* Activate Serial: Button to wake up the controller and activate the Serial interface
* Sensor:          Hall Effect Sensor for logging pulses from water meters.
* RTC:             Real Time Clock to track time and wake up controller every four seconds.
\*******************************************************************************************/

/*****************************************************************\
* Software Description
* Overview: 
*   User inputs:
*     Serial input.
*   Device inputs:
*     Magnetometer Sensor
*     Real Time Clock
*   Device outputs:
*     Serial output
*     Datalog file
*     
* Structure:
*   1. Setup
*   2. Handle Serial (for start-up configuration)
*   3. Loop
*        If serial is enabled
*          Handle serial input
*        If four seconds are up
*          Update the time
*          Construct a timestamp
*          Write data
*        If sleep is enabled (disabled when serial is enabled)
*          Enter Sleep (low-power mode)
*          < Will wake up on Interrupt and continue: >
*        If the Magnetometer interrupted:
*          Read the magnetometer data and check for peaks.
*      Repeat Loop
*      
* Interrupts:
*   1. INT0_ISR()
*      
*   2. INT1_ISR()
*      
\*****************************************************************/

#include <SPI.h>
#include <SD.h>

#include "handleSerial.h"
#include "powerSleep.h"
#include "state.h"
#include "RTC_PCF8523.h"
#include "storeNewRecord.h"
#include "detectPeaks.h"
#include "magnetometer.h"
#include "configuration.h"

// These six macros below are the ten's place mask for the various real time clock registers
#define SECONDS_REG_MASK 0x7                            // keep lower 3 bits
#define MINUTES_REG_MASK 0x7
#define HOURS_REG_MASK   0x3                            // keep lower 2 bits
#define DAYS_REG_MASK    0x3
#define MONTHS_REG_MASK  0x1                            // keep lower 1 bit
#define YEARS_REG_MASK   0xF                            // keep all four bits


/*********************************************************************************\
 * Setup:
 *    Setup the System State structure     
 *    Setup the Digital I/O pins
 *      Pin 2 (INT0):  Magnetometer Interrupt
 *      Pin 3 (INT1):  RTC 4-Second Interrupt
 *      Pin 5 (INPUT): Serial Activate Button
 *    Initialize Magnetometer
 *    Setup RTC Timer
 *    Setup interrupts
 *    Load Date_t with Date/Time info
 *    Disable unneeded peripherals
 *    If Activate Serial Button is pressed
 *      Power on the Serial Interface
\*********************************************************************************/

volatile State_t State;             // System State structure
Date_t Date;                        // System Time and Date structure
File dataFile;                      // File pointer for SD Card operations
volatile SignalState_t SignalState; // Struct containing signal data from magnetometer
byte currDay;                       // Tracks current day for file chunking

void setup() 
{
  resetState(&State);             // Setup the System State structure
  
  pinMode(2, INPUT);              // Setup the Digital I/O Pins       
  pinMode(3, INPUT_PULLUP);
  pinMode(5, INPUT);
  pinMode(4, OUTPUT); // For debugging
  digitalWrite(4, LOW);

  mag_init(); //magnetometerInit(&mag);         // Initialize Magnetometer    // changed out to new function.  4/17/19 by D.H.
  

  rtcTransfer(reg_Tmr_CLKOUT_ctrl, WRITE, 0x3A);                  // Setup RTC Timer
  rtcTransfer(reg_Tmr_A_freq_ctrl, WRITE, 0x02);
  rtcTransfer(reg_Tmr_A_reg, WRITE, 0x04);
  rtcTransfer(reg_Control_2, WRITE, 0x02);
  rtcTransfer(reg_Control_3, WRITE, 0x80);                        

  attachInterrupt(digitalPinToInterrupt(2), INT0_ISR, RISING);   // Setup Interrupts
  attachInterrupt(digitalPinToInterrupt(3), INT1_ISR, FALLING);
  EIMSK &= ~(1 << INT0);        // Disable Sensor interrupt
  EIMSK |= (1 << INT1);         // Enable 4-Second RTC interrupt.
  
  loadDateTime(&Date);            // Load Date_t with Date/Time info
  currDay = Date.days;
  
  if(configurationExists())
  {
    State.configured = true;
    char factor1 = readConfiguration(addr_factor1);
    char factorTenths = readConfiguration(addr_factorTenths);
    char factorHundredths = readConfiguration(addr_factorHundredths);
    char factorThousandths = readConfiguration(addr_factorThousandths);
    State.meterSize = (float)(factor1 - 48) + (float)((factorTenths - 48) / 10.0) + (float)((factorHundredths - 48) / 100.0) + (float)((factorThousandths - 48) / 1000.0);
  }
  else
    State.configured = false;
                                  
  disableUnneededPeripherals();   // Disable unneeded peripherals

  if((digitalRead(5) == 0) && !State.serialOn)  // If Activate Serial Button is pressed
  {
    State.serialOn = true;                        // Power on the Serial Interface
    serialPowerUp();
    if(!State.configured)
      Serial.print(F(">> Logger: Invalid device configuration. Reset configuration with command 'g'\n"));
  }
}

void loop() 
{
  // JOSH
  /*****************************************\
  * ButtonCheck: Is the button pressed?
  * If button is pressed (active-low):
  *   Set serialOn flag.
  *   call serialPowerUp()
  \*****************************************/  
  if((digitalRead(5) == 0) && !State.serialOn)
  {
    State.serialOn = true;
    serialPowerUp();
    if(!State.configured)
      Serial.print(F(">> Logger: Invalid device configuration. Reset configuration with command 'g'\n"));
  }
    
  // JOSH
  /*****************************************\
  * Serial: User I/O over serial
  * serialOn flag is set:
  *   call function handleSerial();
  \*****************************************/
  if(State.serialOn)
    handleSerial(&State, &Date, &SignalState);
  
  // DANIEL
  /*****************************************\
  * 4-second update: Update at 4 seconds
  * If 4-second flag is set:
  *   Store a new record
  \*****************************************/
  if(State.flag4)
  {
    State.flag4 = 0;                                    //     Reset flag4 to zero
    rtcTransfer(reg_Control_2, WRITE, 0x02);            //     Reset real time clock interrupt flag
    loadDateTime(&Date);
    if(State.logging)
    {
      if(Date.days != currDay)
      {
        currDay = Date.days;
        incrementFileNumber();
        nameFile(&State, &Date);
        createHeader(&State);
      }
      storeNewRecord();
    }
  }
  
  // JOSH
  /*****************************************\
   * Read Magnetometer: 
   * 
   * If readMag is set:
   *   call readData(&mag, &SignalState);
   *   If peakDetected(&SignalState)
   *     State.pulseCount += 1;
  \*****************************************/
  if(State.readMag)
  {
    //if(State.serialOn)
    //  Serial.println("Interrupted");
    State.readMag = false;
    readData(&SignalState);  // changed from calling read_mag(mag_out) back to readData(&SignalState). Now it should work as intented, per J.T.'s instructions to me. 4/23/19 by D.H.

    // bool peak = peakDetected(&SignalState);
    bool peak = peakDetected2(&SignalState);
    
    if(peak){
      State.pulseCount += 1;
      //if(State.serialOn)
        //Serial.println("Peak Detected");
    }
  }
  
  // JOSH
  /*****************************************\
  * Sleep: put processor to sleep
  *        to be woken by interrupts
  * If serialOn is not set:
  *   call function Sleep();
  \*****************************************/
  //if(!State.serialOn)
    //enterSleep();

}

// DANIEL
/* Function Title: INT0_ISR()
 * 
 * Friendly Name:  Sensor Interrupt Service Routine (ISR)
 * 
 * Description: increments the value of the pulse count variable by one, each time this 
 *              function is called by hardware.
 */
void INT0_ISR()
{
  // For magnetometer version: set a flag which will allow the datalogger to read the magnetometer
  State.readMag = true;
}

// DANIEL
/* Function Title: INT1_ISR()
 * 
 * Friendly Name:  Real Time Clock OUT Interrupt Service Routine (ISR)
 * 
 * Description: sets the 4-second flag to true each time this function
 *              is called by hardware. The Real Time Clock generates
 *              the signal that calls this ISR once every four seconds.
 */
void INT1_ISR()
{
  State.flag4 = true;     // sets the "four second flag" to true
}

/*********************************************\
 * Function Name: numDigits
 * Purpose:       Calculate the number of
 *                digits in a value (which
 *                equals the number of bytes
 *                in that value)
\*********************************************/

byte numDigits(unsigned long value)
{
  byte count = 0;
  if(value == 0)
    count = 1;
  while(value != 0)
  {
    value = value / 10;
    count++;
  }

  return count;
}

/* Function: storeNewRecord
 *  
 * Author: Daniel Henshaw & AJ Beckwith
 * Date: 11/10/18    
 * 
 * Description: This function gets the current time from the Real Time Clock, and then  
 *              carries out the communication to write a new record to the SD card and 
 *              then turns off the I2C and SD card.
 * 
 * Pseudocode: 
 *  
 * Begin
 *   Reset flag4 to zero
 *   Reset real time clock interrupt flag
 *   Declare variables
 *   Store pulse count to a variable named final count
 *   Set pulseCount to zero
 *   Write the new record to the SD card  
 *     power on SD card  
 *      Store value of file position in pos
 *      Write string to SD
 *      Move SD pointer back to pos
 *      While bytes are available to be read, read, and if -1 is returned rewrite string
 *     power down SD card     
 * End of function storeNewRecord() 
 */

void storeNewRecord() 
{                                                       // Begin
    unsigned int finalCount;                            //     Declare variables
    byte temp;
    bool writeErrorCaught = false;
    byte numBytes = 0;
    finalCount = State.pulseCount;                      //     Store pulse count to a variable named final count
    State.pulseCount = 0;                               //     Set pulseCount to zero
    State.lastCount = finalCount;
    State.totalCount += (unsigned long)finalCount;

      SDPowerUp();  
      dataFile = SD.open(State.filename, FILE_WRITE);
      if(!dataFile)
      {
        if(State.serialOn)
          Serial.print(F("\n>> Whoa dudes, bad file pointer!\n>> User:   "));
        writeErrorCaught = true;
      }
      else 
      {
        bool finished;
        int pos = dataFile.position();          //Store file position
        do{
          finished = true;
          dataFile.print("\""+String(Date.years)+"-"+String(Date.months)+"-"+String(Date.days)+" "+String(Date.hours)+":"+String(Date.minutes)+":"+String(Date.seconds)+"\","+String(State.recordNum)+","+String(finalCount)+"\n");       //write sting to file
          dataFile.seek(pos);         //Move file position back to starting point
          while(dataFile.available()){          //Continue as long as there are bytes to read
            if(dataFile.read() == -1){          //If byte is corrupted it will return -1
              finished = false;         //Set flag to fasle to redo writing
              dataFile.seek(pos);         //Move file position back to beginning for rewriting
              markError();
              break;
            }         
          }
        }while(!finished);          //continues as long as bytes are returned corrupted
      }    
      dataFile.close();         //closes file
      SDPowerDown();          //Powers down SD
      State.recordNum += 1;
}






/* Function: bcdtobin
 *  
 * Author: Daniel Henshaw             
 * Date: 11/17/18    
 * 
 * Description: This function converts Binary Coded Decimal (BCD) numbers to 
 *              standard binary format. This is particularly useful when working
 *              with the real time clock, which stores time data in BCD format
 *              in its registers. The function takes in the value to convert, and
 *              the source register the value is coming from. The source register
 *              is required so that the conversion will be compatible with the 
 *              number of bits into which the BCD value is stored into the RTC's
 *              registers.
 *
 * Optional dependencies: set of macro definitions, included below, improve
 *              this function's usability or user friendliness. These make it
 *              straightforward for the user to specify from which register
 *              the data is coming from. These also do double duty as masks for the 
 *              value, as not all 8 bits in the register are for the BCD value; some 
 *              of the upper bits serve other purposes or are "unused" 
 *              (see datasheet for PCF8523 by NXP).
 *              
 *              #define SECONDS_REG_MASK 0x7
 *              #define MINUTES_REG_MASK 0x7
 *              #define HOURS_REG_MASK   0x3
 *              #define DAYS_REG_MASK    0x3
 *              #define MONTHS_REG_MASK  0x1
 *              #define YEARS_REG_MASK   0xF
 *
 * Pseudocode: 
 *  
 * Begin
 *   Declare variable
 *   shift right by four the bcdValue (Puts upper 4 bits at the bottom)
 *   tensPlace is tensPlace ANDed with the provided bit mask a.k.a. conversion constant (hexadecimal value)
 *   Return the binary value result (the unit's place plus tensPlace times ten)
 * End   
 */

byte bcdtobin(byte bcdValue, byte sourceReg)
{
  byte tensPlace;
 
  tensPlace  = bcdValue >> 4;
  tensPlace &= sourceReg;         /* NOTE: sourceReg is actually a bit mask */

  return (bcdValue & 0x0F) + (tensPlace * 10); // return the binary value result: 
                                               //   step 1) First parenthesis: bcdValue ANDed with mask 0xF gives us the one's place, 
                                               //   step 2) Second parenthesis: move the ten's place value over by multiplying by ten
                                               //   step 3) Add the first parenthesis group to the second parenthesis group
}
