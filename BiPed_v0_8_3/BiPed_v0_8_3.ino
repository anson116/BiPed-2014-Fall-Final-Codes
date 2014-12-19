/* The following two libraries are included with the Arduino IDE  */
#include <SPI.h>                        // supports SPI serial communication and defines analog pins
#include <EEPROM.h>
#include <Servo.h>
#include <Wire.h>                       // I2C support (currently not implemented)
#include <L3G4200D.h>                   // 3-axis Gyro (currently not implemented)
#include <SoftwareSerial.h>             // import the serial library
#include "Pinouts_and_Define_ROFIA.h"   // Pinout for BiPed ROFIA

void setup()
{
  Serial.begin(57600);                  // legacy rovers operated at 57600
  #if bluetooth
  Serial1.begin(9600);
  #endif
 
  timer = millis();                     // telemetry  
  pinMode(led, OUTPUT);                 // initialize LED indicator as an output.
  //initialize the servos
  initializeServos();
  // Apply the servo calibrations to the joint positions.
  ApplyCalibration();
  PlayFrames(numberOfFramesNeutral, playbackDelayNeutral, "home");
  // Set up the Accelerometer
  setUpAcclerometer();
  
  // Delay to give the user time to set the robot down after the legs straighten out
  delay(2000);
  stand = DetectTorque();
}

void loop()
{
  #if bluetooth
  if(Serial1.available() ) commandDecoder();      // note: Leonardo does not support serialEvent() handler
  #else
  if(Serial.available() ) commandDecoder();
  #endif
  
  // future: replace with watchdog timer interrupt  ****
  if (millis() > getNextPing())
  {
    sendWordPacket(EMERGENCY_ID,WATCHDOG_TIMEOUT);
    
    #if bluetooth                                 // if packet sent over USART=>bluetooth,  
    Serial.print("Emergency exception 0x0");      // send duplicate data as text to
    Serial.println(WATCHDOG_TIMEOUT,HEX);         // USB=>Arduino IDE Serial Monitor.
    #endif 
    
    // ****** SLEEP PAPERBOT *******
    // ##### JEFF CHANGED
    // Until the above-mentioned "SLEEP PAPERBOT" is implemented,
    // let's reset to allow another ping interval to avoid the
    // constant barrage of an exception message on every loop.
    updateNextPing();
    // #####
  }
  
  int16_t  torque = DetectTorque();
  // Balance the robot based on the applied torque
  if((stand - torque) > leftLean)
  {
    if ((millis() - lastDebounceTime) > debounceDelay)
    {
      PlayFrames(numberOfFramesLeftLean, playbackDelayBalance, "leftLean");
      // Reset the debouncing timer
      lastDebounceTime = millis();
    }
//    Serial.println("leftLean");
  }
    if((stand - torque) < rightLean)
    {
      if ((millis() - lastDebounceTime) > debounceDelay)
      {
        PlayFrames(numberOfFramesRightLean, playbackDelayBalance, "rightLean");
        // Reset the debouncing time
        lastDebounceTime = millis();
      }
//    Serial.println("rightLean");
    }
  if ((millis() - lastDebounceTime) < debounceDelay)
    stand = torque;                               // While the debounce time, adjust BiPed's stand position
}  // end of loop
