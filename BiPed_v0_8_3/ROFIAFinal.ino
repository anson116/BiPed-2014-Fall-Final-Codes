#include <Servo.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Initialization
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void initializeServos()
{
  // Assign the correct pin to each servo.
  for(int s = 0; s < numberOfServos; s++)
    servos[s].attach(servoPins[s]);
  
  scanServo.attach(34, servoMin, servoMax);
  tiltServo.attach(36, servoMin, servoMax);
  // reset android device 
  reset_camera();
}

void setUpAcclerometer()
{
  // Set up the Accelerometer
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);              // PWR_MGMT_1 register
  Wire.write(0);                 // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Calibration
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ApplyCalibration()
{
  // Apply the servo calibrations to each frame of the animation.
  // This is done before hand to keep from slowing down the playback.
  // The frames could also be stored with the calibrations already
  // applied, however leaving the calibration seperate allows other
  // ROFIs to use this same action with their own calibration.
  for(int f = 0; f < numberOfFramesForward; f++)
    for(int s = 0; s < numberOfServos; s++)    
      framesForward[f][s] = CorrectJointAngle(framesForward[f][s], s);
      
  for(int f = 0; f < numberOfFramesNeutral; f++)
    for(int s = 0; s < numberOfServos; s++)    
      framesNeutral[f][s] = CorrectJointAngle(framesNeutral[f][s], s);
      
  for(int f = 0; f < numberOfFramesRight; f++)
    for(int s = 0; s < numberOfServos; s++)    
      framesRight[f][s] = CorrectJointAngle(framesRight[f][s], s);
      
  for(int f = 0; f < numberOfFramesLeft; f++)
    for(int s = 0; s < numberOfServos; s++)    
      framesLeft[f][s] = CorrectJointAngle(framesLeft[f][s], s);
      
  for(int f = 0; f < numberOfFramesLeftLean; f++)
    for(int s = 0; s < numberOfServos; s++)    
      framesLeftLean[f][s] = CorrectJointAngle(framesLeftLean[f][s], s);
      
  for(int f = 0; f < numberOfFramesRightLean; f++)
    for(int s = 0; s < numberOfServos; s++)    
      framesRightLean[f][s] = CorrectJointAngle(framesRightLean[f][s], s);
}

double CorrectJointAngle(double inputAngle, int servo)
{
  // The input angle is what the angle should be.
  // The corrected angle is the angle that has to be sent to the servo to achieve the input angle.
  if (inputAngle > 0)
    // Do a two point calibration between the middle and high corrected values.
    return map(inputAngle, 0, 4500, 0 + servoCalibrations[servo][1], 4500 + servoCalibrations[servo][2]);
  else
    // Do a two point calibration between the low and middle corrected values.
    return map(inputAngle, -4500, 0, -4500 + servoCalibrations[servo][0], 0 + servoCalibrations[servo][1]);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// BiPed Movement
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/***********************************
 * Sample For ROFIA Movement Comment Test
 * Forward:
 * A5 05 01 01 80 01 80 A1
 * TurnRight:
 * A5 05 01 01 80 02 80 A2
 * TurnLeft:
 * A5 05 01 02 80 01 80 A2
 ***********************************/
void move_BiPed(uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6)
{
  // Distance detected by the ultrasonic sensor
  unsigned long cm = DetectDistance(pingPin);
  delay(50);
  sendData(); 
   
  /***********************************
  * motion command = 0x01
  * motordata[3]   left run    (FORWARD = index 1, BACKWARD = index 2, BRAKE = index 3, RELEASE = index 4)
  * motordata[4]   left speed  0 - 255
  * motordata[5]   right run   (FORWARD, BACKWARD, BRAKE, RELEASE) 
  * motordata[6]   right speed 0 - 255
  * example
  * forward half speed  0x01, 0x01, 0x80, 0x01, 0x80 0101800180
  ***********************************/

  if (data3 == data5 && data4 == data6 && data3 == 1 && data4 != 0)
  {
    // The robot will walk forward, but will turn right if there is an obstacle 30 cm or less in front of it
    if(cm >= 43)
    {
      PlayFrames(numberOfFramesForward, playbackDelayForward, "forward");
//      Serial.println("Walk Forward");
    }
    else
    {
      for (int i = 0; i < 4; i++)
      {
        PlayFrames(numberOfFramesRight, playbackDelayRight, "right");
//        Serial.println("Turn Rignt");
      }
    }
    PlayFrames(numberOfFramesNeutral, playbackDelayNeutral, "home");
  }
  else if (data3 == 4 && data5 == 4 && data4 == 0 && data6 == 0)
  {
    // The robot will go back to home position
    PlayFrames(numberOfFramesNeutral, playbackDelayNeutral, "home");
//    Serial.println("Stay Stand");    
  }
  else if ((data3 == 1 && data5 != 1)||(data3 == 1 && data5 == 1 && data4 > data6)||(data3 != 2 && data5 == 2)||(data3 == 2 && data5 == 2 && data4 < data6))
  {
    // The robot move right
    for (int i = 0; i < 4; i++)
    {
      PlayFrames(numberOfFramesRight, playbackDelayRight, "right");
//      Serial.println("Turn Rignt"); 
    }
    PlayFrames(numberOfFramesNeutral, playbackDelayNeutral, "home");
  }
  else if ((data3 != 1 && data5 == 1)||(data3 == 1 && data5 == 1 && data4 < data6)||(data3 == 2 && data5 != 2)||(data3 == 2 && data5 == 2 && data4 > data6))
  {
    // The robot move left
    for (int i = 0; i < 4; i++)
    {
      PlayFrames(numberOfFramesLeft, playbackDelayLeft, "left");
//      Serial.println("Turn Leftt");
    }
    PlayFrames(numberOfFramesNeutral, playbackDelayNeutral, "home");
  }
  delay(2000);                                    // Delay 2 sec to let BiPed adjust its servos at home position

//  Serial.print("Data3:  "); Serial.print(data3);Serial.print(" Data4:  "); Serial.print(data4);
//  Serial.print(" Data5:  "); Serial.print(data5);Serial.print(" Data6:  "); Serial.println(data6);
//  Serial.print("Ultrasonic Detect:  "); Serial.print(cm);Serial.println(" cm");
}

void PlayFrames(int numberOfFrames, int playbackDelay, String dir)
{
  // Angle value of the servo
  int value;
  // This for loop determines the animation frame the robot is playing through
  for (int framesRowNumber = 0; framesRowNumber < numberOfFrames; framesRowNumber++)
  {
    // This for loop adjusts the position of each servo
    for (int servo = 0; servo < numberOfServos; servo++)
    {
      // each servo position is sent as a 2 byte value (high byte, low byte) integer (from -32,768 to 32,767)
      // this number is encoding the angle of the servo. The number is 100 * the servo angle.  This allows for the
      // storage of 2 significant digits(i.e. the value can be from -60.00 to 60.00 and every value in between).
      // Also remember that the servos have a range of 120 degrees. The angle is written in positions
      // which range from a minimum of 800 (-60 degrees) and go to a maximum of 2200 (60 degrees)
      // This branch determines whether or not the robot is walking forward or turning left based on
      // the parameters passed in the main loop
      if (numberOfFrames == numberOfFramesForward)      value = framesForward[framesRowNumber][servo];
      else if (numberOfFrames == numberOfFramesNeutral) value = framesNeutral[framesRowNumber][servo];
      else if (numberOfFrames == numberOfFramesRight && dir == "right")   value = framesRight[framesRowNumber][servo];
      else if (numberOfFrames == numberOfFramesLeft && dir == "left")    value = framesLeft[framesRowNumber][servo];
      else if (numberOfFrames == numberOfFramesLeftLean && dir == "leftLean")   value = framesLeftLean[framesRowNumber][servo];
      else if (numberOfFrames == numberOfFramesRightLean && dir == "rightLean")  value = framesRightLean[framesRowNumber][servo];
      else value = framesForward[framesRowNumber][servo];

      // flip for the left leg.
      if(servo >= numberOfServos/2) value = map(value, -6000,6000,6000,-6000);

      // tell servo to go to position in variable 'pos'
      servos[servo].write(map(value, -6000,6000,800,2200));   
      // This delay controls the delay between each servo being updated       
      delay(2);
    }
  // This delay controls the delay between each frame
  // This will vary based on the animation and may need to be changed if you make your own
  // animation with a different speed.
  delay(playbackDelay);
  }
}

void safeRover()
{
  PlayFrames(numberOfFramesNeutral, playbackDelayNeutral, "home");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Mirror System
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void move_camera(uint8_t scanAngle, uint8_t tiltAngle)
{
  tiltAngle = ((tiltAngle/3)+tiltDownLimit);
  scanServo.write(scanAngle);
  tiltServo.write(tiltAngle);
//  Serial.print("Scan Servo move to (Angle): "); Serial.println(scanAngle);
//  Serial.print("Tilt Servo move to (Angle): "); Serial.println(tiltAngle);
}

void reset_camera()
{
  scanPosition = scanReset;       // scan servo position at reset
  scanServo.write(scanReset);
  delay(1000);
  tiltPosition = tiltReset;     // tilt servo position at reset
  tiltServo.write(tiltReset);
//  Serial.print("Scan Servo reset to (Angle): "); Serial.println(scanReset);
//  Serial.print("Tilt Servo reset to (Angle): "); Serial.println(tiltReset);
  delay(1000); 
}

void home_camera()
{
  move_camera(scanHome,tiltHome);
//  Serial.print("Scan Servo move to Home(Angle): "); Serial.println(scanHome);
//  Serial.print("Tilt Servo move to Home(Angle): "); Serial.println(tiltHome);
}

byte getScanPosition()
{
  return byte(scanPosition);
}

byte getTiltPosition()
{
  return byte(abs(tiltPosition-180));  //***********************************************
}
/* note: If needed send end-of-pan/tilt message allowing rover's orientation
 *       display to be updated. Because we are locking the playback head this
 *       may or may not required... does Android send telemetry independent of
 *       Arduino?
 */
 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Sensors
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Ultrasonic Sensor
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
long DetectDistance(int pingPin)
{
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(pingPin, INPUT);
  long duration = pulseIn(pingPin, HIGH);

  // convert the time into a distance
  long cm = duration / ultrasonicConstant;
  return cm;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Accelerometer Sensor
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int16_t DetectTorque()
{
  // Distance Measured by the accelerometer sensor MPU-6050
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
  int16_t AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)   
//  Serial.print("Accelerometer X = "); Serial.println(AcX);
  return AcX;
}

