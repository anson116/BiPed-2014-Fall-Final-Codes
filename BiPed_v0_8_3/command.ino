/******************** Command Acquisitions *******************
 * When using Bluetooth or OTG this is where the Command ID  * 
 * and associated bytes are obtained. 
 * Packet Structure based on CCSDS Standards
 * http://public.ccsds.org/publications/archive/102x0b5s.pdf *
 *
 * State Byte   
 *       Index  Bits  Description        Default            MOV example
 *   0   0       3    3.1.1 Version Number 101       
 *               3    3.1.2 Packet ID         001
 *               2    Reserved                   01         data[0] = 0xA5                      
 *   1   1       4    Reserved             000      
 *               4    3.1.4 Packet Length     00001 - 10100 data[1] = 0x05 Bytes in Command Data Field
 *                                                                         excluding checksum
 *   2   2            3.2 Command Data Packet Field         data[2] = 0x01 MOV command
 *       :                                                       :
 *       N+1          where N = 00001 - 10100               data[6] = 4th parameter
 *   3   N+2          checksum                              data[7] = longitudinal redundancy check (LRC)
 *                    warning: does not protect against byte reversals http://en.wikipedia.org/wiki/Checksum
 *   4                Error State 
 ************************************************************/
 
 /********************** Exception Codes ********************
 *   01    Start byte 0xA5 expected
 *   02    Packet length out of range 1 - 20
 *   03    LRC checksum error
 *   04    Undefined command decoder FSM state
 *   05    Array out of range i >= 23
 ************************************************************/

/* Public Function */
void commandDecoder()
{
  static uint8_t data[23];     // 23 bytes = ID + Length + CMD + maximum number of parameters 19 + Checksum
                               // EEPROM read/write maximum block size = 16
                               // maximum index 22
  static uint8_t state = 0;    // initially in state 
  static uint8_t i = 0;        // index pointer for data array   
  static uint8_t N = 0;        // bytes in the Command Data Packet Field nnnn
  static uint8_t checksum = 0; // LRC byte
  uint8_t next_state;          // next state

  // Step through FSM until buffer is empty
  #if bluetooth
  while(Serial1.available()){  // note: Leonardo does not support serialEvent() handler
    data[i] = Serial1.read();
  #else
  while(Serial.available()){
    data[i] = Serial.read();
  #endif
  
  //FSM implementation
  // Moore output decoder section
  checksum ^= data[i];      // includes even parity byte

  // Next State Decoder with Mealy output decoder section
  switch (state)  // set dir = motordata[1] for motor A (left)
  {
    case 0:   // Start Byte
    if (data[0] != COMMAND_PACKET_ID)
    {
      // start code 0xA5, test code 0x73 = 's'
      throw_error(word(0x01,data[0]));
      i = 0;
      checksum = 0;
      next_state = 4;  // exception
    }
    else
    {
      i = 1;           // or i++;
      next_state = 1;  // read packet length
    }
    break;

    case 1:  // Packet Length
      N = data[1];
      if (0 < N && N <=20)
      {
        i = 2;          // or i++;
        next_state = 2;
      }
      else
      {
      throw_error(word(0x02,N));
      i = 0;
      checksum = 0;
      next_state = 4;  // exception
      }
      break;
      
    case 2:  // Read Command Data Packet Field
      i++;
      if (i < N+2) next_state = 2;
      else next_state = 3;
      break;
      
    case 3:  // Checksum
      if (checksum == 0)
      {
        commandHandler(data, N); // Go To function where you run robot code based on Command Received.
        i = 0;
        checksum = 0;
        next_state = 0;
      }
      else
      {
        throw_error(word(0x03,checksum));
        i = 0;
        checksum = 0;
        next_state = 4;  // exception
      }
      break;
      
    case 4:  // Exception Handler
      // destroy data until start byte received
      if (data[0] != COMMAND_PACKET_ID)
      {
        // start code 0xA5, test code 0x73 = 's'
        i = 0;
        checksum = 0;
        next_state = 4;
      }
      else
      {
        i = 1;             // or i++
        next_state = 1;
      }
      break;
            
    default:
      throw_error(word(0x04,data[i-1]));
      i = 0;
      checksum = 0;
      next_state = 4;  // exception
    } // end swirch-case
    
    /* Array out of bounds check
     * 1. The packet length test N precludes this error from ever occuring (at least in theory)   
     * 2. Within this serial.read while loop, when the array index equals 22 
     *    the FSM must be in state 3 (checksum).  
     */
    if (i > 22){
      throw_error(word(0x05,data[22]));
      checksum = 0;
      i = 0;
      next_state = 4;           // FSM exception
    }
    // clock FSM
    state = next_state;
  } // end while
}

/* Test Sequences
 * i =  0  1  2  3  4  5  6  7
 * N =        1  2  3  4  5
 *     A5 05 01 01 80 01 80 A1   MOV forward half speed
 *
 * i =  0  1  2  3
 * N =        1
 *     A5 01 04 A0  Camera Home
 *     A5 01 05 A1  Camera Reset
 *     A5 01 08 AC  Safe Rover
 *     A5 01 0A AE  Sleep
 *     A5 01 0B AF  Wake
 * 
 * i =  0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22
 * N =        1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20
 *     A5 14 07 00 00 10 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F 10 B6
 * Write a 16 byte block to EEPROM
 * Command Packet ID A5
 * N = 20 or 0x14
 * EEPROM Write Command  07 
 * EEPROM Address 0x0000 00 00 
 * Number of Bytes 16    10
 * Data  01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F 10
 * Parity Byte B6
 *
 * i =  0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22
 * N =        1  2  3  4 
 *     A5 04 06 00 00 10 B7
 * Read a 16 byte block from EEPROM starting at address 0x0000
 * Command Packet ID A5
 * Data field length N = 4 
 * EEPROM Read Command 06
 * EEPROM Address 0x0000 00 00 
 * Number of Bytes 16    10
 * Parity Byte B7
 
 * EEPROM Response Example
 * i =  0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22
 * N =        1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 
 *     CA 11 0A 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F 10 
 *                                                               | <- checksum
 * Telemetry Packet ID   CA
 * N = 17 or 0x11 response does not include address or number of bytes, 
 *                which was included in the request.
 * EEPROM Read Response  0A
 * Data  01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F 10 - assuming this data was written to EEPROM
 * Parity Byte C1 
 */

/* Private Function(s) */
 /**********************Robot Commands ***********************
 * Robot code is added here. Based on Command ID in data[0]  *
 *      and associated bytes, command received is known      *
 *                     Phone to Arduino                      *
 *************************************************************/ 
void commandHandler(uint8_t * data, uint8_t N)
{
  // EEPROM
  uint16_t  eeprom_address;
  uint8_t   eeprom_data[16];        // 16 bytes maximum
  uint8_t cmd = data[2];

  #if debug
  command_echo(data, N);
  digitalWrite(led, cmd & 0x01);   // turn LED ON/OFF based on command qualifier bit **** 
  #endif
 
  if (cmd == MOVE) 
  {
    /***********************************
     * motion command = 0x01
     * motordata[3]   left run    (FORWARD = index 1, BACKWARD = index 2, BRAKE = index 3, RELEASE = index 4)
     * motordata[4]   left speed  0 - 255
     * motordata[5]   right run   (FORWARD, BACKWARD, BRAKE, RELEASE) 
     * motordata[6]   right speed 0 - 255
     * example
     * forward half speed  0x01, 0x01, 0x80, 0x01, 0x80 0101800180
        ***********************************/

    /*
    WARNING
    WARNING ONLY THE TB6612FNG HAS BEEN UPDATED WITH NEW INDEX ADDRESSES
    WARNING Typically all you need to do is add 2 to the index (1 -> 3, 2 -> 4, etc.)
    */
        
    #if BiPed == TRUE
    // ROFIAFinal
    move_BiPed(data[3],data[4],data[5],data[6]);
    #endif
  }

  else if (cmd == CAMERA_MOVE)
  {
    /***********************************
     * pan and tilt command 
     * data[3]    0
     * data[4]    pan degrees (0 to 180)
     * data[5]    0
     * data[6]    tilt degrees (0 to 180)
     ***********************************/  
    #if BiPed
    move_camera(data[4],data[6]);
    #endif

//    Serial.print("Pan Servo To Position: ");
//    Serial.print(data[4]);
//    Serial.print(", ");
//    Serial.println(data[6]);
  }
  else if (cmd == CAMERA_RESET)
  {
    /***********************************
     * camera home command 
     * pan position = 90 (default), tilt position = 90 (default)
     ***********************************/ 
    #if BiPed
    reset_camera();
    #endif
//    Serial.println("Camera Home");
 }
 else if (cmd == CAMERA_HOME)
 {
   /***********************************
    * camera home command 
    * pan position = 90 (default), tilt position = 90 (default)
    ***********************************/ 
   #if BiPed
   home_camera();
   #endif
   // Serial.println("Camera Home");
 }
 else if (cmd == READ_EEPROM)
 {
   /***********************************
    * EEPROM
    * Telemetry Table = Robot Capabilities Worksheet
    * data[3]    Address High
    * data[4]    Address Low
    * data[5]    Number Bytes
    * i =  0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22
    * N =        1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20
    ***********************************/ 
   eeprom_address = word(data[3],data[4]); 
   for (int i=0; i<data[5]; i++)
   {
     eeprom_data[i] = EEPROM.read(eeprom_address);
     eeprom_address++;
   }     
      sendPacket(EEPROM_RESPONSE_ID, eeprom_data, data[5]);
 }
 else if (cmd == WRITE_EEPROM)
 {
   /***********************************
    * EEPROM                                   Test
    * data[3]    Address High                   00
    * data[4]    Address Low                    00
    * data[5]    Number Bytes                   10
    * data[6]    Data (Most Significant Byte)   see test data below
    * data[7]    ...
    * data[N+1]    Data (Least Significant Byte) N 
    * i =  0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22
    * N =        1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20
    *                        |
    * test data              1  2  3  4  5  6  7  8  9  A  B  C  D  E  F 10   
    ***********************************/
    eeprom_address = word(data[3],data[4]); 
    for (int i=6; i<data[5]+6; i++)
    {
      EEPROM.write(eeprom_address, data[i]);
      eeprom_address++;
      delay(4);  // An EEPROM write takes 3.3 ms to complete. (optional)
    }
//    Serial.println("Write EEPROM");
 }
 else if (cmd == SAFE_ROVER)
 {
   /***********************************
    * safe rover
    * motors to idle
    ***********************************/ 
   safeRover();
 }
 else if (cmd == PING)
 {
   // ##### JEFF CHANGED
   handlePing();
   // sendPacket(PONG_ID);
   // next_ping = millis() + ping_interval;  // reset watchdog time out ****  
   // sendPacket(PONG_ID);
   // #####
 }
 /***********************************
  * Set Ping Interval 
  * data[3]    most significant byte
  * data[4]
  * data[5]
  * data[6]    least significant byte
  ***********************************/ 
 else if (cmd == PING_INTERVAL)
 {
   // ##### JEFF CHANGED
   // ping_interval = (unsigned long)(data[3],data[4],data[5],data[6]);
   ping_interval = (((unsigned long)data[3]) << 24) | (((unsigned long)data[4]) << 16) | (((unsigned long)data[5]) << 8) | (unsigned long)data[6];
   // (((unsigned long)buffin[0]) << 24) | (((unsigned long)buffin[1]) << 16) | (((unsigned long)buffin[2]) << 8) | (unsigned long)buffin[3];
   // ## TESTING
   /*
   #if bluetooth
   Serial.print("ping_interval set to: ");
   Serial.println(ping_interval, DEC); 
   #endif
   */
   // ##
   handlePing();
   // sendPacket(PONG_ID);
   // #####
 }   
}

void throw_error(uint16_t err)
{
  sendWordPacket(EXCEPTION_ID,err);

  #if bluetooth                                    // if packet sent over USART=>bluetooth,  
  Serial.print("Command decoder exception 0x0");   // send duplicate data as text to
  Serial.println(err,HEX);                         // USB=>Arduino IDE Serial Monitor. 
  #endif
}


void command_echo(uint8_t * data, uint8_t N)
{ 
                              // i = 0     1     2  ...    N + 2 
  uint8_t data_out[N];        // N =             1 .. N 
  for(int i=2; i<N+2; i++)
  {
                              //    A5, Length,          , Checksum;
    data_out[i-2] = data[i];  //               [ID, Data]
  }
  sendPacket(COMMAND_ECHO_ID, data_out, N);  // send packet over USB or bluetooth
  
  #if bluetooth                              // if packet sent over USART=>bluetooth,
  Serial.print("rover received command: ");  // send duplicate data as text to
  Serial.print(data[2], HEX);                // USB=>Arduino IDE Serial Monitor. 
  for(int i=3; i<N+2; i++)
  {
    Serial.print(", ");
    Serial.print(data[i], HEX);
  }
  Serial.println(); 
  #endif
}

unsigned long getNextPing(){
  return next_ping;
}

void updateNextPing(){
  next_ping = millis() + ping_interval;  // reset watchdog time out ****  
}

void handlePing(){
  updateNextPing();
  sendPacket(PONG_ID);
}
/*************************************************************
 * You can read more about the CCSDS standards at the
 * following links.
 * http://public.ccsds.org/default.aspx
 * Packet Structure based on CCSDS Standards
 * http://public.ccsds.org/publications/archive/735x1b1.pdf
 * telecommand
 * http://public.ccsds.org/publications/archive/202x1b2s.pdf
 * http://public.ccsds.org/publications/archive/200x0g6.pdf
 * telemetry packet
 * http://public.ccsds.org/publications/archive/102x0b5s.pdf
 *************************************************************/

