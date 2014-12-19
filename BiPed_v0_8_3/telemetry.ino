 /******************* Emergency Flags ****************
 * source: Found in CommunicationRobotPilot Folder
 ***************************************************/
/*
all clear = 0 or null,
set = One Byte combinable flags bits{
 1 = latency           0b00000001
 2 = obstacle,         0b00000010
 4 = stall,            0b00000100
 8 = unsafe position,  0b00001000
 16 = battery,         0b00010000
 32 = temperature      0b00100000
 64 = command time-out ==> from arduino
*/

/********************** Telemetry Packet *********************
 * When using Bluetooth or OTG this is where the Command ID  * 
 * and associated bytes are obtained. 
 * Packet Structure based on CCSDS Standards
 * http://public.ccsds.org/publications/archive/102x0b5s.pdf *
 *
 * State Byte   
 *       Index  Bits  Description        Default            EEPROM Example
 *   0   0       3    3.1.1 Version Number 110       
 *               3    3.1.2 Packet ID         010
 *               2    Reserved                   10         data[0] = 0xCA                      
 *   1   1       4    Reserved             000      
 *               4    3.1.4 Packet Length     00001 - 10100 data[1] = 0x14 Bytes in Telemetry Data Field
 *                                                                         excluding checksum
 *   2   2            3.2 Telemetry Data Packet Field       data[2] = 0x0A EEPROM response
 *       :                                                       :
 *       N+1          where N = 00001 - 10100               data[21] = 16th parameter
 *   3   N+2          checksum                              data[22] = longitudinal redundancy check (LRC)
 *                    warning: does not protect against byte reversals http://en.wikipedia.org/wiki/Checksum
 *   4                Error State 
 ************************************************************/

 /************************  Send Data ************************
 *   This is where sensor value is checked and if different  *
 *  a data packet of three bytes (Command ID and associated) *
 *                is sent from Arduino to Phone              *
 *************************************************************/
void sendData()
{
  uint16_t data;
  int16_t sensor_value;
  static uint16_t pingRangeL=0;
  const  int16_t tolerance = 4;    
  
  #if BiPed
  sensor_value = getScanPosition();
  if (sensor_value != positionScan)
  {
    positionScan = sensor_value;          // update
    sendWordPacket(0x08, sensor_value);
//    Serial.print("Scan Servo Value: "); Serial.println(sensor_value);
  }
  sensor_value = getTiltPosition();
  if (sensor_value != positionTilt)
  {
    positionTilt = sensor_value;          // update
    sendWordPacket(0x09, sensor_value);
//    Serial.print("Tilt Servo Value: "); Serial.println(sensor_value);
  }
  #endif

  data = 200;
  if(abs(data-pingRangeL) > tolerance)
  {
    pingRangeL=data;     
    sendWordPacket(RANGE_LEFT_ID, pingRangeL); // distance sensor data package
//    Serial.print("UltraSonic : "); Serial.println(data);
  }
}

/* EEPROM Response Example
 * i =  0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22
 * test data     1  2  3  4  5  6  7  8  9  A  B  C  D  E  F 10
 *                                                               | <- checksum
 * note: response does not include address or number of bytes, 
 *       which was included in the request.
 */   

void sendPacket(uint8_t id,  uint8_t data_field[], uint8_t N)  // alt. uint8_t * data_field
{
  static uint8_t data[23];  // 23 bytes = ID + Length + SENSOR + maximum number of parameters 19 + Checksum
                            // EEPROM read/write maximum block size = 16
                            // maximum index 22
  uint8_t datum;
  uint8_t checksum = 0;     // LRC byte

  data[0] = TELEMETRY_PACKET_ID;
  data[1] = N+1;            // id + eeprom data
  data[2] = id;
  checksum = data[0] ^ data[1] ^ data[2];  // update LRC
  for(uint8_t i = 0; i < N; i++){
   datum = data_field[i];
   data[i+3] = datum;
   checksum ^= datum;
  }
  data[N+3] = checksum;      // N+3 = last byte address
  
  #if bluetooth
  Serial1.write(data,N+4);   // N+4 = number of bytes to send
  #else
  Serial.write(data,N+4);
  #endif
}

void sendPacket(uint8_t id)  // alt. uint8_t * data_field
{
  uint8_t data[4];  // 4 bytes = ID + Length + id + Checksum
  data[0] = TELEMETRY_PACKET_ID;
  data[1] = 1;             // id only
  data[2] = id;
  data[3] = data[0] ^ data[1] ^ data[2];  // update LRC
  
  #if bluetooth
  Serial1.write(data,4);   // 4 = number of bytes to send
  #else
  Serial.write(data,4);
  #endif
}

// ****** Overload as sendPacket ********
void sendWordPacket(uint8_t id, uint16_t value)
{
  uint8_t wordData [2] = {highByte(value), lowByte(value)};
  sendPacket(id, wordData, 2);    // bug fix: was 3 in version v7
  
  // sendPacket(id, uint8_t [2] {highByte(value),lowByte(value)},  3);
}
