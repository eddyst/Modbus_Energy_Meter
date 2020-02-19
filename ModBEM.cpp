/* Library for reading Modbus Energy meters.
*  Reading via Hardware or Software Serial library & rs232<->rs485 converter
*  2016-2019 Reaper7 (tested on wemos d1 mini->ESP8266 with Arduino 1.8.10 & 2.5.2 esp8266 core)
*  crc calculation by Jaime García (https://github.com/peninquen/Modbus-Energy-Monitor-Arduino/)
*/
//------------------------------------------------------------------------------
#include "ModBEM.h"

//------------------------------------------------------------------------------
#if defined ( USE_HARDWARESERIAL )
#if defined ( ESP8266 )
ModBEM::ModBEM(HardwareSerial& serial, long baud, int dere_pin, int config, bool swapuart) : ModBEMSer(serial) {
  this->_baud = baud;
  this->_dere_pin = dere_pin;
  this->_config = config;
  this->_swapuart = swapuart;
}
#elif defined ( ESP32 )
ModBEM::ModBEM(HardwareSerial& serial, long baud, int dere_pin, int config, int8_t rx_pin, int8_t tx_pin) : ModBEMSer(serial) {
  this->_baud = baud;
  this->_dere_pin = dere_pin;
  this->_config = config;
  this->_rx_pin = rx_pin;
  this->_tx_pin = tx_pin;
}
#else
ModBEM::ModBEM(HardwareSerial& serial, long baud, int dere_pin, int config) : ModBEMSer(serial) {
  this->_baud = baud;
  this->_dere_pin = dere_pin;
  this->_config = config;
}
#endif
#else
#if defined ( ESP8266 ) || defined ( ESP32 )
ModBEM::ModBEM(SoftwareSerial& serial, long baud, int dere_pin, int config, int8_t rx_pin, int8_t tx_pin) : ModBEMSer(serial) {
  this->_baud = baud;
  this->_dere_pin = dere_pin;
  this->_config = config;
  this->_rx_pin = rx_pin;
  this->_tx_pin = tx_pin;
}
#else
ModBEM::ModBEM(SoftwareSerial& serial, long baud, int dere_pin) : ModBEMSer(serial) {
  this->_baud = baud;
  this->_dere_pin = dere_pin;
}
#endif
#endif

ModBEM::~ModBEM() {
}

void ModBEM::begin(void) {
#if defined ( USE_HARDWARESERIAL )
  #if defined ( ESP8266 )
    ModBEMSer.begin(_baud, (SerialConfig)_config);
  #elif defined ( ESP32 )
    ModBEMSer.begin(_baud, _config, _rx_pin, _tx_pin);
  #else
    ModBEMSer.begin(_baud, _config);
  #endif
#else
  #if defined ( ESP8266 ) || defined ( ESP32 )
    ModBEMSer.begin(_baud, (SoftwareSerialConfig)_config, _rx_pin, _tx_pin);
  #else
    ModBEMSer.begin(_baud);
  #endif
#endif

#if defined ( USE_HARDWARESERIAL ) && defined ( ESP8266 )
  if (_swapuart)
    ModBEMSer.swap();
#endif
  if (_dere_pin != NOT_A_PIN) {
    pinMode(_dere_pin, OUTPUT);                                                 //set output pin mode for DE/RE pin when used (for control MAX485)
  }
  dereSet(LOW);                                                                 //set init state to receive from ModBEM -> DE Disable, /RE Enable (for control MAX485)
}


float ModBEM::readVal(uint8_t node, uint16_t reg, ModBEM::RegisterTyp typ) {
  #ifdef DEBUG_SERIAL
    DEBUG_SERIAL.print("ReadVal node:"); DEBUG_SERIAL.print(node); DEBUG_SERIAL.print(" reg "); DEBUG_SERIAL.println(reg);
  #endif
  float res = NAN;
  if (   millis() - registerArrayTime >= maxAge 
      or not node == registerArray[0] 
      or reg < registerArrayStart 
      or reg >= registerArrayStart + MAX_REGISTER_PER_REQUEST
      or (typ == BIT2_Faktor10_2 and reg + 1 >= registerArrayStart + MAX_REGISTER_PER_REQUEST)
     ) {    //Wenn im Bereich und nicht zu alt dann
    #ifdef DEBUG_SERIAL
      DEBUG_SERIAL.print(" reload");
    #endif
    registerArrayTime = millis();
    registerArrayStart = reg;
    uint16_t temp;
    uint16_t FRAMESIZE_IN;
    unsigned long resptime;
    ErrorCodes readErr = ERR_NO_ERROR;
    
    FRAMESIZE_IN = REGISTERCOUNT - registerArrayStart;
    if (FRAMESIZE_IN > 20)
      FRAMESIZE_IN = 20;
    registerArray[0] = node;
    registerArray[1] = ModBEM_B_02;
    registerArray[2] = highByte(registerArrayStart);
    registerArray[3] = lowByte(registerArrayStart);
    registerArray[4] = highByte(FRAMESIZE_IN);
    registerArray[5] = lowByte(FRAMESIZE_IN);
    temp = calculateCRC(registerArray, 6);                                   //calculate out crc only from first 6 bytes
    registerArray[6] = lowByte(temp);
    registerArray[7] = highByte(temp);

    #if !defined ( USE_HARDWARESERIAL )
      ModBEMSer.listen();                                                              //enable softserial rx interrupt
    #endif
    flush();                                                                      //read serial if any old data is available
    dereSet(HIGH);                                                                //transmit to ModBEM  -> DE Enable, /RE Disable (for control MAX485)
    delay(2);                                                                     //fix for issue (nan reading) by sjfaustino: https://github.com/reaper7/SDM_Energy_Meter/issues/7#issuecomment-272111524
    ModBEMSer.write(registerArray, 8);                                              //send 8 bytes
    ModBEMSer.flush();                                                               //clear out tx buffer
    dereSet(LOW);                                                                 //receive from ModBEM -> DE Disable, /RE Enable (for control MAX485)

    resptime = millis() + MAX_MILLIS_TO_WAIT;
    while (ModBEMSer.available() < 3 + 2*FRAMESIZE_IN + 2) {
      if (resptime < millis()) {
        readErr = ERR_TIMEOUT;
        #ifdef DEBUG_SERIAL
          DEBUG_SERIAL.print("Timeout: "); DEBUG_SERIAL.print(ModBEMSer.available()); DEBUG_SERIAL.print(" von "); DEBUG_SERIAL.print(5+2*FRAMESIZE_IN); DEBUG_SERIAL.println(" Zeichen verfügbar");
        #endif
        break;
      }
      yield();
    }
    if (readErr == ERR_NO_ERROR) {                                            //if no timeout...
      for(int n=0; n<3 + 2*FRAMESIZE_IN + 2; n++) {
        registerArray[n] = ModBEMSer.read();
        #ifdef DEBUG_SERIAL
          DEBUG_SERIAL.print(n);
          DEBUG_SERIAL.print(":");
          DEBUG_SERIAL.print(registerArray[n]);
          DEBUG_SERIAL.print(" ");
        #endif
      }
      #ifdef DEBUG_SERIAL
        DEBUG_SERIAL.println();
      #endif
       if (registerArray[0] == node) {
        if (registerArray[1] == ModBEM_B_02 ) {
          if ((calculateCRC(registerArray, 3 + registerArray[2])) == ((registerArray[(2 + registerArray[2] + 2)] << 8) | registerArray[(2 + registerArray[2] + 1)])) {  //calculate crc for recived bytes and compare with received crc (last two)
            //everything OK, nothing to do
          } else {
            readErr = ERR_CRC_ERROR;
            #ifdef DEBUG_SERIAL
              DEBUG_SERIAL.print("ERR_CRC_ERROR");
              DEBUG_SERIAL.print(3+ registerArray[2]);
              DEBUG_SERIAL.print(",");
//              DEBUG_SERIAL.print(calculateCRC(registerArray, 1 + registerArray[2]));
//              DEBUG_SERIAL.print(",");
//              DEBUG_SERIAL.print(calculateCRC(registerArray, 2 + registerArray[2]));
//              DEBUG_SERIAL.print(",");
              DEBUG_SERIAL.print(calculateCRC(registerArray, 3 + registerArray[2]));
              DEBUG_SERIAL.print(":");
              DEBUG_SERIAL.print(registerArray[(2 + registerArray[2] + 1)]);
              DEBUG_SERIAL.print(",");
              DEBUG_SERIAL.print(registerArray[(2 + registerArray[2] + 2)]);
              DEBUG_SERIAL.print("=");
              DEBUG_SERIAL.print(((registerArray[(2 + registerArray[2] + 2)] << 8) | registerArray[(2 + registerArray[2] + 1)]));
              DEBUG_SERIAL.print(" ");
            #endif
          }
        } else {
          readErr = ERR_WRONG_FUNCTIONCODE;
          #ifdef DEBUG_SERIAL
            DEBUG_SERIAL.println("ERR_WRONG_FUNCTIONCODE");
          #endif
        }
      } else {
        readErr = ERR_WRONG_NODE;
        #ifdef DEBUG_SERIAL
          DEBUG_SERIAL.println("ERR_WRONG_NODE");
        #endif
      }
    }
    flush();                                                                      //read serial if any old data is available
    #if !defined ( USE_HARDWARESERIAL )
      ModBEMSer.stopListening();                                                     //disable softserial rx interrupt
    #endif

    if (readErr != ERR_NO_ERROR) {                                            //if error then copy temp error value to global val and increment global error counter
      readingerrcode = readErr;
      readingerrcount++;
      registerArray[0] = 0;                                                        //to start a new read next time
      return (NAN);
    } else {
      ++readingsuccesscount;
    }
  }
  #ifdef DEBUG_SERIAL
    DEBUG_SERIAL.print("res = ");
    DEBUG_SERIAL.print(2 + (reg - registerArrayStart)*2 + 1);
    DEBUG_SERIAL.print(", typ = ");
    DEBUG_SERIAL.println(typ);
  #endif
  res = registerArray[2 + (reg - registerArrayStart)*2 + 1]*256 + registerArray[2 + (reg - registerArrayStart)*2 + 2];
  switch (typ) {
    case BIT1_Faktor10_0:
      break;
    case BIT1_Faktor10_1:
      res  /= 10;
      break;
    case BIT1_Faktor10_2:
      res /= 100;
      break;
    case BIT2_Faktor10_2:
      res *= 65536;
      res += registerArray[2 + (reg - registerArrayStart)*2 + 3]*256 + registerArray[2 + (reg - registerArrayStart)*2 + 4];
      break;
    default:
      #ifdef DEBUG_SERIAL
        DEBUG_SERIAL.println("\nunbekannter Typ\n");
      #endif
      res=NAN;
      break;
  }
  return (res);
}

ModBEM::ErrorCodes ModBEM::getErrCode(bool _clear) {
  ErrorCodes _tmp = readingerrcode;
  if (_clear == true)
    clearErrCode();
  return (_tmp);
}

uint16_t ModBEM::getErrCount(bool _clear) {
  uint16_t _tmp = readingerrcount;
  if (_clear == true)
    clearErrCount();
  return (_tmp);
}

uint16_t ModBEM::getSuccCount(bool _clear) {
  uint16_t _tmp = readingsuccesscount;
  if (_clear == true)
    clearSuccCount();
  return (_tmp);
}

void ModBEM::clearErrCode() {
  readingerrcode = ERR_NO_ERROR;
}

void ModBEM::clearErrCount() {
  readingerrcount = 0;
}

void ModBEM::clearSuccCount() {
  readingsuccesscount = 0;
}

uint16_t ModBEM::calculateCRC(uint8_t *array, uint8_t num) {
  uint16_t _crc, _flag;
  _crc = 0xFFFF;
  for (uint8_t i = 0; i < num; i++) {
    _crc = _crc ^ array[i];
    for (uint8_t j = 8; j; j--) {
      _flag = _crc & 0x0001;
      _crc >>= 1;
      if (_flag)
        _crc ^= 0xA001;
    }
  }
  return _crc;
}

void ModBEM::flush() {
  uint8_t _i = 0;
  while (ModBEMSer.available() && _i++ < 10)  {                                    //read serial if any old data is available
    ModBEMSer.read();
    delay(1);
  }
}

void ModBEM::dereSet(bool _state) {
  if (_dere_pin != NOT_A_PIN)
    digitalWrite(_dere_pin, _state);                                            //receive from ModBEM -> DE Disable, /RE Enable (for control MAX485)
}
