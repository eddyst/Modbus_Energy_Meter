/* Library for reading SDM 72/120/220/230/630 Modbus Energy meters.
*  Reading via Hardware or Software Serial library & rs232<->rs485 converter
*  2016-2019 Reaper7 (tested on wemos d1 mini->ESP8266 with Arduino 1.8.10 & 2.5.2 esp8266 core)
*  crc calculation by Jaime García (https://github.com/peninquen/Modbus-Energy-Monitor-Arduino/)
*/
//------------------------------------------------------------------------------
#ifndef SDM_h
#define SDM_h

//------------------------------------------------------------------------------
#include <Arduino.h>

#define DEBUG_SERIAL Serial1                                                    //Serielle Schnittstelle für DEBUGING
#define USE_HARDWARESERIAL

//#include <SDM_Config_User.h>
#if defined ( USE_HARDWARESERIAL )
  #include <HardwareSerial.h>
#else
  #include <SoftwareSerial.h>
#endif

#define SDM_UART_BAUD                     4800                                //default baudrate
#define DERE_PIN                          NOT_A_PIN                           //default digital pin for control MAX485 DE/RE lines (connect DE & /RE together to this pin)
#if defined ( USE_HARDWARESERIAL )
  #define SDM_UART_CONFIG                 SERIAL_8N1                          //default hardware uart config
  #if defined ( ESP8266 ) && !defined ( SWAPHWSERIAL )
    #define SWAPHWSERIAL                    0                                   //(only esp8266) when hwserial used, then swap uart pins from 3/1 to 13/15 (default not swap)
  #endif
  #if defined ( ESP32 )
    #define SDM_RX_PIN                        13
    #define SDM_TX_PIN                        15
  #endif
#else
  #if defined ( ESP8266 ) || defined ( ESP32 )
    #define SDM_UART_CONFIG               SWSERIAL_8N1                        //default softwareware uart config for esp8266/esp32
    #define SDM_RX_PIN                        13
    #define SDM_TX_PIN                        15
   #else
    #define SDM_RX_PIN                        10
    #define SDM_TX_PIN                        11  
  #endif
#endif

#define MAX_MILLIS_TO_WAIT                 500                                 //default max time to wait for response from SDM
#define MAX_REGISTER_PER_REQUEST              20
#define REGISTERCOUNT                         52
#define SDM_B_02                            0x03                                //BYTE 2 -> function code (default value 0x04 read from 3X input registers)
                                                                                //BYTES 3 & 4 (BELOW)
//------------------------------------------------------------------------------
#define SDM_ERR_NO_ERROR                    0                                   //no error
#define SDM_ERR_TIMEOUT                     1                                   //timeout
#define SDM_ERR_WRONG_NODE                  3                                   //byte 0 wrong
#define SDM_ERR_WRONG_FUNCTIONCODE          4                                   //byte 1 wrong
#define SDM_ERR_CRC_ERROR                   6                                   //crc error
//------------------------------------------------------------------------------
class SDM {
  public:
#if defined ( USE_HARDWARESERIAL )                                              //hardware serial
  #if defined ( ESP8266 )                                                       //                on esp8266
    SDM(HardwareSerial& serial, long baud = SDM_UART_BAUD, int dere_pin = DERE_PIN, int config = SDM_UART_CONFIG, bool swapuart = SWAPHWSERIAL);
  #elif defined ( ESP32 )                                                       //                on esp32
    SDM(HardwareSerial& serial, long baud = SDM_UART_BAUD, int dere_pin = DERE_PIN, int config = SDM_UART_CONFIG, int8_t rx_pin = SDM_RX_PIN, int8_t tx_pin = SDM_TX_PIN);
  #else                                                                         //                on avr
    SDM(HardwareSerial& serial, long baud = SDM_UART_BAUD, int dere_pin = DERE_PIN, int config = SDM_UART_CONFIG);
  #endif
#else                                                                           //software serial
  #if defined ( ESP8266 ) || defined ( ESP32 )                                  //                on esp8266/esp32
    SDM(SoftwareSerial& serial, long baud = SDM_UART_BAUD, int dere_pin = DERE_PIN, int config = SDM_UART_CONFIG, int8_t rx_pin = SDM_RX_PIN, int8_t tx_pin = SDM_TX_PIN);
  #else                                                                         //                on avr
    SDM(SoftwareSerial& serial, long baud = SDM_UART_BAUD, int dere_pin = DERE_PIN);
  #endif
#endif
    virtual ~SDM();
    void begin(void);
    enum RegisterTyp {
      BIT1_Faktor10_0,
      BIT1_Faktor10_1,
      BIT1_Faktor10_2,
      BIT2_Faktor10_2
    };
    float readVal(uint8_t node, uint16_t reg, RegisterTyp typ);                                                //read value from register = reg 
    uint16_t getErrCode(bool _clear = false);                                   //return last errorcode (optional clear this value, default flase)
    uint16_t getErrCount(bool _clear = false);                                  //return total errors count (optional clear this value, default flase)
    uint16_t getSuccCount(bool _clear = false);                                 //return total success count (optional clear this value, default false)
    void clearErrCode();                                                        //clear last errorcode
    void clearErrCount();                                                       //clear total errors count
    void clearSuccCount();                                                      //clear total success count
    uint16_t maxAge = 10000;
  private:
#if defined ( USE_HARDWARESERIAL )
    HardwareSerial& sdmSer;
#else
    SoftwareSerial& sdmSer;
#endif

#if defined ( USE_HARDWARESERIAL )
    int _config = SDM_UART_CONFIG;
  #if defined ( ESP8266 )
    bool _swapuart = SWAPHWSERIAL;
  #elif defined ( ESP32 )
    int8_t _rx_pin = -1;
    int8_t _tx_pin = -1;
  #endif
#else
  #if defined ( ESP8266 ) || defined ( ESP32 )
    int _config = SDM_UART_CONFIG;
  #endif
    int8_t _rx_pin = -1;
    int8_t _tx_pin = -1; 
#endif
    long _baud = SDM_UART_BAUD;
    int _dere_pin = DERE_PIN;
    uint8_t       registerArrayStart;
    unsigned long registerArrayTime;
    uint8_t       registerArray[3 + MAX_REGISTER_PER_REQUEST * 2 + 2];
    void readRegistersFromDevice();
    uint16_t readingerrcode = SDM_ERR_NO_ERROR;                                 //4 = timeout; 3 = not enough bytes; 2 = number of bytes OK but bytes b0,b1 or b2 wrong, 1 = crc error
    uint16_t readingerrcount = 0;                                               //total errors counter
    uint32_t readingsuccesscount = 0;                                           //total success counter
    uint16_t calculateCRC(uint8_t *array, uint8_t num);
    void flush();                                                               //read serial if any old data is available
    void dereSet(bool _state = LOW);                                            //for control MAX485 DE/RE pins, LOW receive from SDM, HIGH transmit to SDM
};
#endif //SDM_h
