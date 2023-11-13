#ifndef __WICKED_DEVICE_K30__
#define __WICKED_DEVICE_K30__

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <HardwareSerial.h>
#include <stdint.h>

class K30 {
public:
  K30(HardwareSerial * serial);
  K30(SoftwareSerial * swserial);

  void setSerial(HardwareSerial *serial);
  void setSerial(SoftwareSerial *serial);

  boolean getSample(float * result);
  boolean readABCPeriod(uint32_t * result);
  boolean enableABC(uint16_t periodInHours); // 180 is the default
  boolean disableABC(void);

private:
  HardwareSerial * hwserial;
  SoftwareSerial * swserial;
  static uint16_t wCRCTable[];

  void resetSerial();
  void endSerial();
  uint8_t read();
  void write(uint8_t * value, int size);
  void write(uint8_t value);
  
  uint16_t co2_CRC16 (const uint8_t *nData, uint16_t wLength);
  boolean co2ValidResponse(uint8_t * response);
  boolean readABCValidResponse(uint8_t * response);
  uint16_t co2GetValue(uint8_t * response);
  uint32_t abcGetValue(uint8_t * response);
  boolean co2ConsumeResponse(uint8_t * response, uint8_t numResponseBytesExpected = 7);
  boolean co2SendRequest(uint8_t * request, uint8_t numRequestBytes = 7);
  void clearCO2SerialInput(void);
  boolean requestCO2Data(float * co2_ppm);  
  boolean co2ResponseMatchesRequest(uint8_t * request, uint8_t * response, uint8_t num);
};

#endif
