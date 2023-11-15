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

  boolean calibrateToBackground(void);
  boolean calibrateToNitrogen(void);

private:
  HardwareSerial * hwserial;
  SoftwareSerial * swserial;
  static uint16_t wCRCTable[];
  boolean serialBeginCalled;

  void resetSerial();
  void endSerial();
  int read();
  int available();
  void beginSerial();
  void write(uint8_t * value, int size);
  void write(uint8_t value);
  boolean requestResponse(uint8_t * request, uint8_t requestLength, uint8_t * response, uint8_t expectedResponseLength);
  boolean requestResponseOnce(uint8_t * request, uint8_t requestLength, uint8_t * response, uint8_t expectedResponseLength);

  uint16_t extractUint16Field(uint8_t * response, uint8_t byteOffset);
  void clearCO2SerialInput(void);
  uint16_t co2_CRC16 (const uint8_t *nData, uint16_t wLength);
  boolean co2ResponseMatchesRequest(uint8_t * request, uint8_t * response, uint8_t num);

  boolean clearCalibrationAcknowledgements(void);
  boolean startBackgroundCalibration(void);
  boolean startNitrogenCalibration(void);
  boolean checkBackgroundCalibrationComplete(boolean * status);
  boolean checkNitrogenCalibrationComplete(boolean * status);
};

#endif
