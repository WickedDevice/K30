// #define CO2_ECHO_LOW_LEVEL_RESPONSES
// #define CO2_ECHO_LOW_LEVEL_REQUESTS
#include "K30.h"

K30::K30(HardwareSerial * hwserial){
  serialBeginCalled = false;
  this->swserial = NULL;
  this->hwserial = hwserial;
}

K30::K30(SoftwareSerial * swserial){
  serialBeginCalled = false;
  this->swserial = swserial;
  this->hwserial = NULL;
}

void K30::setSerial(HardwareSerial * serial) {
  this->swserial = NULL;
  this->hwserial = serial;
}

void K30::setSerial(SoftwareSerial * serial) {
  this->swserial = serial;
  this->hwserial = NULL;
}

boolean K30::getSample(float * result){
  uint8_t request[7] = {0xFE, 0x44, 0x00, 0x08, 0x02, 0x9F, 0x25};
  uint8_t response[7] = {0,0,0,0,0,0,0};
  boolean ret = requestResponse(request, 7, response, 7);
  
  // Serial.println();
  // Serial.print("CO2 Read Response: ");
  // for (uint8_t ii = 0; ii < 7; ii++) {
  //   Serial.printf("%02x ", response[ii]);
  // }
  // Serial.println();

  if (ret && (result != NULL)) {
    *result = 1.0f * extractUint16Field(&(response[0]), 3);
  }
  return ret;
}

boolean K30::readABCPeriod(uint32_t * result) {
  uint8_t request[8] = {0xFE, 0x03, 0x00, 0x1F, 0x00, 0x01, 0xA1, 0xC3};
  uint8_t response[7] = {0,0,0,0,0,0,0};
  boolean ret = requestResponse(request, 8, response, 7);
  if (ret && (result != NULL)) {
    *result = (uint32_t) extractUint16Field(&(response[0]), 3);
  }
  return ret;  
}

boolean K30::enableABC(uint16_t periodInHours) {
  uint8_t request[8] = {0xFE, 0x06, 0x00, 0x1F, 0x00, 0x00, 0xAC, 0x03};
  uint8_t response[8] = {0,0,0,0,0,0,0,0};

  uint8_t highByte = (periodInHours >> 8) & 0xFF;
  uint8_t lowByte = periodInHours & 0xFF;
  request[4] = highByte;
  request[5] = lowByte;
 
  boolean ret = requestResponse(request, 8, response, 8);  
  if (ret) {
    ret = co2ResponseMatchesRequest(request, response, 8);
  }  
  return ret;
}

boolean K30::disableABC() {
  uint8_t request[8] = {0xFE, 0x06, 0x00, 0x1F, 0x00, 0x00, 0xAC, 0x03};
  uint8_t response[8] = {0,0,0,0,0,0,0,0};
  
  request[4] = 0;
  request[5] = 0;

  boolean ret = requestResponse(request, 8, response, 8);  
  if (ret) {
    ret = co2ResponseMatchesRequest(request, response, 8);
  }  
  return ret;
}

boolean K30::clearCalibrationAcknowledgements(void) {
  boolean ret = false;  
  uint8_t request[8] = {0xFE, 0x06, 0x00, 0x00, 0x00, 0x00, 0x9D, 0xC5}; 
  uint8_t response[8] = {0,0,0,0,0,0,0,0};

  ret = requestResponse(request, 8, response, 8);  

  if (ret) {
    ret = co2ResponseMatchesRequest(request, response, 8);
  }

  return ret;  
}

boolean K30::startBackgroundCalibration(void) {
  boolean ret = false;  
  uint8_t request[8] = {0xFE, 0x06, 0x00, 0x01, 0x7C, 0x06, 0x6C, 0xC7};
  uint8_t response[8] = {0,0,0,0,0,0,0,0};

  ret = requestResponse(request, 8, response, 8);
  if (ret) {
    ret = co2ResponseMatchesRequest(request, response, 8);
  }
  return ret;  
}

boolean K30::startNitrogenCalibration(void) {
  boolean ret = false;  
  uint8_t request[8] = {0xFE, 0x06, 0x00, 0x01, 0x7C, 0x07, 0x6C, 0xC7};
  uint8_t response[8] = {0,0,0,0,0,0,0,0};

  ret = requestResponse(request, 8, response, 8);
  if (ret) {
    ret = co2ResponseMatchesRequest(request, response, 8);
  }
  return ret;  
}

boolean K30::checkBackgroundCalibrationComplete(boolean * status) {
  uint8_t request[8] = {0xFE, 0x03, 0x00, 0x00, 0x00, 0x01, 0x90, 0x05};
  uint8_t response[7] = {0,0,0,0,0,0,0};
  boolean ret = requestResponse(request, 8, response, 7);
  if (ret && (status != NULL)) {
    uint16_t value = extractUint16Field(&(response[0]), 3);
    *status = (value & 0x0020);
  }
  return ret;  
}

boolean K30::checkNitrogenCalibrationComplete(boolean * status) {
  uint8_t request[8] = {0xFE, 0x03, 0x00, 0x00, 0x00, 0x01, 0x90, 0x05};
  uint8_t response[7] = {0,0,0,0,0,0,0};
  boolean ret = requestResponse(request, 8, response, 7);
  if (ret && (status != NULL)) {
    uint16_t value = extractUint16Field(&(response[0]), 3);
    *status = (value & 0x0040);
  }
  return ret;  
}

boolean K30::calibrateToBackground(void) {
  boolean ret = false;
  boolean timeout = false;
  unsigned long start = millis();
  const long timeout_interval = 15000;
  unsigned long currentMillis = millis();  
  boolean status = false;

  while (!timeout) {
    currentMillis = millis();
    if ((currentMillis - start) >= timeout_interval) {
      timeout = true;
      break;
    }

    ret = clearCalibrationAcknowledgements();
    if (ret) {
      ret = startBackgroundCalibration();
    }
    delay(250); // wait a quarter second before checking for completion
    if (ret) {
      ret = checkBackgroundCalibrationComplete(&status);
    }
    if (ret && status) {
      break; // breaking from the loop returns success
    }

    delay(100);  // pause between attempts
    ret = false; // reset return value
  } 
  
  return ret;
}

boolean K30::calibrateToNitrogen(void) {
  boolean ret = false;
  boolean timeout = false;
  unsigned long start = millis();
  const long timeout_interval = 15000;
  unsigned long currentMillis = millis();  
  boolean status = false;

  while (!timeout) {
    currentMillis = millis();
    if ((currentMillis - start) >= timeout_interval) {
      timeout = true;
      break;
    }

    ret = clearCalibrationAcknowledgements();
    if (ret) {
      ret = startNitrogenCalibration();
    }
    delay(250); // wait a quarter second before checking for completion
    if (ret) {
      ret = checkNitrogenCalibrationComplete(&status);
    }
    if (ret && status) {
      break; // breaking from the loop returns success
    }

    delay(100);  // pause between attempts
    ret = false; // reset return value
  } 
  
  return ret;
}

void K30::clearCO2SerialInput(void){
  // Serial.println("clearCO2SerialInput called");
  while(available() > 0){
    read();
  }  
  // Serial.println("clearCO2SerialInput returning");
}

void K30::resetSerial(){
  endSerial();
  beginSerial();
  clearCO2SerialInput();
}


void K30::beginSerial() {
  // Serial.println("beginSerial called");
  if(swserial){
    swserial->begin(9600);
    serialBeginCalled = true;
  }
  else if(hwserial){
    hwserial->begin(9600);
    serialBeginCalled = true;
  }
  // Serial.println("beginSerial returning");
}

void K30::endSerial(){
  // Serial.println("endSerial called");
  if (serialBeginCalled) {
    if(swserial){
      swserial->end();
    }
    else if(hwserial){
      hwserial->end();
    }
  }
  // Serial.println("endSerial returning");
}

int K30::read(){
  if(swserial){
    return swserial->read();
  }
  else if(hwserial){
    return hwserial->read();
  }
  return -1;
}

void K30::write(uint8_t value){
  if(swserial){
    swserial->write(value);
  }
  else if(hwserial){
    hwserial->write(value);
  }
}

void K30::write(uint8_t * value, int size){
  if(swserial){
    swserial->write(value, size);
  }
  else if(hwserial){
    hwserial->write(value, size);
  }
}

int K30::available() {
  if(swserial){
    return swserial->available();
  }
  else if(hwserial){
    return hwserial->available();
  }
  return -1;
}

uint16_t K30::extractUint16Field(uint8_t * response, uint8_t byteOffset) {
  uint8_t high = response[byteOffset];
  uint8_t low  = response[byteOffset + 1];

  uint16_t ret = high;
  ret <<= 8;   // shift the low byte into the high byte
  ret |= low;  // mask in the low byte
  return ret;  // return the result
}


uint16_t K30::co2_CRC16 (const uint8_t *nData, uint16_t wLength) {
   uint8_t nTemp;
   uint16_t wCRCWord = 0xFFFF;

   while (wLength--)
   {
      nTemp = *nData++ ^ wCRCWord;
      wCRCWord >>= 8;
      wCRCWord  ^= wCRCTable[nTemp];
   }
   return wCRCWord;
}


boolean K30::co2ResponseMatchesRequest(uint8_t * request, uint8_t * response, uint8_t num) {
  boolean ret = true;
  for (uint16_t ii = 0; ii < (num & 0xff); ii++) {
    if (request[ii] != response[ii]) {
      // Serial.printf("[%02x <-> %02x] ", request[ii], response[ii]);
      ret = false;
    }
  }
  // Serial.println();
  return ret;
}

boolean K30::requestResponse(uint8_t * request, uint8_t requestLength, uint8_t * response, uint8_t expectedResponseLength) {
  // resetSerial(); // flushes the input stream and calls end/begin  
  for (uint8_t ii = 0; ii < 8; ii++) { // try up to 8 times
    boolean ret = requestResponseOnce(request, requestLength, response,expectedResponseLength);
    if (ret) {
       return true;
    }

    delay(100); // wait 50 milliseconds to try again
  }

  return false;
}

boolean K30::requestResponseOnce(uint8_t * request, uint8_t requestLength, uint8_t * response, uint8_t expectedResponseLength) {
  boolean ret = false;
  boolean timeout = false;
  unsigned long start = millis();
  const long timeout_interval = 200;
  unsigned long currentMillis = millis();  
  
  if ((request != NULL) && (requestLength > 0)) {
    // make sure that the CRC (last two bytes) is correct
    uint16_t crc = co2_CRC16(request, requestLength - 2);
    uint8_t highByte = (crc >> 8) & 0xff;
    uint8_t lowByte  = (crc & 0xff);
    request[requestLength - 2] = lowByte;
    request[requestLength - 1] = highByte;

#if defined(CO2_ECHO_LOW_LEVEL_REQUESTS)
    Serial.printf(">>>>> (%d): ", requestLength);    
#endif 
    for (uint16_t ii = 0; ii < requestLength; ii++) {
      write(request[ii]);
#if defined(CO2_ECHO_LOW_LEVEL_REQUESTS)
      Serial.printf("%02x ", request[ii]);
#endif
    }
#if defined(CO2_ECHO_LOW_LEVEL_REQUESTS)
      Serial.println();
#endif
    
    // request has been sent, now wait for available to be come > 0
    while (!(available() > 0)) {
      currentMillis = millis();
      if ((currentMillis - start) >= timeout_interval) {
        timeout = true;
        break;
      }
    }

    if ((response != NULL) && (expectedResponseLength > 0)) {
      // all responses start with <FE> <06> or <FE> <03>
      uint8_t state = 0; // 0 = waiting for 0xFE
                         // 1 = got valid header
      uint8_t bytesConsumed = 0;

#if defined(CO2_ECHO_LOW_LEVEL_RESPONSES)
      Serial.printf("<<<<< (%d): ", expectedResponseLength);    
#endif            

      while (!timeout && !ret) {
        currentMillis = millis();
        if ((currentMillis - start) >= timeout_interval) {
          timeout = true;
        }

        if (available() > 0) {
          start = currentMillis;
          int b = read();
          if (b >= 0) {

#if defined(CO2_ECHO_LOW_LEVEL_RESPONSES)
            Serial.printf("%02x ", (uint8_t) b);
#endif

            switch (state) {
            case 0:  // need to see a 0xFE
              if (b == 0xFE) {
                response[0] = b;
                state = 1;
                bytesConsumed = 1;
              }
              break;
            default: // we're past the header, keep reading until you have enough bytes
              response[bytesConsumed++] = b;
              if (bytesConsumed == expectedResponseLength) {
                // check the CRC 
                uint16_t expected = response[expectedResponseLength - 1] * 256 + response[expectedResponseLength - 2];
                uint16_t actual = co2_CRC16(response, expectedResponseLength - 2);
                if(expected == actual) {
                  ret = true;
                }
              }
              break;
            }
          }
        }
      }

#if defined(CO2_ECHO_LOW_LEVEL_RESPONSES)
      Serial.println();    
#endif            
    }
  }
  
  return ret;
}

uint16_t K30::wCRCTable[] = {
   0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
   0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
   0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
   0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
   0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
   0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
   0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
   0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
   0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
   0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
   0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
   0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
   0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
   0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
   0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
   0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
   0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
   0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
   0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
   0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
   0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
   0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
   0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
   0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
   0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
   0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
   0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
   0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
   0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
   0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
   0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
   0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040 };
