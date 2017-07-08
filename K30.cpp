#include "K30.h"

K30::K30(HardwareSerial * hwserial){
  this->swserial = NULL;
  this->hwserial = hwserial;
}

K30::K30(SoftwareSerial * swserial){
  this->swserial = swserial;
  this->hwserial = NULL;
}

boolean K30::getSample(float * result){
  resetSerial();
  boolean ret = requestCO2Data(result);
  endSerial();
  return ret;
}

void K30::clearCO2SerialInput(void){
  Stream * s = swserial ? (Stream *) swserial : (Stream *) hwserial;
  if(s){
    while(s->available()){
      s->read();
    }
  }
}

void K30::resetSerial(){
  if(swserial){
    swserial->begin(9600);
  }
  else if(hwserial){
    hwserial->begin(9600);
  }

  clearCO2SerialInput();
}

void K30::endSerial(){
  if(swserial){
    swserial->end();
  }
  else if(hwserial){
    hwserial->end();
  }
}

uint8_t K30::read(){
  if(swserial){
    return swserial->read();
  }
  else if(hwserial){
    return hwserial->read();
  }
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

boolean K30::requestCO2Data(float * co2_ppm){
  uint8_t readCO2[] = {0xFE, 0x44, 0x00, 0x08, 0x02, 0x9F, 0x25};  // Command packet to read Co2 (see app note)
  uint8_t response[] = {0,0,0,0,0,0,0};                            // Create an array to store the response
  // try up to retry times
  const uint8_t retries = sizeof(readCO2);

  for(uint8_t ii = 0; ii < retries; ii++){
    if(co2SendRequest(readCO2)){
      // there is the beginning of a response at least
      if(co2ConsumeResponse(response)){
        // there's a complete response at least
        if(co2ValidResponse(response)){
          // there's a valid response, we have a winner
          *co2_ppm = co2GetValue(response);
          return true;
        }
      }
    }

    // attempt to re-synchronize
    // Serial.println(F("Info: Attempting to resynchronize with CO2 sensor"));
    for(uint8_t jj = 0; jj <= ii; jj++){
      // send out a variable number of padding bytes in order to brute-force resynchronize the sensor
      // eventually the request message should be received by the sensor correctly
      write(0xff);
    }

  }

  // Serial.print(F("Warning: Failed to collect CO2 Data"));
  return false;
}

// sends the request command
// returns true the sensor starts responding within 50ms
boolean K30::co2SendRequest(uint8_t * request){
  // before sending the command, take some steps
  // to ensure that that the input buffer is empty
  Stream * s = swserial ? (Stream *) swserial : (Stream *) hwserial;

  delay(50);
  clearCO2SerialInput();

  write(request, 7);
  unsigned long start = millis();
  const long timeout_interval = 50;

  while(1){
    unsigned long currentMillis = millis();

    if(s->available()){
      // exit with true as soon as you get some bytes back
      return true;
    }

    if(currentMillis - start >= timeout_interval) {
      // exit with false if you get a timeout
      return false;
    }
  }

  return false;
}

boolean K30::co2ConsumeResponse(uint8_t * response){
  // expect a response that starts with 0xFE
  // expect a response that is 7 bytes long
  // expect a response to complete within 100ms
  uint8_t bytes_received = 0;
  Stream * s = swserial ? (Stream *) swserial : (Stream *) hwserial;
  unsigned long start = millis();
  const long timeout_interval = 100;
  while(bytes_received < 7){
    unsigned long currentMillis = millis();
    if(currentMillis - start >= timeout_interval) {
      return false;
    }

    if(s->available()){
      uint8_t value = read();
      if((bytes_received == 0) && (value != 0xFE)){ // reject bytes until synchronized
        continue;
      }
      else{
        response[bytes_received++] = value;
      }
    }
  }

  return true;
}

uint16_t K30::co2GetValue(uint8_t * response){
  uint8_t high = response[3]; // high byte for value is 4th byte in packet in the packet
  uint8_t low  = response[4]; // low byte for value is 5th byte in the packet

  uint16_t ret = high;
  ret <<= 8;   // shift the low byte into the high byte
  ret |= low;  // mask in the low byte
  return ret;  // return the result
}

boolean K30::co2ValidResponse(uint8_t * response){
  uint16_t crc = response[6] * 256 + response[5];
  if(co2_CRC16(response, 5) != crc){
    // Serial.print(F("Warning: CO2 Sensor CRC check failed"));
    // Serial.println();
    return false;
  }

  if(co2GetValue(response) > 10000){
    // Serial.print(F("Warning: CO2 Sensor reported "));
    // Serial.print(co2GetValue(response));
    // Serial.print(F("ppm > 10000ppm"));
    // Serial.println();
    return false;
  }

  return true;
}

uint16_t K30::co2_CRC16 (const uint8_t *nData, uint16_t wLength)
{
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
