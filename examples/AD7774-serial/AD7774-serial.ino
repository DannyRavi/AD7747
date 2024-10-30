#include <Wire.h>
#include "AD7747.h"

AD7747 cap(_AD7747_ADDR_READ_, _AD7747_ADDR_WRITE_);


void setup()
{
  cap.debug_mode = false; //use it to activate log
  Wire.begin();
  Serial.begin(9600);
  bool isConnect = cap.isDeviceConnected();//check device AD7747 is connect?

  if (isConnect)
  {
    cap.loadSettings();//load settings in AD7747
    Serial.println("connected");
  }
}

void loop()
{
  char charBuf[16];
  uint32_t _raw_cap_ = cap.getCap(CAP_DATA_START);
  double cap_pf = cap.capPF(_raw_cap_);
  dtostrf(cap_pf, 12, 6, charBuf);
  Serial.println(charBuf);
}