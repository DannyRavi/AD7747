[![Build workflow](https://img.shields.io/github/actions/workflow/status/DannyRavi/AD7747/c-cpp.yml?style=flat-square)](https://github.com//DannyRavi/AD7747/actions/workflows/c-cpp.yml?query=branch%3Amain)

# AD7747 Arduino Library
Library for Analog devices AD7747 Capacitive sensing.

# Supported devices
>* AD7747

# Usage
Import header and use AD7747 instance, Init and get data.

```cpp
#include "AD7747.h"
AD7747 cap(_AD7747_ADDR_READ_, _AD7747_ADDR_WRITE_);
...
void setup() {
    ...
    Wire.begin();
    cap.debug_mode = false; //use it to activate log
    bool isConnect = cap.isDeviceConnected();//check device AD7747 is connect?

    if (isConnect)
    {
        cap.loadSettings();//load settings in AD7747
        Serial.println("connected");
    }
    ...
}
void loop(){
    ...
        uint32_t _raw_cap_ = cap.getCap(CAP_DATA_START);
    ...
}
```

# Hardware
AD7747  3.3V  or 5.0V powered.

**To run examples, connect FDC with arduino as follows:**
 >* ARDUINO <--> AD7747 
 >* A4 <-------> SDA
 >* A5 --------> SCL
 >* A2 --------> RDY (optinal)


# Tools
To view nice real-time graph of the sensor output, it is highly recommended to use tool like ArduinoSerialPlot.



**sensing cap:**
![sensing](https://github.com/DannyRavi/AD7747/blob/main/assets/ad77_cap.png  "sensing")

**schematic:**
![schematic](https://github.com/DannyRavi/AD7747/blob/main/assets/ad77_sch.png "schematic")

