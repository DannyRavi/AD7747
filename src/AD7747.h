#ifndef AD7747_h
#define AD7747_h

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>


//register conf---------------------------------------

#define CAP_SETUP_REG_VALUE B10100000 // 0XA0 = CAPEN=1, CAPDIFF=1
#define VT_SETUP_REG_VALUE B10000001  // 0X81 = VTEN=1, VTMD1=0, VTMD0=0, EXTREF=0, VSHORT=0, VTCHOP=1
#define EXC_SETUP_REG_VALUE B00001111 // 0X07 = EXCDAC=1, EXCEN=1, EXCLVL1=1, EXCLVL0=0
// TODO: RECHECK DATASHEET FOR CONFIGURATION_REG_VALUE
#define CONFIGURATION_REG_VALUE B01111001 // 0X01 = {VTFS1=0 , VTFS0=0 == CONVERSATION TIME :20.1 ms}
                                          // =  [CAPFS2 CAFPFS1 CAPFS0 = 000 ==> DSP CONVERSATION TIME :22.0 ms ]
                                          // =  [MD2 MD1 MD0 = 001 ===> COUNTINUOUS MODE ]
                                          // =  [OFFSET U GAIN CALIBRATION ]

#define CAPDAC_CH_A_REG_VALUE B10000000 // 0X80  -> DACAENA=1 , DACA=000000
#define CAPDAC_CH_B_REG_VALUE B10000000 // 0X80  -> DACBENB=1 , DBCB=000000

#define CAP_OFFSET_H_REG_VALUE B10000000 
#define CAP_OFFSET_L_REG_VALUE B00000000

#define CAP_GAIN_H_REG_VALUE B00000000 
#define CAP_GAIN_L_REG_VALUE B00000000

#define VOLT_GAIN_H_REG_VALUE B00000000 
#define VOLT_GAIN_L_REG_VALUE B00000000

// reg address

#define __STATUS__ 0x00    // 00000111
#define CAP_DATA_H 0x01    // (R) Capacitive channel data—high byte, 0x00
#define CAP_DATA_M 0x02    // (R) Capacitive channel data—middle byte, 0x00
#define CAP_DATA_L 0x03    // (R) Capacitive channel data—low byte, 0x00
#define VT_DATA_H 0x04     // (R) Voltage/temperature channel data—high byte, 0x00
#define VT_DATA_M 0x05     // (R) Voltage/temperature channel data—middle byte, 0x00
#define VT_DATA_L 0x06     // (R) Voltage/temperature channel data—low byte, 0x00
#define CAP_SETUP 0x07     // (R/W) CAPEN – CAPDIFF – – – – –
#define VT_SETUP 0x08      // (R/W) VTEN VTMD1 VTMD0 EXTREF – - VTSHORT VTCHOP
#define EXC_SETUP 0x09     // (R/W) – – – – EXCDAC EXCEN EXCLVL1 EXCLVL0
#define CONFIGURATION 0x0A // (R/W) VTFS1 VTFS0 CAPFS2 CAPFS1 CAPFS0 MD2 MD1 MD0
#define CAPDAC_CH_A 0x0B     // (R/W) DACAENA – DACA—6-Bit Value
#define CAPDAC_CH_B 0x0C     // (R/W) DACBENB – DACB—6-Bit Value
#define CAP_OFFSET_H 0x0D  // (R/W) Capacitive offset calibration—high byte, 0x80
#define CAP_OFFSET_L 0x0E  // (R/W) Capacitive offset calibration—low byte, 0x00
#define CAP_GAIN_H 0x0F    // (R/W) Capacitive gain calibration—high byte, factory calibrated
#define CAP_GAIN_L 0x10    // (R/W) Capacitive gain calibration—low byte, factory calibrated
#define VOLT_GAIN_H 0x11   // (R/W) Voltage gain calibration—high byte, factory calibrated
#define VOLT_GAIN_L 0x12   // (R/W) Voltage gain calibration—low byte, factory calibrated

#define CAP_DATA_START 0x01
#define VT_DATA_START 0x04

#define _AD7747_ADDR_READ_ 0x91
#define _AD7747_ADDR_WRITE_ 0x90



  class AD7747
  {

  public:
    AD7747::AD7747(uint8_t I2C_addr_read, uint8_t I2C_addr_write)
    {

      _I2c_AddrRead_ = I2C_addr_read >> 1;
      _I2C_AddrWrite_ = I2C_addr_write >> 1;
    }

    uint8_t read8_AD77(uint8_t _address);
    uint8_t read8_debug_AD77(uint8_t _address);
    void write8_AD77(uint8_t _address, uint8_t _data);
    uint16_t read16_AD77(uint16_t _address);
    void write16_AD77(uint8_t _address, uint8_t _dataHIGH, uint8_t _dataLOW);
    void loadSettings(void);
    uint8_t getSetupCap(uint8_t address);
    uint32_t getCap(uint8_t addressStart);
    uint32_t getVT(uint8_t vt_start);
    bool isDeviceConnected(void);
    double capPF(uint32_t rawCap);
    double TempCentigrade(uint32_t rawTemp);
    bool debug_mode = true;

  private:
    uint8_t _I2c_AddrRead_;
    uint8_t _I2C_AddrWrite_;

    uint8_t capDataHigh(uint8_t address);
    uint8_t capDataMid(uint8_t address);
    uint8_t capDataLow(uint8_t address);

    uint8_t vtDataHigh(uint8_t address);
    uint8_t vtDataMid(uint8_t address);
    uint8_t vtDataLow(uint8_t address);

    void setupCapX(uint8_t address, uint8_t data);
    void SetupVT(uint8_t address, uint8_t data);
    void SetupEXC(uint8_t address, uint8_t data);
    void setConfig(uint8_t address, uint8_t data);

    void setCapDacA(uint8_t address, uint8_t data);
    void setCapDacB(uint8_t address, uint8_t data);

    void setOffsetH(uint8_t address, uint8_t dataHigh);
    void setOffsetL(uint8_t address, uint8_t dataLow);

    void setGainH(uint8_t address, uint8_t dataHigh);
    void setGainL(uint8_t address, uint8_t dataLow);

    void setOffset(uint8_t address, uint8_t dataHigh, uint8_t dataLow);
    void setGain(uint8_t address, uint8_t dataHigh, uint8_t dataLow);

    bool checkCapReady(uint8_t address);
    bool checkTempReady(uint8_t address);
    uint8_t check_AD77(uint8_t address);
    uint8_t *READ24bit(uint8_t address);
    void __debugger__(char *title, uint8_t arr_input[], uint8_t size_arr);
    void __debugger__(char *title, double arr_input[], uint8_t size_arr);
  };



#endif
