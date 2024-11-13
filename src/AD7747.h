#ifndef AD7747_h
#define AD7747_h

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>


/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 500 Hz

* 0 Hz - 25 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = 3.9914981244836114 dB

* 30 Hz - 250 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = -40.37831296301491 dB

*/

#define FILTER_TAP_NUM 320

static double filter_taps[FILTER_TAP_NUM] = {
  0.005682152041755636,
  0.007496887406825002,
  0.00627396985368197,
  0.010739734551162105,
  0.010684314921187214,
  0.013772538113848806,
  0.013727900731362163,
  0.015159436344205196,
  0.014239848575093342,
  0.013803071754678934,
  0.011590124979960113,
  0.00943635669500638,
  0.00611126337418197,
  0.002893908648364085,
  -0.0007841379894557365,
  -0.003989995923523741,
  -0.006929979788079516,
  -0.00894636236362259,
  -0.010130410349318997,
  -0.010101879319390127,
  -0.00902196542127722,
  -0.006809856113919764,
  -0.003766942825835133,
  -0.00009415310749908116,
  0.003779273952149918,
  0.00749031576430912,
  0.010573521954903384,
  0.012674247507598116,
  0.013451304876052906,
  0.012718991262408387,
  0.010418016664735467,
  0.006659260408683803,
  0.0017042864779196133,
  -0.004002651108912375,
  -0.009920742107230573,
  -0.015409047227071395,
  -0.019798394929436008,
  -0.02243703899111411,
  -0.02277681156025595,
  -0.020401898545809333,
  -0.01508091979361543,
  -0.006816471586037405,
  0.004159729300790012,
  0.01738235977630521,
  0.03219623419210954,
  0.047764457969961434,
  0.06316302488340003,
  0.07742412438355602,
  0.08963513886940969,
  0.09899652764318011,
  0.10487905889839767,
  0.10688579170459919,
  0.10487905889839767,
  0.09899652764318011,
  0.08963513886940969,
  0.07742412438355602,
  0.06316302488340003,
  0.047764457969961434,
  0.03219623419210954,
  0.01738235977630521,
  0.004159729300790012,
  -0.006816471586037405,
  -0.01508091979361543,
  -0.020401898545809333,
  -0.02277681156025595,
  -0.02243703899111411,
  -0.019798394929436008,
  -0.015409047227071395,
  -0.009920742107230573,
  -0.004002651108912375,
  0.0017042864779196133,
  0.006659260408683803,
  0.010418016664735467,
  0.012718991262408387,
  0.013451304876052906,
  0.012674247507598116,
  0.010573521954903384,
  0.00749031576430912,
  0.003779273952149918,
  -0.00009415310749908116,
  -0.003766942825835133,
  -0.006809856113919764,
  -0.00902196542127722,
  -0.010101879319390127,
  -0.010130410349318997,
  -0.00894636236362259,
  -0.006929979788079516,
  -0.003989995923523741,
  -0.0007841379894557365,
  0.002893908648364085,
  0.00611126337418197,
  0.00943635669500638,
  0.011590124979960113,
  0.013803071754678934,
  0.014239848575093342,
  0.015159436344205196,
  0.013727900731362163,
  0.013772538113848806,
  0.010684314921187214,
  0.010739734551162105,
  0.00627396985368197,
  0.007496887406825002,
  0.005682152041755636
};


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
