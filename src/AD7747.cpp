#include "HardwareSerial.h"
#include "Arduino.h"
#include <Wire.h>
#include "AD7747.h"

void AD7747::__debugger__(char *title, uint8_t arr_input[], uint8_t size_arr)
{
    Serial.print(title);
    for (int i = 0; i < size_arr; ++i)
    {
        Serial.print(arr_input[i]);
        Serial.print(" ");
    }
    Serial.println();
}

void AD7747::__debugger__(char *title, double arr_input[], uint8_t size_arr)
{

    Serial.print(title);
    for (int i = 0; i < size_arr; ++i)
    {
        Serial.print(arr_input[i]);
        Serial.print(" ");
    }
    Serial.println();
}

uint16_t AD7747::read16_AD77(uint16_t address)
{
    uint16_t data = 0;

    Wire.beginTransmission(_I2c_AddrRead_);
    Wire.write(address);
    Wire.endTransmission(false); // restart

    Wire.requestFrom(_I2c_AddrRead_, (uint8_t)2);

    if (Wire.available() == 2)
    {
        data = Wire.read();
        data <<= 8;
        data |= Wire.read();
    }

    return data;
}

uint8_t *AD7747::READ24bit(uint8_t address)
{
    static uint8_t read_buffer[4];

    Wire.beginTransmission(_I2c_AddrRead_);
    Wire.write(address);
    Wire.endTransmission(); // restart

    Wire.requestFrom(_I2c_AddrRead_, (uint8_t)4);
    if (Wire.available())
    {
        read_buffer[0] = Wire.read();
        read_buffer[1] = Wire.read();
        read_buffer[2] = Wire.read();
        read_buffer[3] = Wire.read();
    }
    if (debug_mode)
    {
        uint8_t _size_arr_ = sizeof(read_buffer) / sizeof(read_buffer[0]);
        __debugger__("read_cap_bytes: ", read_buffer, _size_arr_);
    }

    return read_buffer;
}

void AD7747::write8_AD77(uint8_t _address, uint8_t _data)
{

    Wire.beginTransmission(_I2C_AddrWrite_);
    Wire.write(_address);
    Wire.write(_data);
    Wire.endTransmission(true); // restart
    delay(20);
}

void AD7747::write16_AD77(uint8_t _address, uint8_t _dataHIGH, uint8_t _dataLOW)
{
    Wire.beginTransmission(_I2C_AddrWrite_);
    Wire.write(_address);
    Wire.write(_dataHIGH);
    Wire.write(_dataLOW);
    Wire.endTransmission();
}

uint8_t AD7747::read8_AD77(uint8_t _address)
{
    uint8_t data = 0;

    Wire.beginTransmission(_I2c_AddrRead_);
    Wire.write(_address);
    Wire.endTransmission(); 

    Wire.requestFrom(_I2c_AddrRead_, 1);

    if (Wire.available())
    {
        data = Wire.read();
    }

    return data;
}

uint8_t AD7747::read8_debug_AD77(uint8_t _address)
{
    uint8_t data = 0;

    Wire.beginTransmission(_I2C_AddrWrite_);
    Wire.write(_address);
    Wire.endTransmission(false); 

    Wire.requestFrom(_I2c_AddrRead_, (uint8_t)1);

    if (Wire.available() == 1)
    {
        data = Wire.read();
    }

    return data;
}

uint8_t AD7747::check_AD77(uint8_t address)
{
    uint8_t rec = 0;
    rec = read8_AD77(address);
    return rec;
}

uint8_t AD7747::capDataHigh(uint8_t address)
{
    uint8_t rec = 0;
    rec = read8_AD77(address);
    return rec;
}

uint8_t AD7747::capDataMid(uint8_t address)
{
    uint8_t rec = 0;
    rec = read8_AD77(address);
    return rec;
}

uint8_t AD7747::capDataLow(uint8_t address)
{
    uint8_t rec = 0;
    rec = read8_AD77(address);
    return rec;
}

uint8_t AD7747::vtDataHigh(uint8_t address)
{
    uint8_t rec = 0;
    rec = read8_AD77(address);
    return rec;
}

uint8_t AD7747::vtDataMid(uint8_t address)
{
    uint8_t rec = 0;
    rec = read8_AD77(address);
    return rec;
}

uint8_t AD7747::vtDataLow(uint8_t address)
{
    uint8_t rec = 0;
    rec = read8_AD77(address);
    return rec;
}

uint8_t AD7747::getSetupCap(uint8_t address)
{
    uint8_t rec = 5;
    rec = read8_debug_AD77(address);
    return rec;
}

void AD7747::setupCapX(uint8_t address, uint8_t data)
{
    write8_AD77(address, data);
}

void AD7747::SetupVT(uint8_t address, uint8_t data)
{
    write8_AD77(address, data);
}

void AD7747::SetupEXC(uint8_t address, uint8_t data)
{
    write8_AD77(address, data);
}

void AD7747::setConfig(uint8_t address, uint8_t data)
{
    write8_AD77(address, data);
}

void AD7747::setCapDacA(uint8_t address, uint8_t data)
{
    write8_AD77(address, data);
}

void AD7747::setCapDacB(uint8_t address, uint8_t data)
{
    write8_AD77(address, data);
}

void AD7747::setOffset(uint8_t address, uint8_t dataHigh, uint8_t dataLow)
{
    write16_AD77(address, dataHigh, dataLow);
}

void AD7747::setGain(uint8_t address, uint8_t dataHigh, uint8_t dataLow)
{
    write16_AD77(address, dataHigh, dataLow);
}

void AD7747::setOffsetH(uint8_t address, uint8_t dataHigh)
{
    write8_AD77(address, dataHigh);
}

void AD7747::setOffsetL(uint8_t address, uint8_t dataLow)
{
    write8_AD77(address, dataLow);
}

void AD7747::setGainH(uint8_t address, uint8_t dataHigh)
{
    write8_AD77(address, dataHigh);
}

void AD7747::setGainL(uint8_t address, uint8_t dataLow)
{
    write8_AD77(address, dataLow);
}

void AD7747::loadSettings(void)
{

    setupCapX(CAP_SETUP, CAP_SETUP_REG_VALUE);
    SetupVT(VT_SETUP, VT_SETUP_REG_VALUE);
    SetupEXC(EXC_SETUP, EXC_SETUP_REG_VALUE);
    setConfig(CONFIGURATION, CONFIGURATION_REG_VALUE);
    setCapDacA(CAPDAC_CH_A, CAPDAC_CH_A_REG_VALUE);
    setCapDacB(CAPDAC_CH_B, CAPDAC_CH_B_REG_VALUE);

}

uint32_t AD7747::getCap(uint8_t addressStart)
{
    uint32_t __cap__ = 0;
    uint32_t __rec__ = 0;
    uint32_t cap_low = 0;
    uint32_t cap_medium = 0;
    uint32_t cap_high = 0;

    bool _isDataReady_ = checkCapReady(__STATUS__);
    if (_isDataReady_)
    {
        uint8_t *ptr_data; // pointer to hold address
        ptr_data = READ24bit(addressStart); // address of start piont
        cap_high = ptr_data[1];
        cap_high = cap_high << 16;

        cap_medium = ptr_data[2];
        cap_medium = cap_medium << 8;

        cap_low = ptr_data[3];
        __rec__ = cap_high + cap_medium + cap_low;
        __cap__ = __rec__ & 0x00FFFFFF;
        return __cap__;
    }
    return 0;
}

uint32_t AD7747::getVT(uint8_t vt_start)
{
    uint32_t __vt__ = 0;
    uint32_t vt_low = 0;
    uint32_t vt_medium = 0;
    uint32_t vt_high = 0;
    bool _isDataReady_ = checkCapReady(__STATUS__);
    if (_isDataReady_)
    {
        uint8_t *ptr_data; // pointer to hold address
        ptr_data = READ24bit(vt_start);
        vt_high = ptr_data[1];
        vt_high = vt_high << 16;

        vt_medium = ptr_data[2];
        vt_medium = vt_medium << 8;

        vt_low = ptr_data[3];

        __vt__ = (vt_high + vt_medium + vt_low) & 0x00FFFFFF;
        return __vt__;
    }
    return 0;
}

bool AD7747::checkCapReady(uint8_t address)
{
    uint8_t read_status = 1;
    while (true)
    {
        read_status = check_AD77(address) & 0x1;
        if (read_status == 0)
        {
            return true;
        }
        delay(10);
    }
}

bool AD7747::checkTempReady(uint8_t address)
{
    uint8_t read_status = 1;
    while (true)
    {
        read_status = check_AD77(address) & 0x1;
        if (read_status == 0)
        {
            return true;
        }
        delay(10);
    }
}

double AD7747::capPF(uint32_t rawCap)
{

    uint32_t maxPow = 16777216; // 2^24 = 16777216.0 || 16763186
    double max_cap_pf = 16.384;

    double output = 0.0;
    double rawCap_double = rawCap;
    double maxPow_double = maxPow;
    double norm_double_data = rawCap_double / maxPow_double;

    output = norm_double_data * max_cap_pf;

    if (debug_mode)
    {
        double _log_[4] = {rawCap_double, maxPow_double, norm_double_data, output};
        uint8_t _size_arr_ = sizeof(_log_) / sizeof(_log_[0]);
        __debugger__("input_cap,max_cap,norm_cap,pf     ", _log_, _size_arr_);
    }

    return output;
}

double AD7747::TempCentigrade(uint32_t rawTemp)
{

    double temp_centigrade = 0.0;
    Serial.print("raw: ");
    Serial.println(rawTemp);
    double rawTemp_double = (double)rawTemp;
    double Temp_const = 2048;
    double Temp_offset = 4096;
    temp_centigrade = (rawTemp_double / Temp_const) - Temp_offset;

    if (debug_mode)
    {
        double _log_[2] = {rawTemp_double, temp_centigrade};
        uint8_t _size_arr_ = sizeof(_log_) / sizeof(_log_[0]);
        __debugger__("raw_degree ,degree     ", _log_, _size_arr_);
    }

    return temp_centigrade;
}

bool AD7747::isDeviceConnected(void)
{
    while (true)
    {
        int nDevices = 0;
        for (byte address = 1; address < 127; ++address)
        {
            // The i2c_scanner uses the return value of
            // the Write.endTransmisstion to see if
            // a device did acknowledge to the address.
            Wire.beginTransmission(address);
            byte error = Wire.endTransmission();

            if (error == 0)
            {
                Serial.print("I2C device found at address 0x");
                Serial.print(address, HEX);
                Serial.println("  !");
                if (address < 16)
                {
                    Serial.print("0");
                }

                if (_I2C_AddrWrite_ == (uint8_t)address)
                {
                    Serial.println("+++ connected to AD7747 +++");
                    return true;
                }
                else
                {
                    Serial.println(" this device is not AD7747 !");
                }

                ++nDevices;
            }
            else if (error == 4)
            {
                Serial.print("Unknown error at address 0x");
                if (address < 16)
                {
                    Serial.print("0");
                }
                Serial.println(address, HEX);
            }
        }
        if (nDevices == 0)
        {
            Serial.println("No I2C devices found\n");
        }
        else
        {
            Serial.println("reload\n");
        }
        delay(5000); // Wait 5 seconds for next scan
    }
}
