#ifndef BMA020_H
#define BMA020_H

#include <Arduino.h>
#include <Wire.h>

const uint8_t BMA029ADDR = 0x38;

class BMA020 : private TwoWire {
public:
    // 2g, 4g, 8g available
    enum BMA020RANGE {
        BMA020_RANGE_2G = 0x00, // 00b
        BMA020_RANGE_4G = 0x01, // 01b
        BMA020_RANGE_8G = 0x02 // 10b
    };
    // 25 - 1500 Hz available
    enum BMA020BANDWIDTH {
        BMA020_BW_25HZ = 0x00, // 000b (mean 23 Hz)
        BMA020_BW_50HZ = 0x01, // 001b (mean 47 Hz)
        BMA020_BW_100HZ = 0x02, // 010b (mean 94 Hz)
        BMA020_BW_190HZ = 0x03, // 011b (mean 188 Hz)
        BMA020_BW_375HZ = 0x04, // 100b (mean 375 Hz)
        BMA020_BW_750HZ = 0x05, // 101b (mean 750 Hz)
        BMA020_BW_1500HZ = 0x06 // 110b (mean 1500 Hz)
    };
    // list of used registers
    enum BMA020REGISTER {
        E_CHIPID = 0x00,
        E_VERSION = 0x01,
        E_DATA_LSBX = 0x02,
        E_DATA_MSBX = 0x03,
        E_DATA_LSBY = 0x04,
        E_DATA_MSBY = 0x05,
        E_DATA_LSBZ = 0x06,
        E_DATA_MSBZ = 0x07,
        E_SETUP_ACC = 0x14,
        E_CONTROL_OP = 0x15
    };

    // acc data struct
    struct raw_acc_t {
        int16_t acc_x;
        int16_t acc_y;
        int16_t acc_z;
    };

    // acc data struct
    struct g_data_t {
        double_t acc_g_x;
        double_t acc_g_y;
        double_t acc_g_z;
    };

private:
    // internal acc data struct
    raw_acc_t _raw_data;
    // internal acc data struct
    g_data_t _g_data;
    // device adress
    byte _adr;
    // calculate raw int16_t acc values into g data
    double _gfactor;

protected:
    // write data to register
    void writeReg(BMA020REGISTER reg, uint8_t value);
    // read byte from reg
    byte readReg(BMA020REGISTER reg);
    // ask for chip id (010b)
    uint8_t getChipId();
    // get internal data
    bool update_acc(raw_acc_t& data);
    // calculate gfactor
    bool calculateGFactor();

public:
    BMA020();
    BMA020(uint8_t adr, uint8_t sda = SDA, uint8_t scl = SCL);

    bool begin(uint8_t sda = SDA, uint8_t scl = SCL, uint8_t adr = BMA029ADDR);

    bool isOk();

    // acc range
    void setRange(BMA020RANGE range = BMA020_RANGE_8G);
    BMA020RANGE getRange();
    // acc bandwidth ("polling rate")
    void setBandwidth(BMA020BANDWIDTH bandwidth = BMA020_BW_25HZ);
    BMA020BANDWIDTH getBandWidth();
    // get acc data
    raw_acc_t& getRawData();
    // get acc in g data
    g_data_t& getGData();
};

#endif // BMA020_H
