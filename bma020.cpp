#include "bma020.h"

#include <Arduino.h>

// register and low level stuff

// default chip id (r/o)
constexpr byte _DEFAULT_CHIPID = 0x02u; //010b

// R14h ranges
constexpr byte _RANGE0 = 0x3u;
constexpr byte _RANGE1 = 0x4u;
constexpr byte _bit_mask_Range = (1 << _RANGE0) | (1 << _RANGE1);

// R14h bandwidth
constexpr byte _BW0 = 0x0u;
constexpr byte _BW1 = 0x1u;
constexpr byte _BW2 = 0x2u;
constexpr byte _bit_mask_Bandwidth = (1 << _BW0) | (1 << _BW1) | (1 << _BW2);

// ACC data
constexpr byte _ACC_Start_ADR = 0x02u;
constexpr uint8_t _ACC_Byte_Count = 6u;
constexpr uint8_t _acc_reg_resolution = 10u;

// transfer two complement into dec
int16_t twoCompToDec(uint16_t in, uint8_t highestBit)
{
    int16_t erg = in;
    if ((erg >> (highestBit - 1)) > 0) {
        erg |= 1 << (highestBit - 1);
        erg -= 1 << highestBit;
    }
    return erg;
}

// access to registers
void BMA020::writeReg(BMA020REGISTER reg, uint8_t value)
{
    beginTransmission(_adr);
    write(reg);
    write(value);
    endTransmission();
}

uint8_t BMA020::readReg(BMA020::BMA020REGISTER reg)
{
    beginTransmission(_adr);
    write(reg);
    endTransmission();

    requestFrom(_adr, (uint8_t)1);
    return read();
}

// get or set "properties" of BMA020

uint8_t BMA020::getChipId()
{
    return readReg(BMA020::E_CHIPID);
}

int16_t getACCfromBytes(byte lsb, byte msb)
{
    return twoCompToDec((msb << 2) + (lsb >> 6), _acc_reg_resolution);
}

bool BMA020::update_acc(BMA020::raw_acc_t& data)
{
    beginTransmission(_adr);
    write(_ACC_Start_ADR);
    endTransmission();

    uint8_t acc_regs[_ACC_Byte_Count];

    uint8_t cnt = requestFrom(_adr, _ACC_Byte_Count);
    for (uint8_t ctr = 0; ctr < cnt && ctr < _ACC_Byte_Count; ctr++) {
        acc_regs[ctr] = read();
    }

    int16_t acc_x = getACCfromBytes(acc_regs[0], acc_regs[1]);
    int16_t acc_y = getACCfromBytes(acc_regs[2], acc_regs[3]);
    int16_t acc_z = getACCfromBytes(acc_regs[4], acc_regs[5]);

    auto half = (1 << 9);

    this->_raw_data.acc_x = acc_x;
    this->_raw_data.acc_y = acc_y;
    this->_raw_data.acc_z = acc_z;

    return true;
}

bool BMA020::calculateGFactor()
{
    bool ok = isOk();
    auto r = getRange();
    _gfactor = 1.0 * (1 << (static_cast<int>(r) + 1)) / static_cast<double_t>(((1 << (_acc_reg_resolution - 1))));
    return ok;
}

constexpr byte att_mask = ((1 << 7) | (1 << 5) | (1 << 6));
void BMA020::setRange(BMA020::BMA020RANGE range)
{
    byte reg_14h = readReg(E_SETUP_ACC);
    // preserve highest bits
    byte attention = reg_14h & att_mask;
    //11100111b = 11111111b ^ 00011000b
    byte delMask = ((1 << 8) - 1) ^ _bit_mask_Range;
    // clear wanted bits
    reg_14h &= delMask;
    // and reset them with new value
    reg_14h |= range << _RANGE0;

    // restore highest bits
    delMask = ((1 << 8) - 1) ^ att_mask;
    reg_14h &= delMask;
    reg_14h |= attention;

    writeReg(E_SETUP_ACC, reg_14h);
    // update g factor
    calculateGFactor();
}

BMA020::BMA020RANGE BMA020::getRange()
{
    uint8_t reg_14h = (readReg(E_SETUP_ACC) & _bit_mask_Range) >> _RANGE0;
    return static_cast<BMA020RANGE>(reg_14h);
}

void BMA020::setBandwidth(BMA020::BMA020BANDWIDTH bandwidth)
{
    auto reg_14h = readReg(E_SETUP_ACC);
    // preserve highest bits
    byte attention = reg_14h & att_mask;

    //11100111b = 11111111b ^ 00011000b
    auto delMask = ((1 << 8) - 1) ^ _bit_mask_Bandwidth;
    // clear wanted bits
    reg_14h &= delMask;
    // and reset them with new value
    reg_14h |= (bandwidth << _BW0);

    // restore highest bits
    delMask = ((1 << 8) - 1) ^ att_mask;
    reg_14h &= delMask;
    reg_14h |= attention;

    writeReg(E_SETUP_ACC, reg_14h);
}

BMA020::BMA020BANDWIDTH BMA020::getBandWidth()
{
    uint8_t reg_14h = (readReg(E_SETUP_ACC) & _bit_mask_Bandwidth) >> _BW0;
    return static_cast<BMA020BANDWIDTH>(reg_14h);
}

BMA020::raw_acc_t& BMA020::getRawData()
{
    update_acc(_raw_data);
    return _raw_data;
}

BMA020::g_data_t& BMA020::getGData()
{
    update_acc(_raw_data);
    _g_data.acc_g_x = static_cast<double_t>(_raw_data.acc_x) * _gfactor;
    _g_data.acc_g_y = static_cast<double_t>(_raw_data.acc_y) * _gfactor;
    _g_data.acc_g_z = static_cast<double_t>(_raw_data.acc_z) * _gfactor;

    return _g_data;
}

BMA020::BMA020()
    : TwoWire()
{
}

BMA020::BMA020(uint8_t adr, uint8_t sda, uint8_t scl)
    : TwoWire()
{
    begin(sda, scl, adr);
}

bool BMA020::begin(uint8_t sda, uint8_t scl, uint8_t adr)
{
    TwoWire::begin(sda, scl);
    _adr = adr;
    // wait a bit for things to settle
    delay(20);
    bool ok = isOk();
    if (!ok)
        return ok;

    ok &= calculateGFactor();
    return ok;
}

bool BMA020::isOk()
{
    return getChipId() == _DEFAULT_CHIPID;
}
