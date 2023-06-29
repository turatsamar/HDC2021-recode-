/**
 *  @file  hdc2021.c
 *  @brief hdc2021ドライバ
 *
 *  @details TI製 温度湿度センサー HDC2021ドライバ
 *
 *  @author  Yslab Corp. Shouhei Hasegawa (hasegawa@yslab.co.jp)
 *
 *  @internal
 *      Created  2023/05/15
 * ===================================================================
 **/
#include "hdc2021.h"

#define HDC2021_DATA_RANG       (0xFFFF)    /** 16bit **/
#define HDC2021_TEMP_RENG       (125 + 40)  /** 計測可能温度範囲 -40 ～ 125℃ **/
#define HDC2021_HUM_RENG        (100)

static const int32_t HDC2021_TEMP_PER_BIT = ((HDC2021_DATA_RANG * 10) / HDC2021_TEMP_RENG);
static const int32_t HDC2021_HUM_PER_BIT = ((HDC2021_DATA_RANG * 10) / HDC2021_HUM_RENG);

/** レジスタマップ Ref P.20 7.6 **/
static const uint8_t HDC2021_REG_TEMP_LOW        = {0x00};
static const uint8_t HDC2021_REG_TEMP_HIGH       = {0x01};
static const uint8_t HDC2021_REG_HUM_LOW         = {0x02};
static const uint8_t HDC2021_REG_HUM_HIGH        = {0x03};
static const uint8_t HDC2021_REG_STATUS          = {0x04};
static const uint8_t HDC2021_REG_TEMP_MAX        = {0x05};
static const uint8_t HDC2021_REG_HUM_MAX         = {0x06};
static const uint8_t HDC2021_REG_INT_ENABLE      = {0x07};
static const uint8_t HDC2021_REG_TEMP_OFFSET     = {0x08};
static const uint8_t HDC2021_REG_HUM_OFFSET      = {0x09};
static const uint8_t HDC2021_REG_TEMP_THR_L      = {0x0A};
static const uint8_t HDC2021_REG_TEMP_THR_H      = {0x0B};
static const uint8_t HDC2021_REG_RH_THR_L        = {0x0C};
static const uint8_t HDC2021_REG_RH_THR_H        = {0x0D};
static const uint8_t HDC2021_REG_DEVICE_CONFIG   = {0x0E};
static const uint8_t HDC2021_REG_MEAS_CONFIG     = {0x0F};
static const uint8_t HDC2021_REG_MFR_ID_LOW      = {0xFC};
static const uint8_t HDC2021_REG_MFR_ID_HIGH     = {0xFD};
static const uint8_t HDC2021_REG_DEVICE_ID_LOW   = {0xFE};
static const uint8_t HDC2021_REG_DEVICE_ID_HIGH  = {0xFF};

/** 設定レジスタの初期値 **/
static const uint8_t HDC2021_DEFAULT_STATUS             = {0x00};
static const uint8_t HDC2021_DEFAULT_TEMP_MAX           = {0x00};
static const uint8_t HDC2021_DEFAULT_HUM_MAX            = {0x00};
static const uint8_t HDC2021_DEFAULT_INT_ENABLE         = {0x00};
static const uint8_t HDC2021_DEFAULT_TEMP_OFFSET        = {0x00};
static const uint8_t HDC2021_DEFAULT_HUM_OFFSET         = {0x00};
static const uint8_t HDC2021_DEFAULT_TEMP_THR_L         = {0x01};
static const uint8_t HDC2021_DEFAULT_TEMP_THR_H         = {0xFF};
static const uint8_t HDC2021_DEFAULT_RH_THR_L           = {0x00};
static const uint8_t HDC2021_DEFAULT_RH_THR_H           = {0xFF};
static const uint8_t HDC2021_DEFAULT_DEVICE_CONFIG      = {0x00};
static const uint8_t HDC2021_DEFAULT_MEAS_CONFIG        = {0x00};

/** レジスタ固定値 **/
static const uint8_t HDC2021_MANUFACTURER_ID[2]         = {0x49, 0x54};
static const uint8_t HDC2021_DEVICE_ID[2]               = {0xD0, 0x07};



/** ステータス レジスタ設定値 Ref P.23 7.6.5 **/
typedef union
{
    uint8_t data;
    struct
    {
        uint8_t res     :3;
        uint8_t hl      :1;
        uint8_t hh      :1;
        uint8_t tl      :1;
        uint8_t th      :1;
        uint8_t drdy    :1;
    }bit;
}HDC2021RegStatus;

/** 割り込みイネーブル レジスタ設定値 Ref P.25 7.6.8 **/
typedef union
{
    uint8_t data;
    struct
    {
        uint8_t res     :3;
        uint8_t hl      :1;
        uint8_t hh      :1;
        uint8_t tl      :1;
        uint8_t th      :1;
        uint8_t drdy    :1;
    }bit;
}HDC2021RegInterruptEnable;

/** デバイス構成 レジスタ設定値 Ref P.30 7.6.15 **/
typedef union
{
    uint8_t data;
    struct
    {
        uint8_t int_mode    :1;
        uint8_t int_pol     :1;
        uint8_t drdy        :1;
        uint8_t heat        :1;
        uint8_t cc          :3;
        uint8_t reset       :1;
    }bit;
}HDC2021RegDeviceConfig;

/** 測定構成 レジスタ設定値 Ref P.31 7.6.16 **/
typedef union
{
    uint8_t data;
    struct
    {
        uint8_t meas_trig   :1;
        uint8_t meas_conf   :2;
        uint8_t res         :1;
        uint8_t hacc        :2;
        uint8_t tacc        :2;
    }bit;
}HDC2021RegMeasurementConfig;

#pragma pack(1)
typedef struct 
{
    uint8_t temp_max;
    uint8_t hum_max;
    HDC2021RegInterruptEnable int_en;
    uint8_t temp_offset;
    uint8_t hum_offset;
    uint8_t temp_thr_l;
    uint8_t temp_thr_h;
    uint8_t rh_thr_l;
    uint8_t rh_thr_h;
    HDC2021RegDeviceConfig device_conf;
    HDC2021RegMeasurementConfig meas_conf;
}HDC2021Regster;
#pragma pack(0)


static HDC2021Err hdc2021Softreset(void);
static HDC2021Err hdc2021CheckID(void);
static void hdc2021SetDefaultReagster(void);
static bool hdc2021Write(const uint8_t reg, const uint8_t* val, size_t size);
static bool hdc2021Read(const uint8_t reg, uint8_t* dst, size_t size);
static void hdc2021Waitmsec(int msec);
static bool hdc2021I2CWrite(uint8_t data);
static void hdc2021I2CRead(uint8_t* data);

static uint8_t _address = 0x80;
static HDC2021Regster _reg;
static hdc2021write _write;
static hdc2021read _read;
static hdc2021waitmsec _wait;

HDC2021Err hdc2021_init(uint8_t address, hdc2021write write, hdc2021read read, hdc2021waitmsec wait)
{
    HDC2021Err err = HDC2021_ERR_NOT_CONNECT;

    do
    {
        _write = write;
        _read = read;
        _wait = wait;
        _address = address;
        hdc2021SetDefaultReagster();
        if ((err = hdc2021Softreset()) != HDC2021_ERR_NONE)
            break;

        err = hdc2021CheckID();
    } while (0);

    return err;
}

HDC2021Err hdc2021_start(HDC2021CC mode)
{
    HDC2021Err err = HDC2021_ERR_NOT_RESPONSE;

    do
    {
        _reg.device_conf.bit.cc = mode;
        _reg.meas_conf.bit.meas_trig = 1;
        if (!hdc2021Write(HDC2021_REG_TEMP_MAX, (const uint8_t*)&_reg, sizeof(HDC2021Regster)))
            break;

        err = HDC2021_ERR_NONE;
    } while (0);

    return err;
}

HDC2021Err hdc2021_stop(void)
{
    HDC2021Err err = HDC2021_ERR_NOT_RESPONSE;

    do
    {
        _reg.meas_conf.bit.meas_trig = 0;
        if (!hdc2021Write(HDC2021_REG_TEMP_MAX, (const uint8_t*)&_reg, sizeof(HDC2021Regster)))
            break;

        err = HDC2021_ERR_NONE;
    } while (0);

    return err;
}

HDC2021Err hdc2021_readData(HDC2021Data* data)
{
    HDC2021Err err = HDC2021_ERR_NOT_RESPONSE;
    int16_t read[2];

    do
    {
        memset(read, 0, sizeof(read));
        data->temperature = 0;
        data->humidity = 0;

        if (!hdc2021Read(HDC2021_REG_TEMP_LOW, (uint8_t*)read, sizeof(read)))
            break;

        //data->temperature = read[0] * HDC2021_TEMP_PER_BIT;
        //data->humidity = read[1] * HDC2021_HUM_PER_BIT;
        data->temperature = (int16_t)((int32_t)(read[0] * 100) / HDC2021_TEMP_PER_BIT) - 400;
        data->humidity = (int16_t)((int32_t)(read[1] * 10) / HDC2021_HUM_PER_BIT);

        err = HDC2021_ERR_NONE;
    } while (0);

    return err;
}

HDC2021Err hdc2021_readStatus(uint8_t* status)
{
    HDC2021Err err = HDC2021_ERR_NOT_RESPONSE;
    uint8_t read = 0;
    do
    {

        if (!hdc2021Read(HDC2021_REG_STATUS, &read, sizeof(read)))
            break;

        err = HDC2021_ERR_NONE;
    } while (0);

    return err;
}

void hdc2021_setInterrupt(uint8_t int_en, HDC2021IntPol pol)
{
    _reg.int_en.data = int_en;
    _reg.device_conf.bit.int_pol = pol;

    if (int_en == HDC2021_INT_EN_NONE)
        _reg.device_conf.bit.drdy = 0;
    else
        _reg.device_conf.bit.drdy = 1;
}

const char* hdc2021_getErrorString(HDC2021Err err)
{
    const char* ptr = NULL;

    switch (err)
    {

        case HDC2021_ERR_NONE:
            ptr = "HDC2021 Error None";
            break;
        case HDC2021_ERR_NOT_CONNECT:
            ptr = "HDC2021 Error Not Connected";
            break;
        case HDC2021_ERR_NOT_RESPONSE:
            ptr = "HDC2021 Error Not Response";
            break;
        case HDC2021_ERR_ID_MISMATCH:
            ptr = "HDC2021 Error ID Mismatch";
            break;
        default:
            ptr = "HDC2021 Unknown Error";
            break;
    }
    return ptr;
}

HDC2021Err hdc2021Softreset(void)
{
    HDC2021Err err = HDC2021_ERR_NOT_CONNECT;
    HDC2021RegDeviceConfig reg = _reg.device_conf;

    do
    {
        reg.bit.reset = 1;
        if (hdc2021Write(HDC2021_REG_DEVICE_CONFIG, (const uint8_t*)&reg, sizeof(HDC2021RegDeviceConfig)))
            err = HDC2021_ERR_NONE;

        hdc2021Waitmsec(10);
    } while (0);

    return err;
}

HDC2021Err hdc2021CheckID(void)
{
    HDC2021Err err = HDC2021_ERR_NOT_RESPONSE;
    uint8_t id[2];

    do
    {
        if (!hdc2021Read(HDC2021_REG_DEVICE_ID_LOW, id, sizeof(id)))
            break;

        
        err = HDC2021_ERR_NONE;
        if (0 != memcmp(id, HDC2021_DEVICE_ID, sizeof(id)))
            err = HDC2021_ERR_ID_MISMATCH;

    } while (0);

    return err;
}

void hdc2021SetDefaultReagster(void)
{
    memset((void*)&_reg, 0x00, sizeof(HDC2021Regster));
    _reg.temp_thr_l = HDC2021_DEFAULT_TEMP_THR_L;
    _reg.temp_thr_h = HDC2021_DEFAULT_TEMP_THR_H;
    _reg.rh_thr_l = HDC2021_DEFAULT_RH_THR_L;
    _reg.rh_thr_h = HDC2021_DEFAULT_RH_THR_H;
}

bool hdc2021Write(const uint8_t reg, const uint8_t* val, size_t size)
{
    return _write(_address, reg, val, size);
}

bool hdc2021Read(const uint8_t reg, uint8_t* dst, size_t size)
{
    return _read(_address, reg, dst, size);
}

void hdc2021Waitmsec(int msec)
{
    _wait(msec);
}
