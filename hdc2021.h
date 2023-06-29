/**
 *  @file  hdc2021.h
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
#ifndef hdc2021_h_
#define hdc2021_h_

#include "stdint.h"
#include "stdbool.h"
#include "stddef.h"
#include "string.h"

/**
 *  HDC2021
 *  超低消費電力の温度湿度センサー
 *  https://www.ti.com/product/ja-jp/HDC2021
 *
 *  Refrences https://www.ti.com/jp/lit/ds/symlink/hdc2021.pdf?ts=1684202298439&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252Fja-jp%252FHDC2021 
 **/

#if defined(__cplusplus)
extern "C" {
#endif

/** エラー **/
typedef enum
{
    HDC2021_ERR_NONE = 0,       /** エラー無し **/
    HDC2021_ERR_NOT_CONNECT,    /** 接続エラー **/
    HDC2021_ERR_NOT_RESPONSE,   /** レスポンス異常 **/
    HDC2021_ERR_ID_MISMATCH     /** ID不一致 **/
}HDC2021Err;


/** 割り込み **/
#define HDC2021_INT_EN_NONE    (0)         /** 割込み無し **/
#define HDC2021_INT_EN_HL      (1 << 3)    /** 湿度スレッショルド Low **/
#define HDC2021_INT_EN_HH      (1 << 4)    /** 湿度スレッショルド High **/
#define HDC2021_INT_EN_TL      (1 << 5)    /** 温度スレッショルド Low **/
#define HDC2021_INT_EN_TH      (1 << 6)    /** 温度スレッショルド High **/
#define HDC2021_INT_EN_DRDY    (1 << 7)    /** データ準備完了 **/

/** 割込みピン極性 **/
typedef enum
{
    HDC2021_INT_POL_LOW = 0,                /** アクティブ Low **/
    HDC2021_INT_POL_HIGH,                   /** アクティブ High **/

}HDC2021IntPol;

/** 計測モード **/
typedef enum
{
    HDC2021_CC_ONESHOT = 0,
    HDC2021_CC_2MIN,
    HDC2021_CC_1MIN,
    HDC2021_CC_10SEC,
    HDC2021_CC_5SEC,
    HDC2021_CC_1SEC,
    HDC2021_CC_500MSEC,
    HDC2021_CC_200MSEC
}HDC2021CC;


/** 
 * 温度・湿度データ
 * 
 * 温度データ：実データの10倍
 **/
typedef struct 
{
    int16_t temperature;
    int16_t humidity;
}HDC2021Data;


typedef bool (*hdc2021write)(uint8_t addres, uint8_t reg, const uint8_t* src, size_t size);
typedef bool (*hdc2021read)(uint8_t addres, uint8_t reg, uint8_t* dst, size_t size);
typedef void (*hdc2021waitmsec)(int msec);


HDC2021Err hdc2021_init(uint8_t address, hdc2021write write, hdc2021read read, hdc2021waitmsec wait);
HDC2021Err hdc2021_start(HDC2021CC mode);
HDC2021Err hdc2021_stop(void);
HDC2021Err hdc2021_readData(HDC2021Data* data);
HDC2021Err hdc2021_readStatus(uint8_t* status);
void hdc2021_setInterrupt(uint8_t int_en, HDC2021IntPol pol);
const char* hdc2021_getErrorString(HDC2021Err err);


#ifdef __cplusplus
}
#endif

#endif /** hdc2021_h_ **/
