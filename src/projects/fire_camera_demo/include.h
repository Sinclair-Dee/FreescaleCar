#ifndef __INCLUDE_H__
#define __INCLUDE_H__

#include  "common.h"

/*
 * Include 用户自定义的头文件
 */

#include  "gpio.h"       //IO口操作
#include  "uart.h"      //串口
#include  "lptmr.h"     //低功耗定时器(延时)
#include  "pit.h"
#include  "LED.h"
#include  "dma.h"
#include  "FTM.h"
#include "BL144002.h"
#include "lcd.h"
#include "ov7725.h"
#include "spi.h"
#include "NRF24L0.h"
#include "NRF24L0_MSG.h"
#include "OV7725.h"
#include "key.h"



extern volatile u8	img_flag ;		//图像状态


#endif  //__INCLUDE_H__
