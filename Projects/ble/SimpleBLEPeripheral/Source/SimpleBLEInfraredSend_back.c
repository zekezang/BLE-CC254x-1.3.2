/*
 * SimpleBLEInfraredSend.c
 *
 *  Created on: 2013-12-19
 *      Author: zekezang
 */
#include "hal_timer.h"
#include "hal_lcd.h"
#include "SimpleBLEInfraredSend.h"

#define  IRLED P0_4

volatile uint32 count;
volatile uint32 end_count;
volatile uint8 flag;
static uint8 isInitTimer3 = 0;
uint8 i = 0;

/**
 * LOCAL FUNCTIONS
 */
static void initTimer3();
static void GPIO_LedsInit();
static void showsomething(uint8 timerId, uint8 channel, uint8 channelMode);

void sendIRData(uint8 addr1, uint8 addr2, uint8 value) {
	initTimer3();
	HalTimerStart(HAL_TIMER_0, 26);

	end_count = 346; //9ms
	flag = 1;
	count = 0;
	while (count < end_count) {
		if (count >= end_count - 1) {
			break;
		}
	}

	end_count = 173; //4.5ms
	flag = 0;
	count = 0;
	while (count < end_count) {
		if (count >= end_count - 1) {
			break;
		}
	}

	for (i = 0; i < 8; i++) {
		end_count = 21;
		flag = 1;
		count = 0;
		while (count < end_count) {
			if (count == end_count - 1) {
				break;
			}
		}
		if (addr1 - (addr1 / 2) * 2) {
			end_count = 64;
		} else {
			end_count = 21;
		}
		flag = 0;
		count = 0;
		while (count < end_count) {
			if (count == end_count - 1) {
				break;
			}
		}
		addr1 = addr1 >> 1;
	}

	HalTimerStop(HAL_TIMER_0);
}

static void showsomething(uint8 timerId, uint8 channel, uint8 channelMode) {
	count++;
	//HalLcdWriteStringValue("flag:", flag, 10, HAL_LCD_LINE_6);
	if (flag == 1) {
		//HalLcdWriteStringValue("count:", count, 10, HAL_LCD_LINE_6);
		IRLED=~IRLED;
	} else {
		IRLED = 0;
	}
}

static void initTimer3() {
	if (isInitTimer3 == 0) {
		GPIO_LedsInit();
		HalTimerInit();
		HalTimerConfig(HAL_TIMER_0, HAL_TIMER_MODE_CTC, HAL_TIMER_CHANNEL_SINGLE, HAL_TIMER_CH_MODE_OUTPUT_COMPARE, TRUE, showsomething);
		isInitTimer3 = 1;
	}
	count = 0;
	flag = 0;
	IRLED = 0;
}

static void GPIO_LedsInit() {
	IRLED = 0;
	P0DIR |= BV(4);
	P0SEL &= ~(BV(4));
}
