#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"
#include "osal_snv.h"

#include "hal_uart.h"

#include "SimpleBLESerialUart.h"

uint8 SBP_UART_STUDY_CMD = 0xE0;
uint8 SBP_UART_STUDY_CMD_LEN = 1;

UartState u_state;
uint8 UartBuffer[SBP_UART_RX_BUF_SIZE];

//static uint8 sendMsgTo_TaskID;

/*该函数将会在任务函数的初始化函数中调用*/
void SbpHalUART_Init(uint8 taskID) {
	//调用uart初始化代码
	SbpHalUARTInit();
	//记录任务函数的taskID，备用
	// sendMsgTo_TaskID = taskID;
}

/*uart初始化代码，配置串口的波特率、流控制等*/
void SbpHalUARTInit() {
	halUARTCfg_t uartConfig;

	// configure UART
	uartConfig.configured = TRUE;
	uartConfig.baudRate = SBP_UART_BR; //波特率
	uartConfig.flowControl = SBP_UART_FC; //流控制
	uartConfig.flowControlThreshold = SBP_UART_FC_THRESHOLD; //流控制阈值，当开启flowControl时，该设置有效
	uartConfig.rx.maxBufSize = SBP_UART_RX_BUF_SIZE; //uart接收缓冲区大小
	uartConfig.tx.maxBufSize = SBP_UART_TX_BUF_SIZE; //uart发送缓冲区大小
	uartConfig.idleTimeout = SBP_UART_IDLE_TIMEOUT;
	uartConfig.intEnable = SBP_UART_INT_ENABLE; //是否开启中断
	uartConfig.callBackFunc = SbpHalUARTReadCallback; //uart接收回调函数，在该函数中读取可用uart数据

	// start UART
	// Note: Assumes no issue opening UART port.
	(void) HalUARTOpen(SBP_UART_PORT, &uartConfig);

	return;
}

void studyCompletedAndBroadcastData() {
	HalLcdWriteString("write ...", HAL_LCD_LINE_4);
//	if (osal_snv_write(0xE0, UartBuffer[0], UartBuffer) == SUCCESS) {
//		HalLcdWriteString("write ok", HAL_LCD_LINE_4);
//	}
}

void SbpHalUARTRead(uint8 port, uint8 *buf, uint16 len);
/*uart接收回调函数*/
uint16 numBytes;
uint16 i, point, irdatalen;
uint8 pktBuffer[SBP_UART_RX_BUF_SIZE];
uint8 UART_PORT_HAVE_READ = 0;

void SbpHalUARTReadCallback(uint8 port, uint8 event) {
	UART_PORT_HAVE_READ = 0;
	//UART_HAL_DELAY(15000);
	numBytes = Hal_UART_RxBufLen(port);
	HalLcdWriteStringValue("numBytes", numBytes, 10, HAL_LCD_LINE_2);
	if (numBytes > 0 && (u_state == IR_DATA_STUDY_CMD_START_BEGIN_STATE)) {
		SbpHalUARTRead(port, pktBuffer, numBytes);
		//HalLcdWriteStringValue("pktBuffer", pktBuffer[0], 16, HAL_LCD_LINE_3);
		if (pktBuffer[0] == SBP_UART_STUDY_CMD) {
			osal_memset(UartBuffer, 0, SBP_UART_RX_BUF_SIZE);
			point = 0;
			irdatalen = 0;
			u_state = IR_DATA_STUDY_CMD_START_END_STATE;
		}
	} else if (numBytes > 0 && (u_state == IR_DATA_STUDY_CMD_START_END_STATE)) {
		SbpHalUARTRead(port, pktBuffer, numBytes);
		if (irdatalen == 0) {
			irdatalen = pktBuffer[0];
		}
		osal_memcpy(UartBuffer + point, pktBuffer, numBytes);
		point += numBytes;
		if (irdatalen == point) {
			u_state = IR_DATA_STUDY_CMD_RECV_END_STATE;
			studyCompletedAndBroadcastData();
		}
	} else if (numBytes > 0 && (u_state == IR_DATA_SEND_BEGIN_STATE)) {
		SbpHalUARTRead(port, pktBuffer, numBytes);
		HalLcdWriteStringValue("send resp:", pktBuffer[0], 16, HAL_LCD_LINE_6);
		u_state = IR_DATA_SEND_END_RESP_STATE;
	}

	if (numBytes > 0 && UART_PORT_HAVE_READ == 0) {
		SbpHalUARTRead(port, pktBuffer, numBytes);
		HalLcdWriteStringValue("HAVE_READ", pktBuffer[0], 16, HAL_LCD_LINE_6);
	}
}

void SbpHalUARTRead(uint8 port, uint8 *buf, uint16 len) {
	//UART_HAL_DELAY(1000);
	HalUARTRead(port, pktBuffer, numBytes);
	UART_PORT_HAVE_READ = 1;
}

//static uint8 data_len = 0, exsit_data_len = 0, send_times = 0, send_len = 0;
//#define one_time_data_len 125
void SbpHalUARTWrite(uint8 *pBuffer, uint16 length) {

//	data_len = osal_strlen(pBuffer);
//
//	do {
//
//		if(Hal_UART_TxBufLen() > 0){
//			continue;
//		}
//
//		if (data_len <= one_time_data_len) {
//			HalUARTWrite(SBP_UART_PORT, pBuffer, length);
//			exsit_data_len = 0;
//
//		} else if (exsit_data_len == 0 || exsit_data_len >= one_time_data_len) {
//			exsit_data_len = data_len - one_time_data_len;
//			HalUARTWrite(SBP_UART_PORT, (pBuffer + (send_times * one_time_data_len)), one_time_data_len);
//
//		} else if (exsit_data_len > 0 && exsit_data_len < one_time_data_len) {
//			HalUARTWrite(SBP_UART_PORT, (pBuffer + (send_times * one_time_data_len)), exsit_data_len);
//			exsit_data_len = 0;
//		} else {
//		}
//
//		send_times++;
//
//	} while (exsit_data_len > 0);
//
//	data_len = 0;
//	send_len = 0;
//	exsit_data_len = 0;
//	send_times = 0;

	do {
		UART_HAL_DELAY(10);
	} while (Hal_UART_TxBufLen() > 0);

	HalUARTWrite(SBP_UART_PORT, pBuffer, length);
}
