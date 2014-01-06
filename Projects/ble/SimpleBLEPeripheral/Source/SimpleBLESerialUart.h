/*
 * SimpleBLESerialUart1.h
 *
 *  Created on: 2013-12-28
 *      Author: zekezang
 */

#ifndef SIMPLEBLESERIALUART_H_
#define SIMPLEBLESERIALUART_H_

#ifdef __cplusplus
extern "C"
{
#endif

#define SBP_UART_PORT                  HAL_UART_PORT_0
#define SBP_UART_FC                    FALSE
#define SBP_UART_FC_THRESHOLD          48
#define SBP_UART_RX_BUF_SIZE           255
#define SBP_UART_TX_BUF_SIZE           128
#define SBP_UART_IDLE_TIMEOUT          6
#define SBP_UART_INT_ENABLE            TRUE
#define SBP_UART_BR                    HAL_UART_BR_9600

#define UART_HAL_DELAY(n) st( { volatile uint32 i; for (i=0; i<(n); i++) { }; } )

typedef enum {
	IR_DATA_STUDY_CMD_START_BEGIN_STATE,
	IR_DATA_STUDY_CMD_START_END_STATE,
	IR_DATA_STUDY_CMD_RECV_END_STATE,
	IR_DATA_SEND_BEGIN_STATE,
	IR_DATA_SEND_END_RESP_STATE,
} UartState;

// Serial Port Related
extern void SbpHalUART_Init(uint8 taskID);
extern void SbpHalUARTReadCallback(uint8 port, uint8 event);
void SbpHalUARTInit();
void SbpHalUARTWrite(uint8 *pBuffer, uint16 length);

#ifdef __cplusplus
}
#endif

#endif /* SIMPLEBLESERIALUART_H_ */
