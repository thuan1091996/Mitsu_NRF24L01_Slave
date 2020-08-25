/*
 * UART.h
 *
 *  Created on: Nov 5, 2018
 *      Author: Itachi
 */

#ifndef UART_H_
#define UART_H_

#define RX1_PIN         GPIO_PIN_0
#define TX1_PIN         GPIO_PIN_1

/* --------UART1 Initialization-----------------------
 * UART1 - RX pin PB0 - TX pin PB1
 * To communicate with PLC FX series - 9600, 7 bit data, Even parity and 1 bit STOP
 * Using system clock
 * In order to change, change the PORT or pin or different configuration for UART
 * Input:  No
 * Output: No
-------------------------------------------------------*/
void UART_Init(void);           //UART1 Initialization


#endif /* UART_H_ */
