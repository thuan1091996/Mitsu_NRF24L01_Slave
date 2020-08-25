/*
 * UART.c
 *
 *  Created on: Nov 5, 2018
 *      Author: Itachi
 */
#include "include.h"
#include "UART.h"

/* --------UART1 Initialization-----------------------
 * UART1 - RX pin PB0 - TX pin PB1
 * To communicate with PLC FX series - 9600, 7 bit data, Even parity and 1 bit STOP
 * Using system clock
 * In order to change, change the PORT or pin or different configuration for UART
 * Input:  No
 * Output: No
-------------------------------------------------------*/
void UART_Init(){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);//select UART channel
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);//select PORT for UART1
    GPIOPinConfigure(GPIO_PB0_U1RX);            //PIN mux for UART1
    GPIOPinConfigure(GPIO_PB1_U1TX);            //PIN mux for UART1
    GPIOPinTypeUART(GPIO_PORTB_BASE, RX1_PIN |TX1_PIN);
    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 9600,(UART_CONFIG_WLEN_7 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_EVEN));
    UARTFIFOEnable(UART1_BASE);
}

