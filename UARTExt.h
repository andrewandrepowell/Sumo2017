/**
 * @file UARTExt.h
 * @author Andrew Powell
 * @date 8 July 2016
 * @brief Extension to the PIC32 UART functions.
 *
 * This extension facilitates the usage of the 
 * PIC32's UART. The source file contains overridden definitions
 * for the _mon_getc and _mon_putc methods. These definitions
 * were necessary just to get the standard input/output to operate
 * correctly.
 */

#ifndef UARTEXT_H_
#define UARTEXT_H_

/* PIC32 includes. */
#include <plib.h>

/* Standard C includes. */
#include <stdbool.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Configures the UART so that standard input/output library performs
     * correctly.
     * 
     * @param channel The selected UART interface that will be configured.
     * @param source_clock The frequency of the clock driving the UART.
     * @param baud_rate The desired frequency of the UART interface.
     */
	void vSTDIOSetup( UART_MODULE channel, UINT32 source_clock, UINT32 baud_rate );

#ifdef __cplusplus
}
#endif

#endif /* UARTEXT_H_ */