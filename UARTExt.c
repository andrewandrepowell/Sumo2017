#include "UARTExt.h"

void vSTDIOSetup( UART_MODULE channel, UINT32 source_clock, UINT32 baud_rate )
{
	/* Configure the UART. */
    __XC_UART = channel+1;
    setbuf( stdin, NULL );
    setbuf( stdout, NULL );
	UARTConfigure( channel, UART_ENABLE_PINS_TX_RX_ONLY );
	UARTSetLineControl( channel, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1 );
	UARTSetDataRate( channel, source_clock, baud_rate );
	UARTEnable( channel, UART_ENABLE_FLAGS( UART_PERIPHERAL | UART_RX | UART_TX ) );
}

int _mon_getc( int canblock )
{
	UART_MODULE channel = __XC_UART-1;
    int recv;
    
    if ( canblock )
    {
        while ( !UARTReceivedDataIsAvailable( channel ) )
            continue;
    }
    else if ( !UARTReceivedDataIsAvailable( channel ) )
    {
        return -1;
    }
    
    recv = UARTGetDataByte( channel );
    
    return recv;
}

void _mon_putc( char c )
{
    UART_MODULE channel = __XC_UART-1;
    
	while( !UARTTransmitterIsReady( channel ) )
        continue;
    
	UARTSendDataByte( channel , c );
}