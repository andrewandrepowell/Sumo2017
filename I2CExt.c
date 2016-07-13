#include "I2CExt.h"

BOOL bI2CSetup( I2C_MODULE channel, UINT32 sourceClock, UINT32 i2cClock )
{
	UINT32 actual_clock;
    
	/* Configure the I2C clock speed. */
	actual_clock = I2CSetFrequency( channel, sourceClock, i2cClock );
	if( abs( actual_clock-i2cClock ) > i2cClock/10 )
	{
		return FALSE;
	}

	/* Enable the I2C bus. */
	I2CEnable( channel, TRUE );

	return TRUE;
}

BOOL bI2CWriteByte( I2C_MODULE channel, BYTE data )
{
	while( !I2CTransmitterIsReady( channel ) );

	if ( I2CSendByte( channel, data ) == I2C_MASTER_BUS_COLLISION )
	{
		return FALSE;
	}

	while( !I2CTransmissionHasCompleted( channel ) );

	if ( !I2CByteWasAcknowledged( channel ) )
	{
		return FALSE;
	}

	return TRUE;
}

BOOL ucI2CReadByte( I2C_MODULE channel, BYTE* data, BOOL ACK)
{ 
    if ( I2CReceiverEnable( channel, TRUE ) != I2C_SUCCESS )
        return FALSE;
    
	while ( !I2CReceivedDataIsAvailable( channel ) );

	I2CAcknowledgeByte ( channel, ACK );
	while ( !I2CAcknowledgeHasCompleted( channel ) );

	*data = I2CGetByte( channel );
    
    return TRUE;
}

BOOL bI2CStart( I2C_MODULE channel )
{
	I2C_STATUS  status;

	while( !I2CBusIsIdle( channel ) );

	if ( I2CStart( channel ) != I2C_SUCCESS )
	{
		return FALSE;
	}

	do
	{
		status = I2CGetStatus( channel );
	}
	while ( !(status & I2C_START) );

	return TRUE;
}

void vI2CStop( I2C_MODULE channel )
{
    I2C_STATUS  status;

    I2CStop( channel );

    do
    {
        status = I2CGetStatus( channel );
    }
    while ( !(status & I2C_STOP) );
}

BOOL bI2CWriteTrans( I2C_MODULE channel, BYTE slave_addr, BYTE* data, size_t len )
{
	I2C_7_BIT_ADDRESS slave7BitAddress;

	I2C_FORMAT_7_BIT_ADDRESS( slave7BitAddress, slave_addr, I2C_WRITE );
	
	if ( !bI2CStart( channel ) )
		return FALSE;
	
	if ( !bI2CWriteByte( channel, slave7BitAddress.byte ) )
		return FALSE;
	
	while ( len-- )
	{
		if ( !bI2CWriteByte( channel, *(data++) ) )
			return FALSE;
	}
	
	vI2CStop( channel );
	
	return TRUE;
}

BOOL bI2CReadTrans( I2C_MODULE channel, BYTE slave_addr, BYTE* data, size_t len )
{
	I2C_7_BIT_ADDRESS slave7BitAddress;

	I2C_FORMAT_7_BIT_ADDRESS( slave7BitAddress, slave_addr, I2C_READ );
	
	if ( !bI2CStart( channel ) )
		return FALSE;
	
	if ( !bI2CWriteByte( channel, slave7BitAddress.byte ) )
		return FALSE;
	
	while ( len-- )
	{
        if ( !ucI2CReadByte( channel, data++, ( len ) ? TRUE : FALSE ) )
            return FALSE;
	}
	
	vI2CStop( channel );
	
	return TRUE;
}