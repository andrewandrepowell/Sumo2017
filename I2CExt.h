/**
 * @file I2CExt.h
 * @author Andrew Powell
 * @date 8 July 2016
 * @brief Extension to the PIC32 I2C functions.
 *
 * A lot of devices with Slave I2C interfaces tend to operate
 * the same. This library is created to avoid having to re-write
 * the same code to use different devices. 
 */

#ifndef I2CEXT_H_
#define I2CEXT_H_

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
	 * @brief Configures the selected I2C Channel. 
	 *
	 * @param channel Selected Master I2C interface.
	 * @param sourceClock Frequency of the clock that sources the I2C hardware. 
	 * @param i2cClock Desired frequency of the Master I2C interface.
	 * @return Boolean indicating whether or not the operation was successful.
	 */
	BOOL bI2CSetup( I2C_MODULE channel, UINT32 sourceClock, UINT32 i2cClock );

	/**
	 * @brief Write a single byte to the selected Msater I2C interface.
	 *
	 * @param channel Selected Master I2C interface.
	 * @param data The byte written to the selected Master I2C interface.
	 * @return Boolean indicating whether or not the operation was successful.
	 */
	BOOL bI2CWriteByte( I2C_MODULE channel, BYTE data );

	/**
	 * @brief Read a single byte from the selected Msater I2C interface.
	 *
	 * @param channel Selected Master I2C interface.
	 * @param data The byte read from the selected Master I2C interface.
	 * @param ACK Boolean where TRUE indicates ACK is sent and FALSE indicates NACK is sent. 
	 * @return Boolean indicating whether or not the operation was successful.
	 */
	BOOL ucI2CReadByte( I2C_MODULE channel, BYTE* data, BOOL ACK );

	/**
	 * @brief Starts an I2C transaction over the selected Master I2C interface.
	 *
	 * @param channel Selected Master I2C interface.
	 * @return Boolean indicating whether or not the operation was successful.
	 */
	BOOL bI2CStart( I2C_MODULE channel );

	/**
	 * @brief Stops a I2C transaction over the selected Master I2C interface.
	 *
	 * @param channel Selected Master I2C interface.
	 */
	void vI2CStop( I2C_MODULE channel );

	/**
	 * @brief Perform a full write transaction over the selected Master I2C interface.
	 *
	 * @param channel Selected Master I2C interface.
	 * @param slave_addr Address of Slave I2C interface.
	 * @param data The data that will be written.
	 * @param len The number of bytes that will be sent.
	 * @return Boolean indicating whether or not the operation was successful.
	 */
	BOOL bI2CWriteTrans( I2C_MODULE channel, BYTE slave_addr, BYTE* data, size_t len );

	/**
	 * @brief Perform a full read transaction over the selected Master I2C interface.
	 *
	 * @param channel Selected Master I2C interface.
	 * @param slave_addr Address of Slave I2C interface.
	 * @param data The data that will be read.
	 * @param len The number of bytes that will be read
	 * @return Boolean indicating whether or not the operation was successful.
	 */
	BOOL bI2CReadTrans( I2C_MODULE channel, BYTE slave_addr, BYTE* data, size_t len );

#ifdef __cplusplus
}
#endif

#endif /* I2CEXT_H_ */