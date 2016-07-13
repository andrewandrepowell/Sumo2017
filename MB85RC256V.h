/**
 * @file MB85RC256V.h
 * @author Andrew Powell
 * @date 11 July 2016
 * @brief Platform independent library for the MB85RC256V.
 * 
 * This library can be used to operate the MB85RC256V FRAM for access to
 * 32 kilo-bytes of non-volatile memory. 
 * 
 * @see https://www.fujitsu.com/us/Images/MB85RC256V-DS501-00017-3v0-E.pdf
 */


#ifndef MB85RC256V_H_   
#define MB85RC256V_H_

/* Standard C includes. */
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define MB85RC256V_BASE_SLAVE_ADDR       ( 0x50 )       /* Base slave address for device. */

#ifdef __cplusplus
extern "C" {
#endif

    /**
     * @brief Undefined base I2C transaction.
     * 
     * The user must define separate write and read transactions whose signature 
     * follows the MB85RC256VI2CMethod type definition. 
     * 
     * The definition of a I2C Write transaction should somewhat follow the 
     * pseudo code below. 
     * 
     * @code{.c}
     * bool MB85RC256VI2CWriteMethodDef( uint8_t slave_addr, uint8_t* data, size_t len, void* param )
     * {
     * I2C_START( );
     * I2C_WRITE_BYTE( ( slave_addr << 1 ) | 0 );
     * while ( len-- )
     * {
     * I2C_WRITE_BYTE( *(data++) );
     * }
     * I2C_STOP( );
     * return true;
     * }
     * @endcode
     * 
     * The definition of a I2C Read transaction should somewhat follow the 
     * pseudo code below. 
     * 
     * @code{.c}
     * bool MB85RC256VI2CReadMethodDef( uint8_t slave_addr, uint8_t* data, size_t len, void* param )
     * {
     * I2C_START( );
     * I2C_WRITE_BYTE( ( slave_addr << 1 ) | 1 );
     * while ( len-- )
     * {
     * *(data++) = I2C_READ_BYTE( ( len ) ? ACK : NACK );
     * }
     * I2C_STOP( );
     * return true;
     * }
     * @endcode
     * 
     * @param slave_addr 7-bit slave address to the MB85RC256V.
     * @param data Data to write.
     * @param len Length of data in bytes.
     * @param param A user-define value specified in vMB85RC256VSetup. 
     * @return A value indicating whether or not the transaction is successful. Should
     * only return either true or false.
     */
    typedef bool ( *MB85RC256VI2CMethod )( uint8_t slave_addr, uint8_t* data, size_t len, void* param );
    
    /** @brief MB85RC256V object. */
    typedef struct MB85RC256V
    {
        uint8_t slave_addr;                 /**< Slave address to device. */
        MB85RC256VI2CMethod write_ptr;      /**< Pointer to defined I2C Write transaction. @see MB85RC256VI2CMethod. */
        MB85RC256VI2CMethod read_ptr;       /**< Pointer to defined I2C Read transaction. @see MB85RC256VI2CMethod. */
        void* param;                        /**< User specified value that is passed into the functions related to I2C transactions. */
    }
    MB85RC256V;
    
    /**
     * @brief Configures a MB85RC256V object.
     * 
     * The states of a2, a1, and a0 are configured directly on the hardware itself.
     * It is important to make sure the inputs to a2, a1, and a0 for vMB85RC256VSetup
     * reflect the hardware configuration.
     * 
     * @param ptr The MB85RC256V object.
     * @param a2 See description.
     * @param a1 See description.
     * @param a0 See description.
     * @param write_ptr Pointer to the user-defined I2C Write transaction. 
     * @param read_ptr Pointer to the user-defined I2C Read transaction.
     * @param param A value specified by the user and is passed into the I2C functions. 
     */
    void vMB85RC256VSetup( MB85RC256V* ptr, bool a2, bool a1, bool a0,
        MB85RC256VI2CMethod write_ptr, MB85RC256VI2CMethod read_ptr, void* param );
    
    /**
     * @brief Writes data.
     * 
     * @param ptr The MB85RC256V object.
     * @param addr Starting 16-bit address.
     * @param data The data that will be written.
     * @param len The length of data in bytes.
     * @return A value indicating whether or not the transaction is successful. Should
     * only return either true or false.
     */
    bool vMB85RC256VWrite( MB85RC256V* ptr, uint16_t addr, uint8_t* data, size_t len );
    
    /**
     * @brief Reads data.
     * 
     * @param ptr The MB85RC256V object.
     * @param addr Starting 16-bit address.
     * @param data Where the data will be read into.
     * @param len The length of the desired data in bytes.
     * @return A value indicating whether or not the transaction is successful. Should
     * only return either true or false.
     */
    bool vMB85RC256Read( MB85RC256V* ptr, uint16_t addr, uint8_t* data, size_t len );
    
#ifdef __cplusplus
}
#endif

#endif /* MB85RC256V_H_ */