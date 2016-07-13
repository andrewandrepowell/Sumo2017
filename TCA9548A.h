/**
 * @file TCA9548A.h
 * @author Andrew Powell
 * @date 11 July 2016
 * @brief Platform independent library for the TCA9548.
 * 
 * This library can be used to operate the TCA9548A I2C 8 Slave to 1 Master Mux.
 * 
 * @see https://cdn-learn.adafruit.com/downloads/pdf/adafruit-tca9548a-1-to-8-i2c-multiplexer-breakout.pdf
 */

#ifndef TCA9548A_H_    
#define TCA9548A_H_

/* Standard C includes. */
#include <stdbool.h>
#include <stdint.h>

#define TCA9548A_BASE_SLAVE_ADDR       ( 0x70 )     /* Base slave address for device. */

#ifdef __cplusplus
extern "C" {
#endif
    
    /**
     * @brief Undefined I2C Write transaction to the TCA9548A.
     * 
     * The user must define this function according to the platform. The 
     * definition should somewhat follow the pseudo code below. 
     * 
     * @code{.c}
     * bool bTCA9548AI2CMethodDef( uint8_t slave_addr, uint8_t data, void* param )
     * {
     * I2C_START( );
     * I2C_WRITE_BYTE( ( slave_addr << 1 ) | 0 );
     * I2C_WRITE_BYTE( data );
     * I2C_STOP( );
     * return true;
     * }
     * @endcode
     * 
     * @param slave_addr 7-bit slave address to the TCA9548A.
     * @param data 8-bit data to write.
     * @param param A user-define value specified in vTCA9548ASetup. 
     * @return A value indicating whether or not the transaction is successful. Should
     * only return either true or false.
     */
    typedef bool ( *bTCA9548AI2CMethod )( uint8_t slave_addr, uint8_t data, void* param );

    /** @brief TCA9548A object. */
    typedef struct TCA9548A
    {
        uint8_t slave_addr;             /**< Slave address to device. */
        bTCA9548AI2CMethod write_ptr;   /**< Pointer to defined I2C Write transaction. @see bTCA9548AI2CMethod. */
        void* param;                    /**< User specified value that is passed into the functions related to I2C transactions. */
    }
    TCA9548A;
    
    /**
     * @brief Configures a TCA9548A object.
     * 
     * The states of a2, a1, and a0 are configured directly on the hardware itself.
     * It is important to make sure the inputs to a2, a1, and a0 for vTCA9548ASetup
     * reflect the hardware configuration.
     * 
     * @param ptr The TCA9548A object.
     * @param a2 See description.
     * @param a1 See description.
     * @param a0 See description.
     * @param write_ptr Pointer to the user-defined I2C Write transaction. 
     * @param param A value specified by the user and is passed into the I2C functions. 
     */
    void vTCA9548ASetup( TCA9548A* ptr, bool a2, bool a1, bool a0 , 
            bTCA9548AI2CMethod write_ptr, void* param );
    
    /**
     * @brief Selects an I2C Slave interface according to a specified port.
     * 
     * @param ptr The TCA9548A object.
     * @param port The port of the desired I2C Slave interface.
     * @return A value indicating whether or not the transaction is successful. Should
     * only return either true or false.
     */
    static inline  __attribute__ ((always_inline))
    bool bTCA9548ASetPort( TCA9548A* ptr, int port )
    {
        return ptr->write_ptr( ptr->slave_addr, (1 << port), ptr->param );
    }

#ifdef __cplusplus
}
#endif

#endif /* TCA9548A_H_ */
