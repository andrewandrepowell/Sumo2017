/**
 * @file VCNL4010.h
 * @author Andrew Powell
 * @date 11 July 2016
 * @brief Platform independent library for the VCNL4010.
 * 
 * This library can be used to read proximity values from the VCNL4010 and also
 * take limited advantage of the VCNL4010's interrupt.
 * 
 * @warning This library does not support all the features implemented in the
 * VCNL4010. For instance, ambient light readings are not supported.
 * @see https://cdn-shop.adafruit.com/product-files/466/vcnl4010.pdf
 */

#ifndef VCNL4010_H_
#define VCNL4010_H_

/* Standard C includes. */
#include <stdbool.h>
#include <stdint.h>

/* Definitions related to the VCNL4010. */
#define VCNL4010_SLAVE_ADDR						( 0x13 )    /* Slave address of the VCNL4010. */
#define VCNL4010_REG_CMD_ADDR					( 0x80 )    /* Command Register address. */
#define VCNL4010_REG_PROX_RATE_ADDR				( 0x82 )    /* Proximity Rate Register address. */
#define VCNL4010_REG_IRLED_ADDR					( 0x83 )    /* IR LED Current Register address. */
#define VCNL4010_REG_PROX_MS_ADDR				( 0x87 )    /* Proximity Result Register address ( i.e. most significant byte ). */
#define VCNL4010_REG_PROX_LS_ADDR				( 0x88 )    /* Proximity Result Register address ( i.e. least significant byte ). */
#define VCNL4010_REG_INT_CNTRL_ADDR				( 0x89 )    /* Interrupt Control Register address. */
#define VCNL4010_REG_INT_STAT_ADDR				( 0x8E )    /* Interrupt Status Register address. */
#define VCNL4010_REG_PROX_MOD_TIM_ADJ_ADDR		( 0x8F )    /* Proximity Modulator Timing Adjustment Register Address. */

#define VCNL4010_REG_CMD_PROX_OD				( 3 )       /* Proximity On-Demand bit offset. */
#define VCNL4010_REG_CMD_PROX_DATA_RDY			( 5 )       /* Proximity Ready bit offset. */
#define VCNL4010_REG_CMD_PROX_EN                ( 1 )       /* Proximity Periodic Measurement enable bit offset. */
#define VCNL4010_REG_CMD_SELF_TIMED_EN          ( 0 )       /* Enables hardware necessary for self-timed measurements. */

#define VCNL4010_REG_IRLED_MAX_CURR             ( 20 )      /* Integer signifying the maximum current ( i.e. 20 mA ) for the IR LED Current Register. */

#define VCNL4010_REG_PROX_MOD_TIM_ADJ_DEF		( 0x01 )    /* Default values for the Proximity Modulator Timing Adjustment Register. */
#define VCNL4010_REG_PROX_MOD_TIM_ADJ_PROX_FREQ	( 3 )       /* Proximity Frequency bit offset. */

#define VCNL4010_REG_INT_CNTRL_PROX_RDY_EN		( 3 )       /* Enable Interrupt for Proximity bit offset. */
#define VCNL4010_REG_INT_STAT_PROX_RDY			( 3 )       /* Ready bit offset of Proximity Interrupt. */

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Undefined I2C Write transaction to the VCNL4010.
     * 
     * The user must define this function according to the platform. The 
     * definition should somewhat follow the pseudo code below. 
     * 
     * @code{.c}
     * bool bVCNL4010I2CWriteDef( unsigned char slave_addr, unsigned char reg_addr, unsigned char data, void* param )
     * {
     * I2C_START( );
     * I2C_WRITE_BYTE( ( slave_addr << 1 ) | 0 );
     * I2C_WRITE_BYTE( reg_addr );
     * I2C_WRITE_BYTE( data );
     * I2C_STOP( );
     * return true;
     * }
     * @endcode
     * 
     * @param slave_addr 7-bit slave address to the VCNL4010.
     * @param reg_addr 8-bit address to a register in the VCNL4010.
     * @param data 8-bit data to write to the selected register.
     * @param param A user-define value specified in vVCNL4010Setup. 
     * @return A value indicating whether or not the transaction is successful. Should
     * only return either true or false.
     */
	typedef bool (*bVCNL4010I2CWrite)( unsigned char slave_addr, unsigned char reg_addr, unsigned char data, void* param );
    
    /**
     * @brief Undefined I2C Read transaction to the VCNL4010.
     * 
     * The user must define this function according to the platform. The 
     * definition should somewhat follow the pseudo code below. 
     * 
     * @code{.c}
     * bool bVCNL4010I2CReadDef( unsigned char slave_addr, unsigned char* reg_addr, unsigned char data, void* param )
     * {
     * I2C_START( );
     * I2C_WRITE_BYTE( ( slave_addr << 1 ) | 0 );
     * I2C_WRITE_BYTE( reg_addr );
     * I2C_STOP( );
     * 
     * I2C_START( );
     * I2C_WRITE_BYTE( ( slave_addr << 1 ) | 1 );
     * *data = I2C_READ_BYTE( NACK );
     * I2C_STOP( );
     * return true;
     * }
     * @endcode
     * 
     * @param slave_addr 7-bit slave address to the VCNL4010.
     * @param reg_addr 8-bit address to a register in the VCNL4010.
     * @param data Where the 8-bits of data will be read into.
     * @param param A user-define value specified in vVCNL4010Setup. 
     * @return A value indicating whether or not the transaction is successful. Should
     * only return either true or false.
     */
	typedef bool (*bVCNL4010I2CRead)( unsigned char slave_addr, unsigned char reg_addr, unsigned char* data, void* param );

    /** @brief Possible frequencies.  */
	typedef enum VCNL4010ProxFreqParam
	{
		VCNL4010_390p625kHz, VCNL4010_781p25kHz, VCNL4010_1p5625MHz, VCNL4010_3p125MHz
	}
	VCNL4010ProxFreqParam;
    
    /** @brief Possible proximity sample rates.  */
    typedef enum VCNL4010ProxRateParam
    {
        VCNL4010_1p95mps,VCNL4010_3p91mps,VCNL4010_7p81mps,VCNL4010_16p23mps,
                VCNL4010_31p25mps,VCNL4010_62p5mps,VCNL4010_125mps,VCNL4010_250mps
    }
    VCNL4010ProxRateParam;

    /** @brief VCNL4010 object. */
	typedef struct VCNL4010
	{
		bVCNL4010I2CWrite write_ptr;    /**< Pointer to defined I2C Write transaction. @see bVCNL4010I2CWrite. */
		bVCNL4010I2CRead read_ptr;      /**< Pointer to defined I2C Read transaction. @see bVCNL4010I2CRead. */
		void* param;                    /**< User specified value that is passed into the functions related to I2C transactions. */
	}
	VCNL4010;

    /**
     * @brief Configures a VCNL4010 object.
     * 
     * @param ptr The VCNL4010 object.
     * @param write_ptr Pointer to the user-defined I2C Write transaction. 
     * @param read_ptr Pointer to the user-defined I2C Read transaction.
     * @param param A value specified by the user and is passed into the I2C functions. 
     * @see bVCNL4010I2CWrite and bVCNL4010I2CRead 
     */
	static inline  __attribute__ ((always_inline)) 
    void vVCNL4010Setup( VCNL4010* ptr, bVCNL4010I2CWrite write_ptr, bVCNL4010I2CRead read_ptr, void* param )
	{
		ptr->write_ptr = write_ptr;
		ptr->read_ptr = read_ptr;
		ptr->param = param;
	}

    /**
     * @brief Sets the current. 
     * 
     * @param ptr The VCNL4010 object.
     * @param current A 5-bit value that specifies the current. The max value is
     * 20 for 200 mA. @p current times 20 mA results in the current in mA.
     * @return A value indicating whether or not the transaction is successful. Should
     * only return either true or false.
     */
	static inline  __attribute__ ((always_inline))
    bool bVCNL4010SetIRCurrent( VCNL4010* ptr, int current ) /* current is unsigned 5 bits. Max is 20 for 200 mA. */
	{
		return ptr->write_ptr(
				VCNL4010_SLAVE_ADDR,
				VCNL4010_REG_IRLED_ADDR,
				(unsigned char)current, ptr->param );
	}

    /**
     * @brief Sets the proximity frequency.
     * 
     * @param ptr The VCNL4010 object.
     * @param prox_freq The desired frequency.
     * @return A value indicating whether or not the transaction is successful. Should
     * only return either true or false.
     */
	static inline  __attribute__ ((always_inline))
    bool bVCNL4010SetProxFreq( VCNL4010* ptr, VCNL4010ProxFreqParam prox_freq )
	{
		return ptr->write_ptr(
				VCNL4010_SLAVE_ADDR,
				VCNL4010_REG_PROX_MOD_TIM_ADJ_ADDR,
				(((unsigned char)prox_freq) << VCNL4010_REG_PROX_MOD_TIM_ADJ_PROX_FREQ) | VCNL4010_REG_PROX_MOD_TIM_ADJ_DEF,
				ptr->param  );
	}

    /**
     * @brief Sets the rate of proximity measurement. 
     * 
     * @param ptr The VCNL4010 object.
     * @param rate The desired rate.
     * @return A value indicating whether or not the transaction is successful. Should
     * only return either true or false.
     */
	static inline  __attribute__ ((always_inline))
    bool bVCNL4010SetProxRate( VCNL4010* ptr, VCNL4010ProxRateParam rate )
	{
		return ptr->write_ptr(
				VCNL4010_SLAVE_ADDR,
				VCNL4010_REG_PROX_RATE_ADDR,
				( unsigned char ) rate,
				ptr->param );
	}
    
    /**
     * @brief Starts the periodic mode operation. This mode seemingly best used
     * with interrupts.
     * 
     * @param ptr The VCNL4010 object.
     * @return A value indicating whether or not the transaction is successful. Should
     * only return either true or false.
     */
    bool bVCNL4010Periodic( VCNL4010* ptr );

    /** 
     * @brief Starts an on-demand measurement.
     * 
     * @param ptr The VCNL4010 object.
     * @return A value indicating whether or not the transaction is successful. Should
     * only return either true or false.
     */
	bool bVCNL4010Request( VCNL4010* ptr );

    /**
     * @brief Checks whether or not an on-demand reading has completed. 
     * 
     * @param ptr The VCNL4010 object.
     * @param prox_state The state of a proximity reading.
     * @return A value indicating whether or not the transaction is successful. Should
     * only return either true or false.
     */
	bool bVCNL4010Check( VCNL4010* ptr, bool* prox_state );

    /**
     * @brief Gets the proximity value after a successful reading.
     * 
     * @param ptr The VCNL4010 object.
     * @param prox_value The acquired proximity value. 
     * @return A value indicating whether or not the transaction is successful. Should
     * only return either true or false.
     */
	bool bVCNL4010GetProx( VCNL4010* ptr, uint16_t* prox_value );

    /**
     * @brief Configures the interrupt.
     * 
     * @param ptr The VCNL4010 object.
     * @param enable True enables the interrupt, whereas false disables the interrupt.
     * @return A value indicating whether or not the transaction is successful. Should
     * only return either true or false.
     */
	bool bVCNL4010IntSetup( VCNL4010* ptr, bool enable );

    /**
     * @brief Checks and resets the status of the interrupt.
     * 
     * @param ptr The VCNL4010 object.
     * @param status The status of the interrupt. 
     * @return A value indicating whether or not the transaction is successful. Should
     * only return either true or false.
     */
	bool bVCNL4010IntHandler( VCNL4010* ptr, bool* status  );

#ifdef __cplusplus
}
#endif

#endif
