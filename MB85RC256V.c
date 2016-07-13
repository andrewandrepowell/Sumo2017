#include "MB85RC256V.h"

void vMB85RC256VSetup( MB85RC256V* ptr, bool a2, bool a1, bool a0,
        MB85RC256VI2CMethod write_ptr, MB85RC256VI2CMethod read_ptr, void* param )
{
    /* Determine the slave address. */
    ptr->slave_addr = MB85RC256V_BASE_SLAVE_ADDR |
            ( (a0) ? (1 << 0) : 0 ) |
            ( (a1) ? (1 << 1) : 0 ) |
            ( (a2) ? (1 << 2) : 0 ) ;
    
    /* Store pointers to the I2C methods. */
    ptr->write_ptr = write_ptr;
    ptr->read_ptr = read_ptr;
    ptr->param = param;
}

bool vMB85RC256VWrite( MB85RC256V* ptr, uint16_t addr, uint8_t* data, size_t len )
{
    uint8_t* payload;
    bool success;
    
    /* Configure payload. */
    payload = malloc( 2+len );
    if ( payload == NULL ) 
        return false;
    payload[0] = ( uint8_t )( ( addr & 0xff00 ) >> 8 );
    payload[1] = ( uint8_t )( addr & 0xff );
    memcpy( &payload[2], data, len );
    
    /* Perform write operation over I2C interface, free result, and return status. */
    success = ptr->write_ptr( ptr->slave_addr, payload, len+2, ptr->param );
    free( payload );
    if ( success )
    {
        return true;
    }
    else 
    {
        return false;
    }
}

bool vMB85RC256Read( MB85RC256V* ptr, uint16_t addr, uint8_t* data, size_t len )
{
    uint8_t payload[] = 
    { 
        ( uint8_t )( ( addr & 0xff00 ) >> 8 ), 
        ( uint8_t )( addr & 0xff )
    };
    uint8_t slave_addr = ptr->slave_addr;
    void* param = ptr->param;
    
    /* Specify the address to access. */
    if ( !ptr->write_ptr( slave_addr, payload, 2, param ) )
        return false;
    
    /* Perform read operation. */
    if ( !ptr->read_ptr( slave_addr, data, len, param ) )
        return false;
    
    return true;
}