#include "TCA9548A.h"

void vTCA9548ASetup( TCA9548A* ptr, bool a2, bool a1, bool a0 , 
        bTCA9548AI2CMethod write_ptr, void* param )
{
    /* Determine the slave address. */
    ptr->slave_addr = TCA9548A_BASE_SLAVE_ADDR |
            ( (a0) ? (1 << 0) : 0 ) |
            ( (a1) ? (1 << 1) : 0 ) |
            ( (a2) ? (1 << 2) : 0 ) ;
    
    /* Store pointers to the I2C methods. */
    ptr->write_ptr = write_ptr;
    ptr->param = param;    
}