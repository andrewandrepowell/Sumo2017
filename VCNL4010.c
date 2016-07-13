#include "VCNL4010.h"


bool bVCNL4010Periodic( VCNL4010* ptr )
{
    unsigned char command_reg;

	if ( ptr->read_ptr( VCNL4010_SLAVE_ADDR, VCNL4010_REG_CMD_ADDR, &command_reg, ptr->param ) == false )
	{
		return false;
	}

	return ptr->write_ptr(
			VCNL4010_SLAVE_ADDR,
			VCNL4010_REG_CMD_ADDR,
			command_reg | (1<<VCNL4010_REG_CMD_PROX_EN) | (1<<VCNL4010_REG_CMD_SELF_TIMED_EN),
			ptr->param );
}

bool bVCNL4010Request( VCNL4010* ptr )
{
	unsigned char command_reg;

	if ( ptr->read_ptr( VCNL4010_SLAVE_ADDR, VCNL4010_REG_CMD_ADDR, &command_reg, ptr->param ) == false )
	{
		return false;
	}

	return ptr->write_ptr(
			VCNL4010_SLAVE_ADDR,
			VCNL4010_REG_CMD_ADDR,
			command_reg | (1<<VCNL4010_REG_CMD_PROX_OD),
			ptr->param );
}

bool bVCNL4010Check( VCNL4010* ptr, bool* prox_state )
{
	unsigned char command_reg;

	if ( ptr->read_ptr( VCNL4010_SLAVE_ADDR, VCNL4010_REG_CMD_ADDR, &command_reg, ptr->param ) == false )
	{
		return false;
	}

	*prox_state = ( command_reg & (1<<VCNL4010_REG_CMD_PROX_DATA_RDY) ) ?
			true : false;

	return true;
}

bool bVCNL4010GetProx( VCNL4010* ptr, uint16_t* prox_value )
{
	unsigned char data_byte_ms, data_byte_ls;

	if ( ( ptr->read_ptr( VCNL4010_SLAVE_ADDR, VCNL4010_REG_PROX_MS_ADDR, &data_byte_ms, ptr->param ) == false ) ||
			( ptr->read_ptr( VCNL4010_SLAVE_ADDR, VCNL4010_REG_PROX_LS_ADDR, &data_byte_ls, ptr->param ) == false ) ) {
		return false;
	}

	*prox_value = (uint16_t)data_byte_ls | ( (uint16_t)data_byte_ms << 8 );

	return true;
}

bool bVCNL4010IntSetup( VCNL4010* ptr, bool enable )
{
	unsigned char int_cntrl;

	if ( ptr->read_ptr( VCNL4010_SLAVE_ADDR, VCNL4010_REG_INT_CNTRL_ADDR, &int_cntrl, ptr->param ) == false )
	{
		return false;
	}

	if ( enable != false )
	{
		int_cntrl |= (1<<VCNL4010_REG_INT_CNTRL_PROX_RDY_EN);
	}
	else
	{
		int_cntrl &= ~(1<<VCNL4010_REG_INT_CNTRL_PROX_RDY_EN);
	}

	return ptr->write_ptr(
			VCNL4010_SLAVE_ADDR,
			VCNL4010_REG_INT_CNTRL_ADDR,
			int_cntrl,
			ptr->param );
}

bool bVCNL4010IntHandler( VCNL4010* ptr, bool* status )
{
	unsigned char int_status;

    *status = false;
    
	if ( ptr->read_ptr( VCNL4010_SLAVE_ADDR, VCNL4010_REG_INT_STAT_ADDR, &int_status, ptr->param ) == false )
	{
		return false;
	}

	if ( int_status & (1<<VCNL4010_REG_INT_STAT_PROX_RDY) ) {
        *status = true;
		return ptr->write_ptr(
				VCNL4010_SLAVE_ADDR,
				VCNL4010_REG_INT_STAT_ADDR,
				(1<<VCNL4010_REG_INT_STAT_PROX_RDY),
				ptr->param );
	}

	return true;
}
