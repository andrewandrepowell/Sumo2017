#include "L293DNE.h"

static inline  __attribute__ ((always_inline))
void vL293DNEMotorStateSetup( L293DNEMotorState* ptr );

void vL293DNESetup( L293DNE* ptr, vL293DNSetPin setpin_ptr, vL293DNSetMotor setmotor_ptr, void* param )
{
    /* Initialize object. */
    ptr->setpin_ptr = setpin_ptr;
    ptr->setmotor_ptr = setmotor_ptr;
    ptr->param = param;
    vL293DNEMotorStateSetup( &ptr->left );
    vL293DNEMotorStateSetup( &ptr->right );
    vL293DNEUpdateMotors( ptr );
}

void vL293DNEUpdateMotors( L293DNE* ptr )
{
    /* Acquire needed data from object. */
    L293DNEMotorState* left = &ptr->left;
    L293DNEMotorState* right = &ptr->right;
    vL293DNSetMotor setmotor_ptr = ptr->setmotor_ptr;
    vL293DNSetPin setpin_ptr = ptr->setpin_ptr;
    void* param = ptr->param;
    bool leftforward = left->forward;
    bool rightforward = right->forward;
    
    /* Configure left motor. */
    setmotor_ptr( L293DNE_Left, left->speed, param );
    setpin_ptr( L293DNE_A1Left, leftforward, param );
    setpin_ptr( L293DNE_A2Left, !leftforward, param );
    
    /* Configure right motor. */
    setmotor_ptr( L293DNE_Right, right->speed, param );
    setpin_ptr( L293DNE_A1Right, rightforward, param );
    setpin_ptr( L293DNE_A2Right, !rightforward, param );
}

L293DNEMotorState* ptrL293DNEGetMotorState( L293DNE* ptr, L293DNEMotor motor )
{
    switch ( motor )
    {
    case L293DNE_Left:      return &ptr->left;
    case L293DNE_Right:     return &ptr->right;
    }
}

void vL293DNEMotorStateSetup( L293DNEMotorState* ptr )
{
    ptr->forward = true;
    ptr->speed = 0;
}