#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_tim.h"

#define FORWARD    0
#define BACKWARDS  1

#define TURN_LEFT  0
#define TURN_RIGHT 1
#define GO         2
#define STOP       3

#define STOP_LIMIT 25

/*-------------------------------------------------------------------------------------------------
Function: pwm_start



Description:
    This function is a wrapper for HAL_TIM_PWM_Start function.


Parameters In:
    htim:  TIM_HandleTypeDef*  --  timer used for PWM
    Channel: uint32_t          --  Channel name

Parameters Out:
    None

Return Value:
    None
-------------------------------------------------------------------------------------------------*/

void pwm_start (TIM_HandleTypeDef *htim, uint32_t Channel);



/*-------------------------------------------------------------------------------------------------
Function: pwm_stop



Description:
    This function is a wrapper for HAL_TIM_PWM_Stop function.


Parameters In:
    htim:  TIM_HandleTypeDef*  --  timer used for PWM
    Channel: uint32_t          --  Channel name

Parameters Out:
    None

Return Value:
    None
-------------------------------------------------------------------------------------------------*/


void pwm_stop  (TIM_HandleTypeDef *htim, uint32_t Channel);


/*-------------------------------------------------------------------------------------------------
Function: set_direction



Description:
    This function sets direction for a DC motor. Direction can be FORWARD and BACKWARDS.


Parameters In:
    GPIO_1: GPIO_TypeDef*   -- Port for IN1 signal
    GPIO_Pin_1: uint16_t    -- Pin for IN1 signal
    GPIO_2: GPIO_TypeDef*   -- Port for IN2 signal
    GPIO_Pin_2: uint16_t    -- Pin for IN2 signal
    direction: uint8_t      -- direction value (FORWARD, BACKWARDS)

Parameters Out:
    None

Return Value:
    None
-------------------------------------------------------------------------------------------------*/


void set_direction (GPIO_TypeDef* GPIO_1,
		            uint16_t GPIO_Pin_1,
					GPIO_TypeDef* GPIO_2,
					uint16_t GPIO_Pin_2,
					uint8_t direction);


/*-------------------------------------------------------------------------------------------------
Function: set_turn_values



Description:
    This function sets direction and PWM value for the DC motor by the value of turn_direction input parameter.


Parameters In:
    turn_direction: uint8_t   -- turn_direction (GO, STOP, TURN_LEFT, TURN_RIGHT)

Parameters Out:
    right_motor_value: uint16_t*     -- PWM for the DC motors on the right side
    left_motor_value: uint16_t*      -- PWM for the DC motors on the left side
    direction_left_front: uint8_t*   -- direction for front left DC motor
    direction_right_front: uint8_t*  -- direction for front right DC motor
    direction_left_rear: uint8_t*    -- direction for rear left DC motor
    direction_right_rear: uint8_t*   -- direction for rear right DC motor

Return Value:
    None
-------------------------------------------------------------------------------------------------*/


void set_turn_values (uint8_t turn_direction,
		              uint16_t* right_motor_value,
		              uint16_t* left_motor_value,
					  uint8_t* direction_left_front,
					  uint8_t* direction_right_front,
					  uint8_t* direction_left_rear,
					  uint8_t* direction_right_rear);


#endif // MOTOR_CONTROL_H
