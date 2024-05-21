/*
 * motorControl.h
 *
 *  Created on: May 14, 2024
 *      Author: alexandru.coca
 */

#ifndef INC_MOTORCONTROL_H_
#define INC_MOTORCONTROL_H_

extern void startPWM(TIM_HandleTypeDef *htim);
extern void set_direction ( uint8_t direction);
extern void steer_right();
extern void steer_left();
extern void start();
extern void stop();
extern void setSpeed(uint16_t speed);
extern void setDutyCycle(TIM_HandleTypeDef *htim, uint32_t Channel, uint16_t dutyCycle);


#endif /* INC_MOTORCONTROL_H_ */
