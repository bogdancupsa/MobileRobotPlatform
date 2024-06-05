/*
 * DrivingAPIs.h
 *
 *  Created on: May 8, 2024
 *      Author: paul.contis
 */

#ifndef INC_DRIVINGAPIS_H_
#define INC_DRIVINGAPIS_H_

#include "HBridge.h"
#include "ServoSG90.h"
#include <stdint.h>

#define FORWARD 	1
#define BACKWARD	2

/*
#define TURN_RIGHT	1
#define TURN_LEFT	2
*/
#define HALF(Speed)		(Speed/2)

#define CALC_ANGLE 	5

#define ENABLE 		1
#define DISABLE		0

/*Breaks*/
#define BREAK		0
#define LEFTBREAK 	1
#define RIGHTBREAK 	2

/*Driving Speeds */

#define SPEED_100 		65535
#define SPEED_90 		58982
#define SPEED_80 		52428
#define SPEED_70 		45875
#define SPEED_60 		39321
#define SPEED_50 		32768

#define SPEED_40 		26214
#define SPEED_35 		22937
#define SPEED_30 		19661
#define SPEED_20 		13107
#define SPEED_10 		6554
#define SPEED_0 		0


/*Steering Speeds */


#define STEER_0 		0

/*
typedef struct
{
	uint8_t MoveForward;
	uint8_t MoveBackward;
	uint8_t TurnRight;
	uint8_t TurnLeft;

}MoveDir_t;
*/



/*************************************************************************************************************************************************
 *					                 									APIs
 *
 *************************************************************************************************************************************************/
void DrivingAPIs_LineMove(uint16_t Speed, uint8_t Direction);
void DrivingAPIs_TurnMove(uint16_t Dir, uint8_t Angle, uint16_t Speed);
void DrivingAPIs_Break(uint8_t BreakType);








#endif /* INC_DRIVINGAPIS_H_ */
