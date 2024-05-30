/*
 * MotionController.h
 *
 *  Created on: May 8, 2024
 *      Author: paul.contis
 */

#ifndef INC_MOTIONCONTROLLER_H_
#define INC_MOTIONCONTROLLER_H_

#include "DrivingAPIs.h"
#include "SensorMapping.h"


#define CC    00// clear everywhere
#define BB	11// blocked everywhere
#define CB	01// left clear
#define BC	10//	right clear

void state_machine(void);
/*void read_map(void);
void write_updated_pos(void);*/



/*typedef struct
{
	SensorsState_t SensorsState;

}LastState_t;

typedef struct
{
	SensorsState_t SensorsState;

}CurrentState_t;*/



/*typedef struct
{
	uint8_t State1;
	uint8_t State2;
	uint8_t State3;
	uint8_t State4;
}State_t;*/




#endif /* INC_MOTIONCONTROLLER_H_ */
