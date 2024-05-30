/*
 * ServoSG90.c
 *
 *  Created on: May 8, 2024
 *      Author: paul.contis
 */


#include "ServoSG90.h"




void ServoSG90_SteeringWheelCtrl(uint16_t Direction)
{
	htim2.Instance->CCR1 = Direction;
}

