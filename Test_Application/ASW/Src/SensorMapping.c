/*
 * SensorMapping.c
 *
 *  Created on: May 8, 2024
 *      Author: paul.contis
 */


#include "SensorMapping.h"

#include "main.h"
#include "HCSR04.h"



extern uint8_t Distance;




void SensorMapping_Read(SensorsPosition_t *SensorMapping)
{

	static uint8_t flag;

	HCSR04_Read();
	HAL_Delay(200);

	if(Distance <50)
	{
	SensorMapping->FronLeft.Detection= 1;
	SensorMapping->FronLeft.ActualDistance = Distance;

	flag = Distance;
	SensorMapping->FronLeft.PreviousDistance = flag;




	}
	else
	{
	SensorMapping->FronLeft.Detection= 0;
	SensorMapping->FronLeft.ActualDistance = 0;
	SensorMapping->FronLeft.PreviousDistance = 0;
	}
}


