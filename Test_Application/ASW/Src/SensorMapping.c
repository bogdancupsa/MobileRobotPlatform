/*
 * SensorMapping.c
 *
 *  Created on: May 8, 2024
 *      Author: paul.contis
 */


#include "SensorMapping.h"

#include "main.h"
#include "HCSR04.h"

#define TRIG_READ 20

extern uint16_t Distance;




void SensorMapping_Read(SensorsPosition_t *SensorMapping)
{

	static uint8_t flag;

	HCSR04_Read();

	if(Distance <DETECTION_60cm)
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


