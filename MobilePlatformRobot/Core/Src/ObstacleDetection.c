/*
 * ObstacleDetection.c
 *
 *  Created on: May 17, 2024
 *      Author: ovidiu.suciu
 */


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ObstacleDetection.h"
#include "Sensordatacapture.h"
#include "MotorsControlService.h"

ObstacleState_Typedef ObstacleState = {FALSE,FALSE};

void OD_Main(void)
{
	SenData_TypeDef SensorData;

	Get_SensorData_Mapping(&SensorData);

	if(SensorData[POS_FR].Value < 15)
	{
		ObstacleState.OBSTACLE_FR = TRUE;

	}
	if(SensorData[POS_FL].Value < 15)
	{
		ObstacleState.OBSTACLE_FL = TRUE;
	}

	if((ObstacleState.OBSTACLE_FR == TRUE) && (ObstacleState.OBSTACLE_FL == TRUE))
	{
		Mcs_SetState(MCS_BREAK);
	}
	if((ObstacleState.OBSTACLE_FR == FALSE) && (ObstacleState.OBSTACLE_FL == TRUE))
	{
		MCS_SetDirection(MCS_RiGHT);
	}
	if((ObstacleState.OBSTACLE_FR == TRUE) && (ObstacleState.OBSTACLE_FL == FALSE))
	{
		MCS_SetDirection(MCS_LEFT);
	}

}
