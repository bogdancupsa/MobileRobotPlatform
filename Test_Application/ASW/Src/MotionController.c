/*
 * MotionController.c
 *
 *  Created on: May 8, 2024
 *      Author: paul.contis
 */


#include "MotionController.h"


void state_machine(void)
{

	static SensorsPosition_t ActSensPos={0};

	SensorMapping_Read(&ActSensPos);


static uint8_t flag;
static uint8_t flag2;
static uint8_t flag3;

if((ActSensPos.FronLeft.Detection)||(flag3==1))
{



	if((ActSensPos.FronLeft.ActualDistance <DETECTION_10cm)||(flag3==1))
	{
		flag3=1;
		if(flag2 < 20)
		{
		DrivingAPIs_Break(BREAK);
		flag2++;
		}
		else if(flag2 >= 20)
		{
		DrivingAPIs_LineMove(SPEED_50,BACKWARD);
		DrivingAPIs_TurnMove(TURN_RIGHT, 0, SPEED_0);
		flag2++;
		}
		if(flag2 == 50)
		{
			flag2 =0;
			flag3 =0;

		}


	}
	else if (flag <= 150)
	{

		DrivingAPIs_TurnMove(TURN_RIGHT, 0, SPEED_35);
		flag++;


	}
	else if (flag <= 300)
	{

		DrivingAPIs_TurnMove(TURN_LEFT, 0, SPEED_35);
		flag++;
	}


}
else
{
	if(flag3 !=1)
	{
	DrivingAPIs_LineMove(SPEED_20,FORWARD);
	}
	//flag++;

}


if (flag >= 301)
	{
		flag =0;
	}



}


