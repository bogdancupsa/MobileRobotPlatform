/*
 * MotionController.c
 *
 *  Created on: May 8, 2024
 *      Author: paul.contis
 */


#include "MotionController.h"



uint8_t Encoding(SensorsPosition_t data);

void state_machine(void)
{

	static SensorsPosition_t ActSensPos={0};

	//	uint8_t lu8_LastSensPos =0;
	//	uint8_t lu8_ActSensPos =0;

		//static uint8_t State;

		//DrivingAPIs_Break(BREAK);


		SensorMapping_Read(&ActSensPos);

		//lu8_ActSensPos = Encoding(ActSensPos);


	//	if(lu8_LastSensPos != lu8_ActSensPos)
	//	{
			//TODO : change state based on sensor readings

			/*State = lu8_ActSensPos;


			switch(State)
				{
					case CB:

						DrivingAPIs_TurnMove(TURN_BACK, 0, SPEED_80);
						DrivingAPIs_LineMove(SPEED_80,FORWARD);
						State=BC;
					break;

					case BC:

						DrivingAPIs_TurnMove(TURN_RIGHT, 0, SPEED_80);
						DrivingAPIs_LineMove(SPEED_80,FORWARD);
					break;

					default:
					{
						DrivingAPIs_TurnMove(TURN_LEFT, 0, SPEED_100);
						//DrivingAPIs_LineMove(SPEED_80,FORWARD);
						//DrivingAPIs_TurnMove(TURN_RIGHT, 0, SPEED_100);
						State=CB;
					}

				}*/
		//DrivingAPIs_TurnMove(TURN_LEFT, 0, SPEED_30);
		//HAL_Delay(1000);

static uint8_t flag;
static uint8_t flag2;
static uint8_t flag3;

if(ActSensPos.FronLeft.Detection)
{

	if(ActSensPos.FronLeft.ActualDistance <10)
	{
		flag3=1;
	}

	if(flag3==1)
	{
		if(flag2 < 20)
		{
		DrivingAPIs_Break(BREAK);
		flag2++;
		}
		else if(flag2 >= 20)
		{

			//if(flag==20)
			//{
				//DrivingAPIs_TurnMove(TURN_RIGHT, 0, SPEED_0);
			//}
			/*else if(flag == 40)
			{
				DrivingAPIs_TurnMove(TURN_BACK, 0, SPEED_0);
			}*/
		DrivingAPIs_LineMove(SPEED_50,BACKWARD);
		//DrivingAPIs_TurnMove(TURN_RIGHT, 0, SPEED_0);

		flag2++;

		if(flag2 == 160)
		{
			flag2 =0;

		}

		}
		flag3 =0;
	}
	else if (flag <= 150)
	{

		DrivingAPIs_TurnMove(TURN_RIGHT, 0, SPEED_20);
		flag++;

	}
	else if (flag <= 300)
	{

		DrivingAPIs_TurnMove(TURN_LEFT, 0, SPEED_20);
		flag++;
	}


}
else
{
	DrivingAPIs_LineMove(SPEED_20,FORWARD);
	flag++;

}


if (flag >= 301)
	{
		flag =0;
	}



}


uint8_t Encoding(SensorsPosition_t data)
{

	uint8_t lu8_retVal = 0u;

	lu8_retVal |=data.FronLeft.Detection;
	lu8_retVal <<=1u;
	lu8_retVal |=data.FrontRight.Detection;
	lu8_retVal <<=1u;
	//lu8_retVal |=data.BackLeft.Detection;
	//lu8_retVal <<=1u;
	//lu8_retVal |=data.BackRight.Detection;

	return lu8_retVal;
}

/*void read_map(void)
{
	;
}




void write_updated_pos(void)
{
	;
}*/
