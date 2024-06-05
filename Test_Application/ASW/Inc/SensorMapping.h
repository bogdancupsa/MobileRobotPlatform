/*
 * SensorMapping.h
 *
 *  Created on: May 8, 2024
 *      Author: paul.contis
 */

#ifndef INC_SENSORMAPPING_H_
#define INC_SENSORMAPPING_H_

#include "HCSR04.h"
#include <stdint.h>

#define DETECTION_10cm 		10
#define DETECTION_30cm 		30
#define DETECTION_60cm 		60



typedef struct
{
	uint8_t Detection;
	uint8_t PreviousDistance;
	uint8_t	ActualDistance;

}HCSR04_t;


typedef struct
{
	HCSR04_t FrontRight;
	HCSR04_t FronLeft;
	HCSR04_t BackRight;
	HCSR04_t BackLeft;
}SensorsPosition_t;

void SensorMapping_Read(SensorsPosition_t *SensorMapping);


#endif /* INC_SENSORMAPPING_H_ */
