/*
 * Sensordatacapture.h
 *
 *  Created on: May 17, 2024
 *      Author: ovidiu.suciu
 */

#ifndef SRC_SENSORDATACAPTURE_H_
#define SRC_SENSORDATACAPTURE_H_



/* Exported types ------------------------------------------------------------*/

typedef enum{
	POS_FR, /*Front Right*/
	POS_FL, /*Front Left*/
	POS_BR, /*Back Right*/
	POS_BL, /*Back Left*/
}SenPos_TypeDef;

typedef struct
{
	SenPos_TypeDef orientation;	/* the position of the sensor*/
	uint16_t Value;         /*Distance Value captured */

} SenData_TypeDef;


/* Exported constants --------------------------------------------------------*/


/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

void HCSR04_Read (void);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef Get_SensorData_Mapping(SenData_TypeDef *mapping);

#endif /* SRC_SENSORDATACAPTURE_H_ */
