/*
 * MotorsControlService.h
 *
 *  Created on: May 17, 2024
 *      Author: ovidiu.suciu
 */

#ifndef SRC_MOTORSCONTROLSERVICE_H_
#define SRC_MOTORSCONTROLSERVICE_H_


/* Exported types ------------------------------------------------------------*/
typedef enum{

	MCS_LEFT,
	MCS_RiGHT,
	MCS_NOTURN
}MCS_Direction_TypeDef;

typedef enum{
	MCS_FORWARD,
	MCS_BACKWARD,
	MCS_BREAK,
	MCS_LOCKED,
	MCS_ERROR
}MCS_State_TypeDef;

/* Exported constants --------------------------------------------------------*/


/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
HAL_StatusTypeDef MCS_Init(void);
void MCS_MainFunction(void);

#endif /* SRC_MOTORSCONTROLSERVICE_H_ */
