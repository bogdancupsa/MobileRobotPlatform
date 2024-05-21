/*
 * TimersConf.h
 *
 *  Created on: May 17, 2024
 *      Author: ovidiu.suciu
 */

#ifndef SRC_TIMERSCONF_H_
#define SRC_TIMERSCONF_H_


/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/


/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
HAL_StatusTypeDef Timers_Configuration(void);
void delay_us (uint16_t us);

#endif /* SRC_TIMERSCONF_H_ */
