/*
 * ObstacleDetection.h
 *
 *  Created on: May 17, 2024
 *      Author: ovidiu.suciu
 */

#ifndef SRC_OBSTACLEDETECTION_H_
#define SRC_OBSTACLEDETECTION_H_



/* Exported types ------------------------------------------------------------*/
typedef struct
{
	boolean OBSTACLE_FR;
	boolean OBSTACLE_FL;
}ObstacleState_Typedef;

/* Exported constants --------------------------------------------------------*/


/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void OD_Main(void);

#endif /* SRC_OBSTACLEDETECTION_H_ */
