#define MOVE_FWD 1
#define MOVE_BWD 0
#define STOP_MOTORS 3
extern void TIM1_PWM_Init(void);

extern void DMA_PWM_Init(void);

extern void Set_Speed_And_Direction_Motor_Right(uint8_t Speed, uint8_t Direction);
extern void Set_Speed_And_Direction_Motor_Left(uint8_t Speed, uint8_t Direction);
