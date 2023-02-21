#ifndef __WAKEUP_H__
#define __WAKEUP_H__

void EXTI_WKUP_Init(void);
void STM32_Enter_StandbyMode(void);
void MPU_Motion_Detection_Init(u8 acceleration_LSB, u8 time_ms)
void MPU_Enter_Cycle_Mode(void);

#endif

