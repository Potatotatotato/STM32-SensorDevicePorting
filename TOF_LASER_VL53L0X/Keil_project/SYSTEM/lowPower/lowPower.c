#include "lowPower.h"
#include "sys.h"
#include "LED.h"
#include "KEY.h"
#include "mpu6050.h"

//初始化WKUP引脚EXTI
void EXTI_WKUP_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,EXTI_PinSource0);  //PC0
	
	EXTI_InitStruct.EXTI_Line = EXTI_Line0;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
	
	EXTI_Init(&EXTI_InitStruct);
	
	
	NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 2;
	NVIC_Init(&NVIC_InitStruct);
}

void MPU_Motion_Detection_Init(u8 acceleration_LSB, u8 time_ms)
{
	MPU_WriteByte(MPU_MOTION_DET_REG,acceleration_LSB);	//运动检测加速度阈值
	MPU_WriteByte(MPU_MOTION_DUR_REG,time_ms);	//运动检测计时器阈值
	MPU_Set_Rate(50);						//设置采样率50Hz，低通滤波器带宽为50/2=25Hz
	MPU_WriteByte(MPU_ACCEL_CFG_REG,0x04);  //加速度计量程±2g，高通滤波设置
	MPU_WriteByte(MPU_INTBP_CFG_REG,0x82);	//INTERRUPT引脚配置，低电平有效，推挽输出
	MPU_WriteByte(MPU_INT_EN_REG,0x40);  		//中断使能
	MPU_WriteByte(MPU_PWR_MGMT1_REG,0X01);	//唤醒MPU6050,选择时钟源为gyroX时钟
	MPU_WriteByte(MPU_PWR_MGMT2_REG,0X00);	//加速度与陀螺仪都工作
}

void MPU_Enter_Cycle_Mode(void)
{
	MPU_WriteByte(MPU_PWR_MGMT1_REG,0x21);	//打开cycle模式，芯片时钟选择gyroX时钟
	MPU_WriteByte(MPU_PWR_MGMT2_REG,0xC0);		//设置cycle interval
}

void STM32_Enter_StandbyMode(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);
	PWR_BackupAccessCmd(ENABLE);
	
	RTC_ITConfig(RTC_IT_TS|RTC_IT_WUT|RTC_IT_ALRB|RTC_IT_ALRA|RTC_IT_TAMP,DISABLE);
	RTC_ClearITPendingBit(RTC_IT_TS|RTC_IT_WUT|RTC_IT_ALRB|RTC_IT_ALRA|RTC_IT_TAMP);
	
	PWR_WakeUpPinCmd(ENABLE);
	PWR_ClearFlag(PWR_FLAG_WU);
	PWR_EnterSTANDBYMode();
}

void EXTI0_IRQHandler(void)
{
	EXTI_ClearITPendingBit(EXTI_Line0);
	
}

