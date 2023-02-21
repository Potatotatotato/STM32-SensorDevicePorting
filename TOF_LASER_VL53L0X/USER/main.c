#include "stm32f4xx.h"
#include "usart.h"
#include "delay.h"
#include "key.h"
#include "lcd.h"
#include "led.h"
#include "VL53L0X.h"

#define LONG_RANGE 0
#define HIGH_SPEED_LOW_ACCURACY 0

void VL53L0X_single_measurement_test_init(void)
{
	VL530LX_init(1, 500);
	
	#if LONG_RANGE == 1
		// lower the return signal rate limit (default is 0.25 MCPS)
		VL53L0X_SetSignalRateLimit(0.1);
		// increase laser pulse periods (defaults are 14 and 10 PCLKs)
		VL53L0X_SetVcselPulsePeriod(VcselPeriodPreRange, 18);
		VL53L0X_SetVcselPulsePeriod(VcselPeriodFinalRange, 14);
	#endif

	#if HIGH_SPEED_LOW_ACCURACY ==1
		// reduce timing budget to 20 ms (default is about 33 ms)
		VL53L0X_SetMeasurementTimingBudget(20000);
	#elif HIGH_SPEED_LOW_ACCURACY == 0
		// increase timing budget to 200 ms
		VL53L0X_SetMeasurementTimingBudget(200000);
	#endif
}

void VL53L0X_continus_measurement_test_init(void)
{
	VL530LX_init(1, 500);
	// Start continuous back-to-back mode (take readings asfast as possible).
	VL53L0X_StartContinuous(0);
}

int main(void)
{

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	delay_init(168);
	uart_init(115200);
	LED_Init();
	KEY_Init();
 	//LCD_Init();
	VL53L0X_single_measurement_test_init();
	VL53L0X_continus_measurement_test_init();
	
	while(1)
	{
		printf("%d\r\n", VL53L0X_ReadSingleMeasurement());
		//printf("%d\r\n", VL53L0X_ContinusMeasurement());
		if(VL53L0X_TimeoutOccurred())
		{
			printf("TIMEOUT");
		}
		delay_ms(200);
	}	
	
}
