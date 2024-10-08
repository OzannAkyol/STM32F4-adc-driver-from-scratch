
#include "main.h"
#include "stm32f4xx_it.h"
#include "adc.h"


void NMI_Handler(void);

void HardFault_Handler(void);

void MemManage_Handler(void);

void BusFault_Handler(void);

void UsageFault_Handler(void);

void SVC_Handler(void);

void DebugMon_Handler(void);

void PendSV_Handler(void);

void SysTick_Handler(void);

void ADC_IRQHandler(void){
	adc_interrupt_irq();
}


