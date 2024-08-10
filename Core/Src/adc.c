/*
 * adc.c
 *
 *  Created on: Jul 26, 2024
 *      Author: ozana
 */

#include "adc.h"

uint16_t sData;
uint16_t gDataArr[MAX_SIZE_OF_CONVERTED_DATA];
uint16_t *ptr = gDataArr;
uint16_t gDmaArr[MAX_CHANNEL_NUM_OF_ADC];

/*
 * Date............: Jul 28, 2024 - 16:32
 * Function........: rcc_clock_enable
 * .................
 */

void rcc_clock_enable(RCC_TypeDef* rcc_p, uint8_t gpiox, uint8_t adcx, uint8_t dmax){
	rcc_p->AHB1ENR |= (1U << gpiox);
	rcc_p->APB2ENR |= (1U << adcx);
	rcc_p->AHB1ENR |= (1U << dmax);
}

/*
 * Date............: Jul 28, 2024 - 16:44
 * Function........: gpio_config
 * .................
 */

void gpio_config(GPIO_TypeDef* gpio, uint8_t pin_number, uint8_t mode){
	gpio->MODER &= ~(CLEAR_BIT_POS << pin_number);
	gpio->MODER |= (mode << pin_number);

}

/*
 * Date............: Jul 31, 2024 - 19:33
 * Function........: adc_open
 * .................
 */

void adc_open(ADC_TypeDef* adcx){
	/*Disable the ADC */
	adcx->CR2 |= CR2_ADON;
}

/*
 * Date............: Jul 28, 2024 - 16:46
 * Function........: adc_close
 * .................
 */

void adc_close(ADC_TypeDef* adcx){
	/*Disable the ADC */
	adcx->CR2 &= ~CR2_ADON;
	/*update the states of ADC (like : adc_idle)*/
}

/*
 * Date............: Jul 28, 2024 - 16:46
 * Function........: adc_config
 * .................
 */

void adc_config(ADC_TypeDef* adcx, ConversionType conv_mode, uint16_t total_num_of_channel){

	/*Enable ADC */
	adcx->CR2 |= CR2_ADON;

	/*Set ADC as a 12-bit resolution.*/
	adcx->CR1 &= ~(CR1_12_BIT_RESOLUTION);

	/*Set Data as Right alignment*/
	adcx->CR2 &= ~(CR2_RIGHT_ALIGN);

	/*Set Data sequence length*/
	if(conv_mode == SINGLE_CONVERSION_MODE)
	{
		/*Single conversion mode*/
		adcx ->CR2 &= ~(CR2_CONT);

		/*Set number of conversion*/
		adcx ->SQR1 &= ~(total_num_of_channel << 20);

	}
	else if(conv_mode == CONTINUOUS_CONVERSION_MODE)
	{
		/*Activate the continuous conversion*/
		adcx->CR2 &= ~(CR2_CONT);

		/*Continuous conversion mode*/
		adcx ->CR2 |= (CR2_CONT);

		/*Set number of conversion*/
		adcx ->SQR1 |= (total_num_of_channel << 20);
	}
	else if(conv_mode == SCAN_SINGLE_CONVERSION_MODE)
	{
		/*Set scan mode*/
		adcx->CR1|= ADC_CR1_SCAN_BIT;

		/*Set number of conversion*/
		adcx ->SQR1 |= (total_num_of_channel << 20);

		/*Activate the single conversion*/
		adcx->CR2 &= ~(CR2_CONT);

		/*The EOC bit is set at the end of each regular conversion.*/
		adcx->CR2 |= ADC_CR2_EOCS;

		/*use DMA to store converted value*/
		adcx->CR2 &= ~(ADC_CR2_DMA_EN);
		adcx->CR2 |= (ADC_CR2_DMA_EN);

		/*DDS: DMA disable selection (for single ADC mode)*/
		adcx->CR2 &= ~(ADC_CR2_DDS);

	}
	else if(conv_mode == SCAN_CONTINUOUS_CONVERSION_MODE)
	{
		/*Set scan mode*/
		adcx->CR1|= ADC_CR1_SCAN_BIT;

		/*Set number of conversion*/
		adcx ->SQR1 |= (total_num_of_channel << 20);

		/*Activate the continuous conversion*/
		adcx->CR2 &= ~(CR2_CONT);
		adcx->CR2 |= (CR2_CONT);

		/*The EOC bit is set at the end of each regular conversion.*/
		adcx->CR2 |= ADC_CR2_EOCS;

		/*use DMA to store converted value*/
		adcx->CR2 &= ~(ADC_CR2_DMA_EN);
		adcx->CR2 |= (ADC_CR2_DMA_EN);

		/*DDS: DMA disable selection (for single ADC mode)*/
		adcx->CR2 |= (ADC_CR2_DDS);

	}
}

/*
 * Date............: Jul 30, 2024 - 20:14
 * Function........: adc_conversion_polling
 * .................
 */

uint32_t adc_conversion_polling(ADC_TypeDef* adcx, uint32_t channel_number, uint32_t sdata){

	/*Trigger the conversion.*/
	adcx->CR2 |= (ADC_CR2_SWSTART);

	/*Wait  Regular channel end of conversion*/
	while(!(adcx->SR & ADC_SR_EOC_BIT))
					;
	/*get_adc_value*/
	sdata = get_adc_value(adcx);

	/*Clear the EOC bit*/
	adcx->SR &= ~ADC_SR_EOC_BIT;

	return sdata;
}

/*
 * Date............: Jul 30, 2024 - 20:14
 * Function........: adc_continuous_conversion_polling
 * .................
 */

void adc_continuous_conversion_polling(ADC_TypeDef* adcx, uint32_t* data_ptr, uint32_t num_of_conversion){


	/*The EOC bit is set at the end of each regular conversion.*/
	adcx->CR2 |= ADC_CR2_EOCS;

	for(int i = 0; i < num_of_conversion; ++i)
	{
		/*Trigger the conversion.*/
		adcx->CR2 |= (ADC_CR2_SWSTART);

		/*Wait  Each channel end of conversion*/
		while(!(adcx->SR & ADC_SR_EOC_BIT))
					;

		/*Store the data*/
		*(data_ptr + i) = get_adc_value(adcx);

		/*clear EOC bit*/
		ADC_CLEAR_BIT(adcx, SR, ADC_SR_EOC_BIT);


		if(adcx ->SR & ADC_SR_OVR_BIT)
		{
			/*Clear the OVR bit*/
			ADC_CLEAR_BIT(adcx, SR, ADC_SR_OVR_BIT);

			/*Trigger the conversion.*/
			adcx->CR2 |= (ADC_CR2_SWSTART);
		}
	}
}

/*
 * Date............: Jul 28, 2024 - 17:29
 * Function........: adc_interrupt_open
 * ................. call core4's interrupt function
 */

void adc_interrupt_open(IRQn_Type irq_num, uint32_t priority){
	/*Set interrupt priority*/
	NVIC_SetPriority(irq_num, priority);
	/*Enable global interrupt*/
	NVIC_EnableIRQ(irq_num);
}

/*
 * Date............: Jul 28, 2024 - 17:21
 * Function........: adc_interrupt_enable
 * .................
 */
void adc_interrupt_enable(ADC_TypeDef* adcx){
	/*Interrupt enable for EOC*/
	adcx->CR1 |= ADC_EOCIE_BIT;

	/* Analog watchdog interrupt enable*/
	adcx->CR1 |= ADC_AWDIE_BIT;

	/*Interrupt enable for injected channels*/
	adcx->CR1 |= ADC_JEOCIE_BIT;

	/*Set Overrun interrupt enable*/
	adcx->CR1 |= ADC_OVRIE_BIT;

	ADC_trigger(ADC1);
}


/*
 * Date............: Jul 28, 2024 - 17:56
 * Function........: adc_interrupt_irq
 * .................
 */

void adc_interrupt_irq(void){
	if(ADC_IS_FLAG_SET(ADC1, SR, ADC_SR_EOC_BIT))
	{
		if(ptr >= gDataArr + MAX_SIZE_OF_CONVERTED_DATA)
			ptr = gDataArr;

		*(ptr++) = ADC1->DR;
		/*Clear the bit*/
		ADC_CLEAR_BIT(ADC1, SR, ADC_SR_EOC_BIT);

	}
	else if(ADC_IS_FLAG_SET(ADC1, SR, ADC_SR_JEOC_BIT))
	{
		/*Clear the bit*/
		ADC_CLEAR_BIT(ADC1, SR, ADC_SR_JEOC_BIT);

		/*Interrupt enable for injected channels*/
	}
	else if(ADC_IS_FLAG_SET(ADC1, SR, ADC_SR_AWD_BIT))
	{
		/*Clear the bit*/
		ADC_CLEAR_BIT(ADC1, SR, ADC_SR_AWD_BIT);

		/* Analog watchdog interrupt enable*/

	}
	else if(ADC_IS_FLAG_SET(ADC1, SR, ADC_SR_OVR_BIT))
	{
		/*close ADC*/
		adc_close(ADC1);

		/*open ADC*/
		adc_open(ADC1);

		/*close DMA2, reconfig and open DMA*/
		DMA_config(DMA2_Stream0);

		/*Clear the bit*/
		ADC_CLEAR_BIT(ADC1, SR, ADC_SR_OVR_BIT);

		/*Trigger the conversion.*/
		ADC1->CR2 |= (ADC_CR2_SWSTART);
	}
}

/*
 * Date............: Jul 30, 2024 -
 * Function........: adc_set_channel
 * .................
 *
 */

ADCStatusType adc_set_channel(ADC_TypeDef* adcx, uint32_t channel, uint32_t rank,
								uint8_t clock_cycles )
{

	/*Check invalid rank*/
	if(rank <= 0 && rank > 17)
		return ADC_ERROR;

	/*Check invalid channel*/
	if(channel < 0 && channel > 18)
		return ADC_ERROR;

	/*Set adc sampling time*/
	if(channel < 10)
	{
		/*Clear the bit pos*/
		adcx->SMPR2 &= ~(CLEAR_BIT_POS_3 << ((channel) * 3));
		/*Set the clock cycles*/
		adcx->SMPR2 |= (clock_cycles << ((channel) * 3));
	}
	else
	{
		/*Clear the bit pos*/
		adcx->SMPR1 &= ~(CLEAR_BIT_POS_3 << ((channel - 10) * 3));

		/*Set the clock cycles*/
		adcx->SMPR1 |= (clock_cycles << ((channel - 10) * 3));
	}

	/*rank = [1,6]*/
	if(rank < 7)
	{
		adcx->SQR3 |= channel << ((rank - 1) * 5);
	}
	/*rank = [7,12]*/
	else if(6 < rank && rank < 13)
	{
		adcx->SQR2 |= channel << ((rank - 7) * 5);
	}
	/*rank = [13,16]*/
	else
	{
		adcx->SQR1 |= channel << ((rank - 13) * 5);
	}

	return ADC_OK;
}

/*
 * Date............: Jul 28, 2024 - 20:49
 * Function........: get_adc_value
 * .................
 */

uint32_t get_adc_value(ADC_TypeDef* adcx){
	return adcx->DR;
}

/*
 * Date............: Jul 29, 2024 - 21:49
 * Function........: test_adc_internal_sensor_function
 * .................
 */

void test_adc_internal_sensor_function(ADC_TypeDef * adcx, ADC_Common_TypeDef * adc_common, ADCTestTypeDef param){
	uint8_t error_check;

	if(param == ADC_TEMP)
	{
		/*Set the TSVREFE bit for internal temperature sensor.*/
		adc_common->CCR |= ADC_TSVREFE_BIT;

		error_check = adc_set_channel(adcx, ADC_IN16_CHANNEL, SEQUENCE_Rank_1, ADC_CLOCK_CYCLES_3);

		if(error_check != ADC_OK)
			return;
	}
	else if(param == ADC_VBAT)
	{
		/*Reset the TSVREFE bit for internal temperature sensor.*/
		adc_common->CCR &= ~ADC_TSVREFE_BIT;

		/*To enable the battery channel*/
		adc_common->CCR |= ADC_VBATE_BIT;

		error_check = adc_set_channel(adcx, ADC_IN18_CHANNEL, SEQUENCE_Rank_1, ADC_CLOCK_CYCLES_3);

		if(error_check != ADC_OK)
			return;
	}
}

/*
 * Date............: Aug 01, 2024 - 02:49
 * Function........: DMA_open_stream
 * .................
 */

DMAStatusType DMA_open_stream(DMA_Stream_TypeDef* dma){
	/*Open the stream*/
	dma->CR |= DMA2_EN;

	return DMA_OK;
}

/*
 * Date............: Aug 01, 2024 - 02:51
 * Function........: DMA_close_stream
 * .................
 */

DMAStatusType DMA_close_stream(DMA_Stream_TypeDef* dma){
	/*Disable the stream*/
	dma->CR &= ~(DMA2_EN);

	/*Wait for the stream is disabled*/
	while((dma->CR & DMA2_EN))
			;
	return DMA_OK;
}

/*
 * Date............: Aug 01, 2024 - 03:48
 * Function........: DMA_config
 * .................
 */

void DMA_config(DMA_Stream_TypeDef* dma){
	uint8_t states;
	states = DMA_close_stream(dma);
	if(states != DMA_OK)
		return;

	/*Set peripheral port as a source address*/
	dma->PAR = (uint32_t)&(ADC1->DR);

	/*Set Memory port as a destination address*/
	dma->M0AR = (uint32_t)gDmaArr;

	/*Configure total number of data*/
	dma->NDTR = MAX_NUM_OF_CONVERTED_DATA;

	/*Select DMA req channel for ADC1 (channel 0)*/
	dma->CR &= ~(DMA2_CHSEL);

	/*Clear Stream priority*/
	dma->CR &= ~(STREAM_PRIORITY_MASK);

	/*Set Stream priority*/
	dma->CR |= (STREAM_HIGH_PRIORITY);

	/*Define the data transfer direction. Peripheral to memory*/
	dma->CR &= ~(DATA_TRANSFER_DIR);

	/*Peripheral fixed mode*/
	dma->CR &= ~(DMA_PINC);

	/*Memory increment mode*/
	dma->CR &= ~(DMA_MINC);
	dma->CR |=  (DMA_MINC);

	/*Single transfer.*/
	dma->CR &= ~(DMA_MBURST);
	dma->CR &= ~(DMA_PBURST);

	/*Peripheral Data widths*/
	dma->CR |= (DMA_PSIZE_16BIT);

	/*Memory Data widths*/
	dma->CR |= (DMA_MSIZE_16BIT);

	/*Configure circular Buffer*/
	dma->CR |= (DMA_CIRC);

	/*open dma*/
	DMA_open_stream(dma);

}


/*
 * Date............: Aug 03, 2024 - 12:53
 * Function........: ADC_trigger
 * .................
 */

void ADC_trigger(ADC_TypeDef * adcx){

	adcx->CR2 &= ~(ADC_CR2_SWSTART);
	adcx->CR2 |= (ADC_CR2_SWSTART);

	/*according to data sheet wait 15 clock cycles to conversion */
	for (volatile uint32_t i = 0; i < 15; i++);
}


