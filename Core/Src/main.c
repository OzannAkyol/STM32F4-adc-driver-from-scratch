/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * @author			: ozan akyol
  * @date			: 26.07.2024 23:26-Fri
  *
  ******************************************************************************
*/

#include "main.h"
#include "adc.h"

uint16_t gdata = 0;
uint16_t gADCdata= 0;
float gtest_temp;
float Vstep;
float step_size = (3.3/4096);
float Vstep1;
float Vstep2;
float Vstep3;


uint16_t channel_in1_sensor_value=0;
uint16_t channel_in2_sensor_value=0;
uint16_t channel_in3_sensor_value=0;


int main(void)
{
	/*
	 	 TO TEST SCAN SINGLE CONVERSION ---> DMA
		 *
		 * CHANNEL_IN_1	 <-----> PA1
		 * CHANNEL_IN_2  <-----> PA2
		 * CHANNEL_IN_3 <-----> PA3
		 * ADC1_REQ		 <-----> DMA2 (Stream 0, Channel 0)
		 * Order of Channel(To convert) R1 > R2 > R3
		 */


	rcc_clock_enable(RCC, GPIOAEN, ADC1EN, DMA2EN);

	gpio_config(GPIOA, GPIO_PIN_1, ANALOG_MODE);
	gpio_config(GPIOA, GPIO_PIN_2, ANALOG_MODE);
	gpio_config(GPIOA, GPIO_PIN_3, ANALOG_MODE);

	adc_config(ADC1, SCAN_SINGLE_CONVERSION_MODE, SEQUENCE_LENGTH_3);

	adc_set_channel(ADC1, ADC_IN1_CHANNEL, SEQUENCE_Rank_1, ADC_CLOCK_CYCLES_3);
	adc_set_channel(ADC1, ADC_IN2_CHANNEL, SEQUENCE_Rank_2, ADC_CLOCK_CYCLES_3);
	adc_set_channel(ADC1, ADC_IN3_CHANNEL, SEQUENCE_Rank_3, ADC_CLOCK_CYCLES_3);

	DMA_config(DMA2_Stream0);
	ADC_trigger(ADC1);


	channel_in1_sensor_value = gDmaArr[0];
	channel_in2_sensor_value = gDmaArr[1];
	channel_in3_sensor_value = gDmaArr[2];


	Vstep1 = step_size * channel_in1_sensor_value;
	Vstep2 = step_size * channel_in2_sensor_value;
	Vstep3 = step_size * channel_in3_sensor_value;

  while (1)
  {

  }


}

/*
 	 TO TEST SCAN SINGLE CONVERSION ---> DMA
	 *
	 * CHANNEL_IN_1	 <-----> PA1
	 * CHANNEL_IN_2  <-----> PA2
	 * CHANNEL_IN_3 <-----> PA3
	 * ADC1_REQ		 <-----> DMA2 (Stream 0, Channel 0)
	 * Order of Channel(To convert) R1 > R2 > R3
	 */
/*

	rcc_clock_enable(RCC, GPIOAEN, ADC1EN, DMA2EN);

	gpio_config(GPIOA, GPIO_PIN_1, ANALOG_MODE);
	gpio_config(GPIOA, GPIO_PIN_2, ANALOG_MODE);
	gpio_config(GPIOA, GPIO_PIN_3, ANALOG_MODE);

	adc_config(ADC1, SCAN_SINGLE_CONVERSION_MODE, SEQUENCE_LENGTH_3);

	adc_set_channel(ADC1, ADC_IN1_CHANNEL, SEQUENCE_Rank_1, ADC_CLOCK_CYCLES_3);
	adc_set_channel(ADC1, ADC_IN2_CHANNEL, SEQUENCE_Rank_2, ADC_CLOCK_CYCLES_3);
	adc_set_channel(ADC1, ADC_IN3_CHANNEL, SEQUENCE_Rank_3, ADC_CLOCK_CYCLES_3);

	DMA_config(DMA2_Stream0);
	ADC_trigger(ADC1);


	channel_in1_sensor_value = gDmaArr[0];
	channel_in2_sensor_value = gDmaArr[1];
	channel_in3_sensor_value = gDmaArr[2];


	Vstep1 = step_size * channel_in1_sensor_value;
	Vstep2 = step_size * channel_in2_sensor_value;
	Vstep3 = step_size * channel_in3_sensor_value;



*/


/*
 	 TO TEST SCAN CONTINUOUS CONVERSION ---> DMA
	 *
	 * CHANNEL_IN_1	 <-----> PA1
	 * CHANNEL_IN_2  <-----> PA2
	 * CHANNEL_IN_3 <-----> PA3
	 * ADC1_REQ		 <-----> DMA2 (Stream 0, Channel 0)
	 * Order of Channel(To convert) R1 > R2 > R3
	 */
/*

	rcc_clock_enable(RCC, GPIOAEN, ADC1EN, DMA2EN);

	gpio_config(GPIOA, GPIO_PIN_1, ANALOG_MODE);
	gpio_config(GPIOA, GPIO_PIN_2, ANALOG_MODE);
	gpio_config(GPIOA, GPIO_PIN_3, ANALOG_MODE);

	adc_config(ADC1, SCAN_CONTINUOUS_CONVERSION_MODE, SEQUENCE_LENGTH_3);

	adc_set_channel(ADC1, ADC_IN1_CHANNEL, SEQUENCE_Rank_1, ADC_CLOCK_CYCLES_3);
	adc_set_channel(ADC1, ADC_IN2_CHANNEL, SEQUENCE_Rank_2, ADC_CLOCK_CYCLES_3);
	adc_set_channel(ADC1, ADC_IN3_CHANNEL, SEQUENCE_Rank_3, ADC_CLOCK_CYCLES_3);

	DMA_config(DMA2_Stream0);
	ADC_trigger(ADC1);


	channel_in1_sensor_value = gDmaArr[0];
	channel_in2_sensor_value = gDmaArr[1];
	channel_in3_sensor_value = gDmaArr[2];


	Vstep1 = step_size * channel_in1_sensor_value;
	Vstep2 = step_size * channel_in2_sensor_value;
	Vstep3 = step_size * channel_in3_sensor_value;



*/

/*




 	 TO TEST SINGLE_CONVERSION_MODE, INTERRUPT,

	rcc_clock_enable(RCC, GPIOAEN, ADC1EN, DMA2EN);

	gpio_config(GPIOA, GPIO_PIN_1, ANALOG_MODE);

	adc_config(ADC1, SINGLE_CONVERSION_MODE, SEQUENCE_LENGTH_1);

 	adc_set_channel(ADC1, ADC_IN1_CHANNEL, SEQUENCE_Rank_1, ADC_CLOCK_CYCLES_15);

 	adc_interrupt_open(ADC_IRQn, 0);

 	adc_interrupt_enable(ADC1);

 	ADC_trigger(ADC1);

 	Vstep1 = gDataArr[0];

 	Vstep2 = step_size * Vstep1;




 */

/*

 * TO TEST SINGLE_CONVERSION_MODE,POLLING METHOD, TEMP SENSOR

 	rcc_clock_enable(RCC);

 	adc_config(ADC1, SINGLE_CONVERSION_MODE, 1);

 	test_adc_internal_sensor_function(ADC1, ADC123_COMMON, ADC_TEMP);

  while (1)
  {
	 	gADCdata = adc_conversion_polling(ADC1, ADC_IN16_CHANNEL, gADCdata);

	 	Vstep = step_size * gADCdata;

	 	gtest_temp = ((Vstep - 0.76) - 0.0025) + 25;
  }


 * */


