/*
 * adc.h
 *
 *  Created on: Jul 26, 2024
 *      Author: ozan akyol
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include <stdlib.h> 
#include <stdint.h>
#include "stm32f407xx.h"

#define __IO    volatile


#ifdef SENSOR_TEST_MODE
#endif



/*
 * This library requires a circular buffer implementation.
 * We need to collect data in different modes:
 * ADC Single Conversion Mode and Continuous Conversion Mode.
 * Therefore, I will implement a circular buffer here.
 * In the future, I plan to develop a separate library for the circular buffer.
 *
 * #include "circular_buffer.h"
 *
 *typedef struct{
	uint16_t* const circularBuffer;
};
 */


/*ADC's Registers*/
typedef struct
{
  __IO uint32_t SR;     /*ADC status register*/
  __IO uint32_t CR1;    /*ADC control register 1*/
  __IO uint32_t CR2;    /*ADC control register 2*/
  __IO uint32_t SMPR1;  /*ADC sample time register 1*/
  __IO uint32_t SMPR2;  /*ADC sample time register 2*/
  __IO uint32_t JOFR1;  /*ADC injected channel data offset register */
  __IO uint32_t JOFR2;  /*ADC injected channel data offset register 2*/
  __IO uint32_t JOFR3;  /*ADC injected channel data offset register 3*/
  __IO uint32_t JOFR4;  /*ADC injected channel data offset register 4*/
  __IO uint32_t HTR;    /*ADC watchdog higher threshold register*/
  __IO uint32_t LTR;    /*ADC watchdog lower threshold register*/
  __IO uint32_t SQR1;   /*ADC regular sequence register 1*/
  __IO uint32_t SQR2;   /*ADC regular sequence register 2*/
  __IO uint32_t SQR3;   /*ADC regular sequence register 3*/
  __IO uint32_t JSQR;   /*ADC injected sequence register*/
  __IO uint32_t JDR1;   /*ADC injected data register 1*/
  __IO uint32_t JDR2;   /*ADC injected data register 2*/
  __IO uint32_t JDR3;   /*ADC injected data register 3*/
  __IO uint32_t JDR4;   /*ADC injected data register 4*/
  __IO uint32_t DR;     /*ADC regular data register*/
} ADC_TypeDef;

/**
  * @brief Reset and Clock Control
  */

typedef struct
{
  __IO uint32_t CR;            /*!< RCC clock control register,                                  Address offset: 0x00 */
  __IO uint32_t PLLCFGR;       /*!< RCC PLL configuration register,                              Address offset: 0x04 */
  __IO uint32_t CFGR;          /*!< RCC clock configuration register,                            Address offset: 0x08 */
  __IO uint32_t CIR;           /*!< RCC clock interrupt register,                                Address offset: 0x0C */
  __IO uint32_t AHB1RSTR;      /*!< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
  __IO uint32_t AHB2RSTR;      /*!< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
  __IO uint32_t AHB3RSTR;      /*!< RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                                    */
  __IO uint32_t APB1RSTR;      /*!< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
  __IO uint32_t APB2RSTR;      /*!< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                               */
  __IO uint32_t AHB1ENR;       /*!< RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
  __IO uint32_t AHB2ENR;       /*!< RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
  __IO uint32_t AHB3ENR;       /*!< RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                                    */
  __IO uint32_t APB1ENR;       /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
  __IO uint32_t APB2ENR;       /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                               */
  __IO uint32_t AHB1LPENR;     /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
  __IO uint32_t AHB2LPENR;     /*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
  __IO uint32_t AHB3LPENR;     /*!< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                                    */
  __IO uint32_t APB1LPENR;     /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
  __IO uint32_t APB2LPENR;     /*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                               */
  __IO uint32_t BDCR;          /*!< RCC Backup domain control register,                          Address offset: 0x70 */
  __IO uint32_t CSR;           /*!< RCC clock control & status register,                         Address offset: 0x74 */
  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                               */
  __IO uint32_t SSCGR;         /*!< RCC spread spectrum clock generation register,               Address offset: 0x80 */
  __IO uint32_t PLLI2SCFGR;    /*!< RCC PLLI2S configuration register,                           Address offset: 0x84 */
} RCC_TypeDef;


typedef struct
{
  __IO uint32_t CSR;    /*!< ADC Common status register,                  Address offset: ADC1 base address + 0x300 */
  __IO uint32_t CCR;    /*!< ADC common control register,                 Address offset: ADC1 base address + 0x304 */
  __IO uint32_t CDR;    /*!< ADC common regular data register for dual
                             AND triple modes,                            Address offset: ADC1 base address + 0x308 */
} ADC_Common_TypeDef;


/**
  * @brief General Purpose I/O
  */

typedef struct
{
  __IO uint32_t MODER;    /*!< GPIO port mode register,               Address offset: 0x00      */
  __IO uint32_t OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04      */
  __IO uint32_t OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08      */
  __IO uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
  __IO uint32_t IDR;      /*!< GPIO port input data register,         Address offset: 0x10      */
  __IO uint32_t ODR;      /*!< GPIO port output data register,        Address offset: 0x14      */
  __IO uint32_t BSRR;     /*!< GPIO port bit set/reset register,      Address offset: 0x18      */
  __IO uint32_t LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C      */
  __IO uint32_t AFR[2];   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
} GPIO_TypeDef;


typedef enum ConversionTypeDefEnum{
	SINGLE_CONVERSION_MODE,
	CONTINUOUS_CONVERSION_MODE,
	SCAN_SINGLE_CONVERSION_MODE,
	SCAN_CONTINUOUS_CONVERSION_MODE,
	DISCONTIUOUS_MODE,
	NUM_OF_MODE,
}ConversionType;

typedef enum ADCStatesTypeDefEnum{
	ADC_IDLE,
	ADC_READY,
	ADC_BUSY,
	ADC_TIMEOUT,
	NUM_OF_ADC_STATES,
}ADCStatesType;

typedef enum ADCStatusTypeDefEnum{
	ADC_OK = 1,
	ADC_ERROR,
	NUM_OF_STATUS,
}ADCStatusType;

typedef enum ADCTestTypeDefEnum{
	ADC_TEMP,
	ADC_VBAT,
	NUM_OF_ADC_TEST_DEVICES,
}ADCTestTypeDef;

typedef enum InterruptPriorityEnum{
	HIGHEST_PRIORITY,
	NUM_OF_PRIORITY_LEVEL,
}InterruptPriority;

typedef enum {
	ADC1_CHANNEL_IN0,
	ADC1_CHANNEL_IN1,
	ADC1_CHANNEL_IN2,
	ADC1_CHANNEL_IN3,
	ADC1_CHANNEL_IN4,
	ADC1_CHANNEL_IN5,
	ADC1_CHANNEL_IN6,
	ADC1_CHANNEL_IN7,
	ADC1_CHANNEL_IN8,
	ADC1_CHANNEL_IN9,
	ADC1_CHANNEL_IN10,
	ADC1_CHANNEL_IN11,
	ADC1_CHANNEL_IN12,
	ADC1_CHANNEL_IN13,
	ADC1_CHANNEL_IN14,
	ADC1_CHANNEL_IN15,
	ADC1_CHANNEL_IN16,
	ADC1_CHANNEL_IN17,
	ADC1_CHANNEL_IN18,
	MAX_CHANNEL_NUM_OF_ADC
}ChannelNumber;

typedef enum DMAStatusTypeDefEnum{
	DMA_OK = 1,
	DMA_ERROR,
	NUM_OF_DMA_STATUS,
}DMAStatusType;


typedef struct{
	ADC_TypeDef* instance;

}adc_object;

#define GPIO_PIN_SET(__PIN_POS__, __PIN_MODE__)	 (__PIN_MODE__ << __PIN_POS__)

#define CLEAR_BIT_POS		  (0x03U)
#define CLEAR_BIT_POS_3		  (0x07U)
#define INPUT_MODE			  (0x00U)
#define OUTPUT_MODE			  (0x01U)
#define ALTERNATE_MODE	 	  (0x02U)
#define ANALOG_MODE			  (0x03U)

#define GPIO_PIN_0       	  (0U)
#define GPIO_PIN_1	          (2U)
#define GPIO_PIN_2         	  (4U)
#define GPIO_PIN_3            (6U)
#define GPIO_PIN_4            (8U)
#define GPIO_PIN_5            (10U)
#define GPIO_PIN_6            (12U)
#define GPIO_PIN_7            (14U)
#define GPIO_PIN_8            (16U)
#define GPIO_PIN_9            (18U)
#define GPIO_PIN_10           (20U)
#define GPIO_PIN_11           (22U)
#define GPIO_PIN_12           (24U)
#define GPIO_PIN_13           (26U)
#define GPIO_PIN_14           (28U)
#define GPIO_PIN_15           (30U)



/* Peripheral base address*/
#define PERIPH_BASE           0x40000000UL

/* Peripheral memory map */
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000UL)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000UL)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x10000000UL)

/* Module base address*/
#define RCC_BASE              (AHB1PERIPH_BASE + 0x3800UL)
#define ADC1_BASE             (APB2PERIPH_BASE + 0x2000UL)
#define GPIOA_BASE            (AHB1PERIPH_BASE + 0x0000UL)
#define DMA2_BASE             (AHB1PERIPH_BASE + 0x6400UL)
#define DMA2_Stream0_BASE     (DMA2_BASE + 0x010UL)

#define ADC123_COMMON_BASE    (APB2PERIPH_BASE + 0x2300UL)
/* Legacy define */
#define ADC_BASE               ADC123_COMMON_BASE

#define ADC123_COMMON       ((ADC_Common_TypeDef *) ADC123_COMMON_BASE)

#define ADC1_DATA_REG_OFFSET	(0x4C)
#define ADC1_DATA_REG_ADDRESS	(ADC1_BASE + ADC1_DATA_REG_OFFSET)

#define MASK_N(n)                     ((1 << n) - 1)
#define MAKE_MASK(offset, len)        (MASK_N(len) << (offset))
#define MASK_EQ(reg, mask)            (((reg & mask) == mask))
#define GET_BITS(reg, offset, len)    ((reg >> offset) & MASK_N(len))
#define VAL2FIELD(val, offset, len)   (((val & MASK_N(len)) << offset))
#define IS_IN_RANGE(VAL, MIN, MAX)    ((VAL > MIN) && (VAL < MAX))
#define IS_IN_RANGE_EQ(VAL, MIN, MAX) ((VAL >= MIN) && (VAL <= MAX))
 
#define IS_ENUM_IN_RANGE(val, enum_max) ((val >= 0) && (val < enum_max))
 
#define ADC1                ((ADC_TypeDef *) ADC1_BASE)
#define RCC                 ((RCC_TypeDef *) RCC_BASE)
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define DMA2                ((DMA_TypeDef *) DMA2_BASE)
#define DMA2_Stream0        ((DMA_Stream_TypeDef *) DMA2_Stream0_BASE)


/*Define RCC Bits.*/
#define RCC_GPIOA_POS			(0U)
#define GPIOAEN     			(RCC_GPIOA_POS)

#define RCC_ADC1_POS      		(8U)
#define ADC1EN     			  	(RCC_ADC1_POS)

#define RCC_DMA2_POS			(22U)
#define DMA2EN					(RCC_DMA2_POS)

/*ADC Channel Number*/
#define ADC_IN0_CHANNEL			(0x00U)
#define ADC_IN1_CHANNEL			(0x01U)
#define ADC_IN2_CHANNEL			(0x02U)
#define ADC_IN3_CHANNEL			(0x03U)
#define ADC_IN4_CHANNEL			(0x04U)
#define ADC_IN5_CHANNEL			(0x05U)
#define ADC_IN6_CHANNEL			(0x06U)
#define ADC_IN7_CHANNEL			(0x07U)
#define ADC_IN8_CHANNEL			(0x08U)
#define ADC_IN9_CHANNEL			(0x09U)
#define ADC_IN10_CHANNEL		(0x0AU)
#define ADC_IN11_CHANNEL		(0x0BU)
#define ADC_IN12_CHANNEL		(0x0CU)
#define ADC_IN13_CHANNEL		(0x0DU)
#define ADC_IN14_CHANNEL		(0x0EU)
#define ADC_IN15_CHANNEL		(0x0FU)
#define ADC_IN16_CHANNEL		(0x10U)	//Internal Temp channel
#define ADC_IN17_CHANNEL		(0x11U)
#define ADC_IN18_CHANNEL		(0x12U)	// VBAT channel


/*ADC Channel Number*/
#define SEQUENCE_LENGTH_1		(0x00U)
#define SEQUENCE_LENGTH_2		(0x01U)
#define SEQUENCE_LENGTH_3		(0x02U)
#define SEQUENCE_LENGTH_4		(0x03U)
#define SEQUENCE_LENGTH_5		(0x04U)
#define SEQUENCE_LENGTH_6		(0x05U)
#define SEQUENCE_LENGTH_7		(0x06U)
#define SEQUENCE_LENGTH_8		(0x07U)
#define SEQUENCE_LENGTH_9		(0x08U)
#define SEQUENCE_LENGTH_10		(0x09U)
#define SEQUENCE_LENGTH_11		(0x0AU)
#define SEQUENCE_LENGTH_12		(0x0BU)
#define SEQUENCE_LENGTH_13		(0x0CU)
#define SEQUENCE_LENGTH_14		(0x0DU)
#define SEQUENCE_LENGTH_15		(0x0EU)
#define SEQUENCE_LENGTH_16		(0x0FU)


/*ADC Rank Number*/
#define SEQUENCE_Rank_1			(1U)
#define SEQUENCE_Rank_2			(2U)
#define SEQUENCE_Rank_3			(3U)
#define SEQUENCE_Rank_4			(4U)
#define SEQUENCE_Rank_5			(5U)
#define SEQUENCE_Rank_6			(6U)
#define SEQUENCE_Rank_7			(7U)
#define SEQUENCE_Rank_8			(8U)
#define SEQUENCE_Rank_9			(9U)
#define SEQUENCE_Rank_10		(10U)
#define SEQUENCE_Rank_11		(11U)
#define SEQUENCE_Rank_12		(12U)
#define SEQUENCE_Rank_13		(13U)
#define SEQUENCE_Rank_14		(14U)
#define SEQUENCE_Rank_15		(15U)
#define SEQUENCE_Rank_16		(16U)

/*Clock cycles*/
#define ADC_CLOCK_CYCLES_3		(0x00)
#define ADC_CLOCK_CYCLES_15		(0x01)
#define ADC_CLOCK_CYCLES_28		(0x02)
#define ADC_CLOCK_CYCLES_56		(0x03)
#define ADC_CLOCK_CYCLES_84		(0x04)
#define ADC_CLOCK_CYCLES_112	(0x05)
#define ADC_CLOCK_CYCLES_144	(0x06)

/*Conversion Number*/
#define ADC_CONV_NUM_1			(0x00)	// 1 conversion
#define ADC_CONV_NUM_2			(0x01)	// 2 conversion
#define ADC_CONV_NUM_3			(0x02)	// 3 conversion
#define ADC_CONV_NUM_4			(0x03)  // 4 conversion
#define ADC_CONV_NUM_5			(0x04)	// 5 conversion
#define ADC_CONV_NUM_6			(0x05)	// 6 conversion
#define ADC_CONV_NUM_7			(0x06)
#define ADC_CONV_NUM_8			(0x07)
#define ADC_CONV_NUM_9			(0x08)
#define ADC_CONV_NUM_10			(0x09)
#define ADC_CONV_NUM_11			(0x0A)
#define ADC_CONV_NUM_12			(0x0B)
#define ADC_CONV_NUM_13			(0x0C)
#define ADC_CONV_NUM_14			(0x0D)
#define ADC_CONV_NUM_15			(0x0E)
#define ADC_CONV_NUM_16			(0x0F)




/*ADC Bits.*/
#define ANALOG_PA1							(0x03U << 2)
#define ANALOG_PA2							(0x03U << 4)
#define ANALOG_PA3							(0x03U << 6)
#define IDR_PA1								(1U << 1)
#define CR2_SWSTART							(1U << 30)
#define CR2_ADON							(1U << 0)
#define CR2_CONT							(1U << 1)
#define CR2_RIGHT_ALIGN						(1U << 11)
#define CR1_12_BIT_RESOLUTION				(0x03U << 24)
#define ADC_CR2_EOCS						(1U << 10)
#define ADC_SR_EOC_BIT						(1U << 1)
#define ADC_SR_JEOC_BIT						(1U << 2)
#define ADC_SR_AWD_BIT						(1U << 0)
#define ADC_SR_OVR_BIT						(1U << 5)
#define ADC_CR1_SCAN_BIT					(1U << 8)
#define ADC_SQR1_SEQUENCE_LENGTH_BIT 		(0x02 << 20)	//Sample 3 channel
#define ADC_SINGLE_CONV_LENGTH_BIT			(0x0F)
#define ADC_CR2_SWSTART						(1U << 30)
#define ADC_CR2_DMA_EN						(1U << 8)
#define ADC_CR2_DDS							(1U << 9)

/*ADC Interrupt flag control*/
#define ADC_EOCIE_BIT			(1U << 5)
#define ADC_JEOCIE_BIT			(1U << 7)
#define ADC_AWDIE_BIT			(1U << 6)
#define ADC_OVRIE_BIT			(1U << 26)

/*ADC Test Bits.*/
#define ADC_TSVREFE_BIT			(1U << 23)
#define ADC_VBATE_BIT			(1U << 22)

/*DMA2 Bits*/
#define DMA2_EN 				(1U << 0)
#define DMA2_CHSEL				(0x07U << 25)
#define STREAM_PRIORITY_MASK	(0x03U << 16)
#define STREAM_HIGH_PRIORITY	(0x02U << 16)
#define DATA_TRANSFER_DIR		(0x03U << 6)
#define DMA_CIRC				(1U << 8)
#define DMA_PINC 				(1U << 9)
#define DMA_MINC				(1U << 10)
#define DMA_MBURST				(0x03U << 23)
#define DMA_PBURST				(0x03U << 21)
#define DMA_PSIZE_16BIT			(1U << 11)
#define DMA_MSIZE_16BIT			(1U << 13)



#define ADC_IS_FLAG_SET(__ADC__,__REG__,__FLAG__)		(__ADC__-> __REG__  & __FLAG__) == __FLAG__
#define ADC_IS_BIT_SET(__ADC__,__REG__,__BIT__)			(__ADC__-> __REG__  & __BIT__) == __BIT__
#define ADC_IS_EOCIE_FLAG(__ADC__)						(__ADC__-> CR1  & ADC_EOCIE_BIT) == ADC_EOCIE_BIT
#define ADC_IS_JEOCIE_FLAG(__ADC__)						(__ADC__-> CR1  & ADC_JEOCIE_BIT) == ADC_JEOCIE_BIT
#define ADC_IS_AWDIE_FLAG(__ADC__)						(__ADC__-> CR1  & ADC_AWDIE_BIT) == ADC_AWDIE_BIT
#define ADC_IS_OVRIE_FLAG(__ADC__)						(__ADC__-> CR1  & ADC_OVRIE_BIT) == ADC_OVRIE_BIT

#define ADC_CLEAR_BIT(__ADC__,__REG__,__FLAG__)		(__ADC__ -> __REG__ &= ~(__FLAG__))

#define MAX_SIZE_OF_CONVERTED_DATA			10
#define MAX_NUM_OF_CONVERTED_DATA			(uint16_t)3

/*Config functions*/
void rcc_clock_enable(RCC_TypeDef* rcc_p, uint8_t gpiox, uint8_t adcx, uint8_t dmax);
void gpio_config(GPIO_TypeDef* gpio, uint8_t pin_number, uint8_t mode);
void adc_config(ADC_TypeDef* adcx, ConversionType conv_mode, uint16_t total_num_of_channel);
void adc_interrupt_irq(void);
void adc_interrupt_enable(ADC_TypeDef* adcx);
void adc_interrupt_open(IRQn_Type irq_num, uint32_t priority);
uint32_t adc_conversion_polling(ADC_TypeDef* adcx, uint32_t channel_number, uint32_t sdata);
void adc_continuous_conversion_polling(ADC_TypeDef* adcx, uint32_t* data_ptr, uint32_t num_of_conversion);
void test_adc_internal_sensor_function(ADC_TypeDef * adcx, ADC_Common_TypeDef * adc_common, ADCTestTypeDef param);
uint32_t get_adc_value(ADC_TypeDef* adcx);
ADCStatusType adc_set_channel(ADC_TypeDef* adcx, uint32_t channel, uint32_t rank, uint8_t clock_cycles);
void adc_open(ADC_TypeDef* adcx);
void ADC_trigger(ADC_TypeDef * adcx);


DMAStatusType DMA_open_stream(DMA_Stream_TypeDef* dma);
DMAStatusType DMA_close_stream(DMA_Stream_TypeDef* dma);
void DMA_config(DMA_Stream_TypeDef* dma);



extern uint16_t *ptr;
extern uint16_t gDataArr[MAX_SIZE_OF_CONVERTED_DATA];
extern uint16_t gDmaArr[MAX_CHANNEL_NUM_OF_ADC];

#endif /* INC_ADC_H_ */
