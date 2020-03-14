#ifndef _BOARD_SAME70_H_
#define _BOARD_SAME70_H_

#include "sam.h"

#include <drivers/ioport/ioport.hpp>
#include <drivers/pio/pio.hpp>

/*-----------------------------------------------*/

/** Board oscillator settings */
#define BOARD_FREQ_SLCK_XTAL            (32768U)
#define BOARD_FREQ_SLCK_BYPASS          (32768U)
#define BOARD_FREQ_MAINCK_XTAL          (12000000U)
#define BOARD_FREQ_MAINCK_BYPASS        (12000000U)

/** Master clock frequency */
#define BOARD_MCK                       CHIP_FREQ_CPU_MAX

/** board main clock xtal statup time */
#define BOARD_OSC_STARTUP_US            15625

/*-----------------------------------------------*/

/* TWIHS pins definition [PA3, PA4] */
#define TWIHS_DATA_GPIO		PIO_PA3_IDX
#define TWIHS_DATA_FLAGS	(IOPORT_MODE_MUX_A)
#define TWIHS_CLK_GPIO		PIO_PA4_IDX
#define TWIHS_CLK_FLAGS		(IOPORT_MODE_MUX_A)

/* SPI pins definition */
#define SPI_MISO_GPIO   	PIO_PD20_IDX
#define SPI_MISO_FLAGS  	(IOPORT_MODE_MUX_B)
#define SPI_MOSI_GPIO   	PIO_PD21_IDX
#define SPI_MOSI_FLAGS  	(IOPORT_MODE_MUX_B)
#define SPI_SCLK_GPIO   	PIO_PD22_IDX
#define SPI_SCLK_FLAGS  	(IOPORT_MODE_MUX_B)
#define SPI_NPCS0_GPIO  	PIO_PB2_IDX
#define SPI_NPCS0_FLAGS 	(IOPORT_MODE_MUX_D)
#define SPI_NPCS1_GPIO  	PIO_PD25_IDX
#define SPI_NPCS1_FLAGS 	(IOPORT_MODE_MUX_B)

/* HSMCI pins definition */
#define HSMCI_MCCK_GPIO         PIO_PA25_IDX
#define HSMCI_MCCK_FLAGS        (IOPORT_MODE_MUX_D)
#define HSMCI_MCCDA_GPIO        PIO_PA28_IDX
#define HSMCI_MCCDA_FLAGS       (IOPORT_MODE_MUX_C)
#define HSMCI_MCDA0_GPIO        PIO_PA30_IDX
#define HSMCI_MCDA0_FLAGS       (IOPORT_MODE_MUX_C)
#define HSMCI_MCDA1_GPIO        PIO_PA31_IDX
#define HSMCI_MCDA1_FLAGS       (IOPORT_MODE_MUX_C)
#define HSMCI_MCDA2_GPIO        PIO_PA26_IDX
#define HSMCI_MCDA2_FLAGS       (IOPORT_MODE_MUX_C)
#define HSMCI_MCDA3_GPIO        PIO_PA27_IDX
#define HSMCI_MCDA3_FLAGS       (IOPORT_MODE_MUX_C)

#define HSMCI_MCDET_GPIO        (PIO_PA24_IDX)
#define HSMCI_MCDET_FLAGS       (0)
#define HSMCI_MCDET_SENSE       (IOPORT_SENSE_BOTHEDGES)
#define PIN_HSMCI_MCDET         {PIO_PA24, PIOA, ID_PIOA, PIO_INPUT, \
                                PIO_DEFAULT}
#define PIN_HSMCI_MCDET_MASK    PIO_PA24
#define PIN_HSMCI_MCDET_PIO     PIOA
#define PIN_HSMCI_MCDET_ID      ID_PIOA
#define PIN_HSMCI_MCDET_TYPE    PIO_INPUT
#define PIN_HSMCI_MCDET_ATTR    (PIO_DEFAULT)
#define PIN_HSMCI_MCDET_IRQn    PIOA_IRQn

/* UART pins definition */
#define UART_RXD_GPIO   PIO_PB0_IDX
#define UART_RXD_FLAGS  (IOPORT_MODE_MUX_C)
#define UART_TXD_GPIO   PIO_PB1_IDX
#define UART_TXD_FLAGS  (IOPORT_MODE_MUX_C)

/* PWM pins definition */
#define PWM0_H0_GPIO    PIO_PA11_IDX
#define PWM0_H0_FLAGS   (IOPORT_MODE_MUX_B)
#define PWM0_H1_GPIO    PIO_PA12_IDX
#define PWM0_H1_FLAGS   (IOPORT_MODE_MUX_B)
#define PWM0_H2_GPIO    PIO_PA13_IDX
#define PWM0_H2_FLAGS   (IOPORT_MODE_MUX_B)
#define PWM0_H3_GPIO    PIO_PA14_IDX
#define PWM0_H3_FLAGS   (IOPORT_MODE_MUX_B)
#define PWM1_L0_GPIO    PIO_PD0_IDX
#define PWM1_L0_FLAGS   (IOPORT_MODE_MUX_B)
#define PWM1_L1_GPIO    PIO_PD2_IDX
#define PWM1_L1_FLAGS   (IOPORT_MODE_MUX_B)
#define PWM1_L2_GPIO    PIO_PD4_IDX
#define PWM1_L2_FLAGS   (IOPORT_MODE_MUX_B)

/* Thrust Vector Control (TVC) pins definition */
#define TVCX_GPIO       PWM0_H0_GPIO
#define TVCX_FLAGS      PWM0_H0_FLAGS
#define TVCX_CHANNEL    PWM_CHANNEL_0
#define TVCY_GPIO       PWM0_H1_GPIO
#define TVCY_FLAGS      PWM0_H1_FLAGS
#define TVCY_CHANNEL    PWM_CHANNEL_1

/* Buzzer pin configuration */
#define BUZZER_GPIO     PWM0_H2_GPIO
#define BUZZER_FLAGS    PWM0_H2_FLAGS
#define BUZZER_CHANNEL  PWM_CHANNEL_2

/* External PWM pin configuration */
#define EXT_PWM_GPIO    PWM0_H3_GPIO
#define EXT_PWM_FLAGS   PWM0_H3_FLAGS
#define EXT_PWM_CHANNEL PWM_CHANNEL_3

/* RGB LED pins configuration */
#define LED_R_GPIO      PWM1_L0_GPIO
#define LED_R_FLAGS     PWM1_L0_FLAGS
#define LED_R_CHANNEL   PWM_CHANNEL_0
#define LED_G_GPIO      PWM1_L2_GPIO
#define LED_G_FLAGS     PWM1_L2_FLAGS
#define LED_G_CHANNEL   PWM_CHANNEL_2
#define LED_B_GPIO      PWM1_L1_GPIO
#define LED_B_FLAGS     PWM1_L1_FLAGS
#define LED_B_CHANNEL   PWM_CHANNEL_1

/* External USER pins configuration */
#define EXT_PIN1_GPIO   PIO_PD6_IDX
#define EXT_PIN1_FLAGS
#define EXT_PIN2_GPIO   PIO_PD7_IDX
#define EXT_PIN2_FLAGS
#define EXT_PIN3_GPIO   PIO_PD8_IDX
#define EXT_PIN3_FLAGS
#define EXT_PIN4_GPIO   PIO_PD9_IDX
#define EXT_PIN4_FLAGS

/* I2C pins configuration */
#define I2C_SDA_GPIO    TWIHS_DATA_GPIO
#define I2C_SDA_FLAGS   TWIHS_DATA_FLAGS
#define I2C_SCL_GPIO    TWIHS_CLK_GPIO
#define I2C_SCL_FLAGS   TWIHS_CLK_FLAGS

/* Sensors interrupt pins definition [PA0, PA5, PD28, PD17] */
#define GPIO_BMP388_INT             (PIO_PA0_IDX)
#define GPIO_BMP388_INT_FLAGS       (IOPORT_MODE_PULLUP)
#define GPIO_BMP388_INT_SENSE       (IOPORT_SENSE_RISING)
#define PIN_BMP388_INT              {PIO_PA0, PIOA, ID_PIOA, PIO_INPUT, \
                                     PIO_PULLUP | PIO_IT_RISE_EDGE}
#define PIN_BMP388_INT_MASK         PIO_PA0
#define PIN_BMP388_INT_PIO          PIOA
#define PIN_BMP388_INT_ID           ID_PIOA
#define PIN_BMP388_INT_TYPE         PIO_INPUT
#define PIN_BMP388_INT_ATTR         (PIO_PULLUP | PIO_IT_RISE_EDGE)
#define PIN_BMP388_INT_IRQn         PIOA_IRQn

#define GPIO_BMI088_ACCEL_INT       (PIO_PA5_IDX)
#define GPIO_BMI088_ACCEL_INT_FLAGS (IOPORT_MODE_PULLUP)
#define GPIO_BMI088_ACCEL_INT_SENSE (IOPORT_SENSE_RISING)
#define PIN_BMI088_ACCEL_INT        {PIO_PA5, PIOA, ID_PIOA, PIO_INPUT, \
                                     PIO_PULLUP | PIO_IT_RISE_EDGE}
#define PIN_BMI088_ACCEL_INT_MASK   PIO_PA5
#define PIN_BMI088_ACCEL_INT_PIO    PIOA
#define PIN_BMI088_ACCEL_INT_ID     ID_PIOA
#define PIN_BMI088_ACCEL_INT_TYPE   PIO_INPUT
#define PIN_BMI088_ACCEL_INT_ATTR   (PIO_PULLUP | PIO_IT_RISE_EDGE)
#define PIN_BMI088_ACCEL_INT_IRQn   PIOA_IRQn

#define GPIO_BMI088_GYRO_INT        (PIO_PD28_IDX)
#define GPIO_BMI088_GYRO_INT_FLAGS  (IOPORT_MODE_PULLUP)
#define GPIO_BMI088_GYRO_INT_SENSE  (IOPORT_SENSE_RISING)
#define PIN_BMI088_GYRO_INT         {PIO_PD28, PIOD, ID_PIOD, PIO_INPUT, \
                                     PIO_PULLUP | PIO_IT_RISE_EDGE}
#define PIN_BMI088_GYRO_INT_MASK    PIO_PD28
#define PIN_BMI088_GYRO_INT_PIO     PIOD
#define PIN_BMI088_GYRO_INT_ID      ID_PIOD
#define PIN_BMI088_GYRO_INT_TYPE    PIO_INPUT
#define PIN_BMI088_GYRO_INT_ATTR    (PIO_PULLUP | PIO_IT_RISE_EDGE)
#define PIN_BMI088_GYRO_INT_IRQn    PIOD_IRQn

#define GPIO_BMM150_INT             (PIO_PD17_IDX)
#define GPIO_BMM150_INT_FLAGS       (IOPORT_MODE_PULLUP)
#define GPIO_BMM150_INT_SENSE           (IOPORT_SENSE_RISING)
#define PIN_BMM150_INT              {PIO_PD17, PIOD, ID_PIOD, PIO_INPUT, \
                                     PIO_PULLUP | PIO_IT_RISE_EDGE}
#define PIN_BMM150_INT_MASK         PIO_PD17
#define PIN_BMM150_INT_PIO          PIOD
#define PIN_BMM150_INT_ID           ID_PIOD
#define PIN_BMM150_INT_TYPE         PIO_INPUT
#define PIN_BMM150_INT_ATTR         (PIO_PULLUP | PIO_IT_RISE_EDGE)
#define PIN_BMM150_INT_IRQn         PIOD_IRQn

#define PINS_SENSORS    {PIN_BMP388_INT, PIN_BMI088_ACCEL_INT, PIN_BMI088_GYRO_INT, PIN_BMM150_INT}

#endif  // _BOARD_SAME70_H_