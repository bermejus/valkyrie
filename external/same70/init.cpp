#include "init.hpp"

#include <sam.h>
#include <board_same70.hpp>
#include <drivers/fpu/fpu.hpp>

void ioport_set_port_peripheral_mode(ioport_port_t port, ioport_port_mask_t masks, ioport_mode_t mode)
{
    ioport_set_port_mode(port, masks, mode);
    ioport_disable_port(port, masks);
}

void ioport_set_pin_peripheral_mode(ioport_pin_t pin, ioport_mode_t mode)
{
    ioport_set_pin_mode(pin, mode);
    ioport_disable_pin(pin);
}

void ioport_set_pin_input_mode(ioport_pin_t pin, ioport_mode_t mode, ioport_sense sense)
{
    ioport_set_pin_dir(pin, IOPORT_DIR_INPUT);
    ioport_set_pin_mode(pin, mode);
    ioport_set_pin_sense_mode(pin, sense);
}

/*
 * TCM memory enable
 * The function enables TCM memories
 */
static inline void tcm_disable()
{
    __DSB();
    __ISB();

    SCB->ITCMCR &= ~(uint32_t)(1UL);
    SCB->DTCMCR &= ~(uint32_t)SCB_DTCMCR_EN_Msk;

    __DSB();
    __ISB();
}

void board_init()
{
    /* Disable the watchdog */
    WDT->WDT_MR = WDT_MR_WDDIS;

    /* Enable data & instruction cache */
    SCB_EnableICache();
    SCB_EnableDCache();

    /* TCM Configuration */
	EFC->EEFC_FCR = (EEFC_FCR_FKEY_PASSWD | EEFC_FCR_FCMD_CGPB | EEFC_FCR_FARG(8));
	EFC->EEFC_FCR = (EEFC_FCR_FKEY_PASSWD | EEFC_FCR_FCMD_CGPB | EEFC_FCR_FARG(7));
	tcm_disable();

    /* Initialize IO Ports */
    ioport_init();

    /* Configure TWIHS ports (I2C) */
    ioport_set_pin_peripheral_mode(TWIHS_DATA_GPIO, TWIHS_DATA_FLAGS);
    ioport_set_pin_peripheral_mode(TWIHS_CLK_GPIO, TWIHS_CLK_FLAGS);

    /* Configure SPI ports */
    ioport_set_pin_peripheral_mode(SPI_MISO_GPIO, SPI_MISO_FLAGS);
    ioport_set_pin_peripheral_mode(SPI_MOSI_GPIO, SPI_MOSI_FLAGS);
    ioport_set_pin_peripheral_mode(SPI_SCLK_GPIO, SPI_SCLK_FLAGS);
    ioport_set_pin_peripheral_mode(SPI_NPCS0_GPIO, SPI_NPCS0_FLAGS);
    ioport_set_pin_peripheral_mode(SPI_NPCS1_GPIO, SPI_NPCS1_FLAGS);

    /* Configure HSMCI ports */
    ioport_set_pin_peripheral_mode(HSMCI_MCCK_GPIO, HSMCI_MCCK_FLAGS);
    ioport_set_pin_peripheral_mode(HSMCI_MCCDA_GPIO, HSMCI_MCCDA_FLAGS);
    ioport_set_pin_peripheral_mode(HSMCI_MCDA0_GPIO, HSMCI_MCDA0_FLAGS);
    ioport_set_pin_peripheral_mode(HSMCI_MCDA1_GPIO, HSMCI_MCDA1_FLAGS);
    ioport_set_pin_peripheral_mode(HSMCI_MCDA2_GPIO, HSMCI_MCDA2_FLAGS);
    ioport_set_pin_peripheral_mode(HSMCI_MCDA3_GPIO, HSMCI_MCDA3_FLAGS);
    ioport_set_pin_input_mode(HSMCI_MCDET_GPIO, HSMCI_MCDET_FLAGS, HSMCI_MCDET_SENSE);

    /* Configure UART ports */
    ioport_set_pin_peripheral_mode(UART_RXD_GPIO, UART_RXD_FLAGS);
    ioport_set_pin_peripheral_mode(UART_TXD_GPIO, UART_TXD_FLAGS);

    /* Configure PWM ports */
    ioport_set_pin_peripheral_mode(PWM0_H0_GPIO, PWM0_H0_FLAGS);
    ioport_set_pin_peripheral_mode(PWM0_H1_GPIO, PWM0_H1_FLAGS);
    ioport_set_pin_peripheral_mode(PWM0_H2_GPIO, PWM0_H2_FLAGS);
    ioport_set_pin_peripheral_mode(PWM0_H3_GPIO, PWM0_H3_FLAGS);
    ioport_set_pin_peripheral_mode(LED_R_GPIO, LED_R_FLAGS);
    ioport_set_pin_peripheral_mode(LED_G_GPIO, LED_G_FLAGS);
    ioport_set_pin_peripheral_mode(LED_B_GPIO, LED_B_FLAGS);

    /* Configure external USER pins */
    ioport_set_pin_dir(EXT_PIN1_GPIO, IOPORT_DIR_INPUT);
    ioport_set_pin_dir(EXT_PIN2_GPIO, IOPORT_DIR_INPUT);
    ioport_set_pin_dir(EXT_PIN3_GPIO, IOPORT_DIR_INPUT);
    ioport_set_pin_dir(EXT_PIN4_GPIO, IOPORT_DIR_INPUT);

    /* Configure sensors interrupt pins */
    ioport_set_pin_input_mode(GPIO_BMP388_INT, GPIO_BMP388_INT_FLAGS, GPIO_BMP388_INT_SENSE);
    ioport_set_pin_input_mode(GPIO_BMI088_ACCEL_INT, GPIO_BMI088_ACCEL_INT_FLAGS, GPIO_BMI088_ACCEL_INT_SENSE);
    ioport_set_pin_input_mode(GPIO_BMI088_GYRO_INT, GPIO_BMI088_GYRO_INT_FLAGS, GPIO_BMI088_GYRO_INT_SENSE);
    ioport_set_pin_input_mode(GPIO_BMM150_INT, GPIO_BMM150_INT_FLAGS, GPIO_BMM150_INT_SENSE);

    /* Enable FPU */
    fpu_enable();
}