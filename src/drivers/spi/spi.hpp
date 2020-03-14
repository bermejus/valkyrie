#pragma once

#include <sam.h>

/* Time-out value (number of attempts). */
#define SPI_TIMEOUT		15000

/*
 * \brief Generate Peripheral Chip Select Value from Chip Select ID
 * \note When chip select n is working, PCS bit n is set to low level.
 *
 * \param chip_sel_id The chip select number used
 */
#define spi_get_pcs(chip_sel_id) ((~(1u<<(chip_sel_id)))&0xF)

/* Status codes used by the SPI driver. */
enum spi_status_t
{
    SPI_ERROR = -1,
    SPI_OK = 0,
    SPI_ERROR_TIMEOUT = 1,
    SPI_ERROR_ARGUMENT,
    SPI_ERROR_OVERRUN,
    SPI_ERROR_MODE_FAULT,
    SPI_ERROR_OVERRUN_AND_MODE_FAULT
};

/* SPI Chip Select behavior modes while transferring. */
enum spi_cs_behavior_t {
    /* CS does not rise until a new transfer is requested on different chip select. */
    SPI_CS_KEEP_LOW = SPI_CSR_CSAAT,
    /* CS rises if there is no more data to transfer. */
    SPI_CS_RISE_NO_TX = 0,
    /* CS is de-asserted systematically during a time DLYBCS. */
    SPI_CS_RISE_FORCED = SPI_CSR_CSNAAT
};

struct spi_cmd_t
{
    uint8_t* data;
    uint8_t* cmd;
    uint8_t cmd_size;
    uint8_t dummy_size : 7;
    bool rx : 1;
    uint32_t data_size;
};

struct spi_cmd_const_t
{
    const uint8_t* data;
    uint8_t* cmd;
    uint8_t cmd_size;
    uint8_t dummy_size : 7;
    bool rx : 1;
    uint32_t data_size;
};

/* Reset SPI and set it to Slave mode. */
static inline void spi_reset(Spi *p_spi)
{
    p_spi->SPI_CR = SPI_CR_SWRST;
}

/* Enable SPI. */
static inline void spi_enable(Spi *p_spi)
{
    p_spi->SPI_CR = SPI_CR_SPIEN;
}

/*
    * Disable SPI.
    *
    * CS is de-asserted, which indicates that the last data is done, and user
    * should check TX_EMPTY before disabling SPI.
    */
static inline void spi_disable(Spi *p_spi)
{
    p_spi->SPI_CR = SPI_CR_SPIDIS;
}

/*
    * Issue a LASTXFER command.
    * The next transfer is the last transfer and after that CS is de-asserted.
    */
static inline void spi_set_lastxfer(Spi *p_spi)
{
    p_spi->SPI_CR |= SPI_CR_LASTXFER;
}

/* Set SPI to Master mode. */
static inline void spi_set_master_mode(Spi *p_spi)
{
    p_spi->SPI_MR |= SPI_MR_MSTR;
}

/* Set SPI to Slave mode. */
static inline void spi_set_slave_mode(Spi *p_spi)
{
    p_spi->SPI_MR &= (~SPI_MR_MSTR);
}

/* Get SPI work mode. */
static inline uint32_t spi_get_mode(Spi *p_spi)
{
    if (p_spi->SPI_MR & SPI_MR_MSTR) {
        return 1;
    } else {
        return 0;
    }
}

/*
    * Set Variable Peripheral Select.
    * Peripheral Chip Select can be controlled by SPI_TDR.
    */
static inline void spi_set_variable_peripheral_select(Spi *p_spi)
{
    p_spi->SPI_MR |= SPI_MR_PS;
}

/**
 * Set Fixed Peripheral Select.
 * Peripheral Chip Select is controlled by SPI_MR.
 */
static inline void spi_set_fixed_peripheral_select(Spi *p_spi)
{
    p_spi->SPI_MR &= (~SPI_MR_PS);
}

/*
    * Get Peripheral Select mode.
    * return 1 for Variable mode, 0 for fixed mode.
    */
static inline uint32_t spi_get_peripheral_select_mode(Spi *p_spi)
{
    if (p_spi->SPI_MR & SPI_MR_PS) {
        return 1;
    } else {
        return 0;
    }
}

/* Enable Peripheral Select Decode. */
static inline void spi_enable_peripheral_select_decode(Spi *p_spi)
{
    p_spi->SPI_MR |= SPI_MR_PCSDEC;
}

/* Disable Peripheral Select Decode. */
static inline void spi_disable_peripheral_select_decode(Spi *p_spi)
{
    p_spi->SPI_MR &= (~SPI_MR_PCSDEC);
}

/*
    * Get Peripheral Select Decode mode.
    * return 1 for decode mode, 0 for direct mode.
    */
static inline uint32_t spi_get_peripheral_select_decode_setting(Spi *p_spi)
{
    if (p_spi->SPI_MR & SPI_MR_PCSDEC) {
        return 1;
    } else {
        return 0;
    }
}

/* Enable Mode Fault Detection. */
static inline void spi_enable_mode_fault_detect(Spi *p_spi)
{
    p_spi->SPI_MR &= (~SPI_MR_MODFDIS);
}

/* Disable Mode Fault Detection. */
static inline void spi_disable_mode_fault_detect(Spi *p_spi)
{
    p_spi->SPI_MR |= SPI_MR_MODFDIS;
}

/*
    * Check if mode fault detection is enabled.
    * return 1 for disabled, 0 for enabled.
    */
static inline uint32_t spi_get_mode_fault_detect_setting(Spi *p_spi)
{
    if (p_spi->SPI_MR & SPI_MR_MODFDIS) {
        return 1;
    } else {
        return 0;
    }
}

/* Enable waiting RX_EMPTY before transfer starts. */
static inline void spi_enable_tx_on_rx_empty(Spi *p_spi)
{
    p_spi->SPI_MR |= SPI_MR_WDRBT;
}

/* Disable waiting RX_EMPTY before transfer starts. */
static inline void spi_disable_tx_on_rx_empty(Spi *p_spi)
{
    p_spi->SPI_MR &= (~SPI_MR_WDRBT);
}

/*
    * Check if SPI waits RX_EMPTY before transfer starts.
    * return 1 for SPI waits, 0 for no wait.
    */
static inline uint32_t spi_get_tx_on_rx_empty_setting(Spi *p_spi)
{
    if (p_spi->SPI_MR & SPI_MR_WDRBT) {
        return 1;
    } else {
        return 0;
    }
}

/* Enable loopback mode. */
static inline void spi_enable_loopback(Spi *p_spi)
{
    p_spi->SPI_MR |= SPI_MR_LLB;
}

/* Disable loopback mode. */
static inline void spi_disable_loopback(Spi *p_spi)
{
    p_spi->SPI_MR &= (~SPI_MR_LLB);
}

/* Read status register. */
static inline uint32_t spi_read_status(Spi *p_spi)
{
    return p_spi->SPI_SR;
}

/*
    * Test if the SPI is enabled.
    * return 1 if the SPI is enabled, otherwise 0.
    */
static inline uint32_t spi_is_enabled(Spi *p_spi)
{
    if (p_spi->SPI_SR & SPI_SR_SPIENS) {
        return 1;
    } else {
        return 0;
    }
}

/* Put one data to a SPI peripheral. */
static inline void spi_put(Spi *p_spi, uint16_t data)
{
    p_spi->SPI_TDR = SPI_TDR_TD(data);
}

/* Get one data to a SPI peripheral. */
static inline uint16_t spi_get(Spi *p_spi)
{
    return (p_spi->SPI_RDR & SPI_RDR_RD_Msk);
}

/*
    * Check if all transmissions are complete.
    * 
    * return 1 if transmissions are complete.
    * return 0 if transmissions are not complete.
    */
static inline uint32_t spi_is_tx_empty(Spi *p_spi)
{
    if (p_spi->SPI_SR & SPI_SR_TXEMPTY) {
        return 1;
    } else {
        return 0;
    }
}

/*
    * Check if all transmissions are ready.
    *
    * return 1 if transmissions are complete.
    * return 0 if transmissions are not complete.
    */
static inline uint32_t spi_is_tx_ready(Spi *p_spi)
{
    if (p_spi->SPI_SR & SPI_SR_TDRE) {
        return 1;
    } else {
        return 0;
    }
}

/*
    * Check if the SPI contains a received character.
    * return 1 if the SPI Receive Holding Register is full, otherwise 0.
    */
static inline uint32_t spi_is_rx_full(Spi *p_spi)
{
    if (p_spi->SPI_SR & SPI_SR_RDRF) {
        return 1;
    } else {
        return 0;
    }
}

/*
    * Check if all receptions are ready.
    * return 1 if the SPI Receiver is ready, otherwise 0.
    */
static inline uint32_t spi_is_rx_ready(Spi *p_spi)
{
    if ((p_spi->SPI_SR & (SPI_SR_RDRF | SPI_SR_TXEMPTY))
            == (SPI_SR_RDRF | SPI_SR_TXEMPTY)) {
        return 1;
    } else {
        return 0;
    }
}

/* Enable SPI interrupts. */
static inline void spi_enable_interrupt(Spi *p_spi, uint32_t ul_sources)
{
    p_spi->SPI_IER = ul_sources;
}

/* Disable SPI interrupts. */
static inline void spi_disable_interrupt(Spi *p_spi, uint32_t ul_sources)
{
    p_spi->SPI_IDR = ul_sources;
}

/* Read SPI interrupt mask. */
static inline uint32_t spi_read_interrupt_mask(Spi *p_spi)
{
    return p_spi->SPI_IMR;
}

/*
    * Get transmit data register address for DMA operation.
    * return Transmit address for DMA access.
    */
static inline void *spi_get_tx_access(Spi *p_spi)
{
    return (void *)&(p_spi->SPI_TDR);
}

/*
    * Get receive data register address for DMA operation.
    * return Receive address for DMA access.
    */
static inline void *spi_get_rx_access(Spi *p_spi)
{
    return (void *)&(p_spi->SPI_RDR);
}

void spi_init(Spi* p_spi);
void spi_enable_clock(Spi *p_spi);
void spi_disable_clock(Spi *p_spi);
void spi_set_peripheral_chip_select_value(Spi *p_spi, uint32_t ul_value);
void spi_select_cs(Spi* p_spi, uint32_t ul_value);
void spi_set_delay_between_chip_select(Spi *p_spi, uint32_t ul_delay);
spi_status_t spi_read(Spi *p_spi, uint16_t *us_data, uint8_t *p_pcs);
spi_status_t spi_write(Spi *p_spi, uint16_t us_data, uint8_t uc_pcs, uint8_t uc_last);
void spi_send_cmd(Spi* p_spi, spi_cmd_t* cmd);
void spi_send_cmd_const(Spi* p_spi, spi_cmd_const_t* cmd);

void spi_set_clock_polarity(Spi *p_spi, uint32_t ul_pcs_ch, uint32_t ul_polarity);
void spi_set_clock_phase(Spi *p_spi, uint32_t ul_pcs_ch, uint32_t ul_phase);
void spi_configure_cs_behavior(Spi *p_spi, uint32_t ul_pcs_ch, uint32_t ul_cs_behavior);
void spi_set_bits_per_transfer(Spi *p_spi, uint32_t ul_pcs_ch, uint32_t ul_bits);
int16_t spi_calc_baudrate_div(const uint32_t baudrate, uint32_t mck);
int16_t spi_set_baudrate_div(Spi *p_spi, uint32_t ul_pcs_ch, uint8_t uc_baudrate_divider);
void spi_set_transfer_delay(Spi *p_spi, uint32_t ul_pcs_ch, uint8_t uc_dlybs, uint8_t uc_dlybct);

void spi_set_writeprotect(Spi *p_spi, uint32_t ul_enable);
uint32_t spi_get_writeprotect_status(Spi *p_spi);