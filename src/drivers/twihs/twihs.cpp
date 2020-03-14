#include "twihs.hpp"

#include "../clock/sysclk.hpp"
#include "../pmc/pmc.hpp"
#include "../delay/delay.hpp"

/* Low level time limit of I2C Fast Mode. */
#define LOW_LEVEL_TIME_LIMIT   384000
#define I2C_FAST_MODE_SPEED    400000
#define TWIHS_CLK_DIVIDER      2
#define TWIHS_CLK_CALC_ARGU    3
#define TWIHS_CLK_DIV_MAX      0xFF
#define TWIHS_CLK_DIV_MIN      7

#define FAIL	1
#define PASS	0

/* Enable TWIHS master mode. */
void twihs_enable_master_mode(Twihs* p_twihs)
{
    /* Set Master Disable bit and Slave Disable bit */
    p_twihs->TWIHS_CR = TWIHS_CR_MSDIS;
    p_twihs->TWIHS_CR = TWIHS_CR_SVDIS;

    /* Set Master Enable bit */
    p_twihs->TWIHS_CR = TWIHS_CR_MSEN;
}

/* Disable TWIHS master mode. */
void twihs_disable_master_mode(Twihs* p_twihs)
{
    /* Set Master Disable bit */
    p_twihs->TWIHS_CR = TWIHS_CR_MSDIS;
}

/* Initialize TWIHS master mode. */
uint32_t twihs_master_init(Twihs* p_twihs, const twihs_options_t* p_opt)
{
    uint32_t status = TWIHS_SUCCESS;

    /* Disable TWIHS interrupts */
    p_twihs->TWIHS_IDR = ~0UL;

    /* Dummy read in status register */
    p_twihs->TWIHS_SR;

    /* Reset TWIHS peripheral */
    twihs_reset(p_twihs);

    twihs_enable_master_mode(p_twihs);

    /* Select the speed */
    if (twihs_set_speed(p_twihs, p_opt->speed, p_opt->master_clk) == FAIL) {
        /* The desired speed setting is rejected */
        status = TWIHS_INVALID_ARGUMENT;
    }

    return status;
}

/* Set the I2C bus speed in conjunction with the clock frequency. */
uint32_t twihs_set_speed(Twihs *p_twihs, uint32_t ul_speed, uint32_t ul_mck)
{
    uint32_t ckdiv = 0;
    uint32_t c_lh_div;
    uint32_t cldiv, chdiv;

    /* High-Speed can be only used in slave mode, 400k is the max speed allowed for master */
    if (ul_speed > I2C_FAST_MODE_SPEED) {
        return FAIL;
    }

    /* Low level time not less than 1.3us of I2C Fast Mode. */
    if (ul_speed > LOW_LEVEL_TIME_LIMIT) {
        /* Low level of time fixed for 1.3us. */
        cldiv = ul_mck / (LOW_LEVEL_TIME_LIMIT * TWIHS_CLK_DIVIDER) - TWIHS_CLK_CALC_ARGU;
        chdiv = ul_mck / ((ul_speed + (ul_speed - LOW_LEVEL_TIME_LIMIT)) * TWIHS_CLK_DIVIDER) - TWIHS_CLK_CALC_ARGU;
        
        /* cldiv must fit in 8 bits, ckdiv must fit in 3 bits */
        while ((cldiv > TWIHS_CLK_DIV_MAX) && (ckdiv < TWIHS_CLK_DIV_MIN)) {
            /* Increase clock divider */
            ckdiv++;
            /* Divide cldiv value */
            cldiv /= TWIHS_CLK_DIVIDER;
        }
        /* chdiv must fit in 8 bits, ckdiv must fit in 3 bits */
        while ((chdiv > TWIHS_CLK_DIV_MAX) && (ckdiv < TWIHS_CLK_DIV_MIN)) {
            /* Increase clock divider */
            ckdiv++;
            /* Divide cldiv value */
            chdiv /= TWIHS_CLK_DIVIDER;
        }

        /* set clock waveform generator register */
        p_twihs->TWIHS_CWGR =
                TWIHS_CWGR_CLDIV(cldiv) | TWIHS_CWGR_CHDIV(chdiv) |
                TWIHS_CWGR_CKDIV(ckdiv);
    } else {
        c_lh_div = ul_mck / (ul_speed * TWIHS_CLK_DIVIDER) - TWIHS_CLK_CALC_ARGU;

        /* cldiv must fit in 8 bits, ckdiv must fit in 3 bits */
        while ((c_lh_div > TWIHS_CLK_DIV_MAX) && (ckdiv < TWIHS_CLK_DIV_MIN)) {
            /* Increase clock divider */
            ckdiv++;
            /* Divide cldiv value */
            c_lh_div /= TWIHS_CLK_DIVIDER;
        }

        /* set clock waveform generator register */
        p_twihs->TWIHS_CWGR =
                TWIHS_CWGR_CLDIV(c_lh_div) | TWIHS_CWGR_CHDIV(c_lh_div) |
                TWIHS_CWGR_CKDIV(ckdiv);
    }

    return PASS;
}

/* Test if a chip answers a given I2C address. */
uint32_t twihs_twihs_probe(Twihs *p_twihs, uint8_t uc_slave_addr)
{
    twihs_packet_t packet;
    uint8_t data = 0;

    /* Data to send */
    packet.buffer = &data;
    /* Data length */
    packet.length = 1;
    /* Slave chip address */
    packet.chip = (uint32_t) uc_slave_addr;
    /* Internal chip address */
    packet.addr[0] = 0;
    /* Address length */
    packet.addr_length = 0;

    /* Perform a master write access */
    return (twihs_master_write(p_twihs, &packet));
}

/* Construct the TWIHS module address register field. */
static uint32_t twihs_mk_addr(const uint8_t *addr, int len)
{
    uint32_t val;

    if (len == 0)
        return 0;

    val = addr[0];
    if (len > 1) {
        val <<= 8;
        val |= addr[1];
    }
    if (len > 2) {
        val <<= 8;
        val |= addr[2];
    }
    return val;
}

/* Read multiple bytes from a TWIHS compatible slave device. */
uint32_t twihs_master_read(Twihs* p_twihs, twihs_packet_t* p_packet)
{
    uint32_t status, cnt = p_packet->length;
    uint8_t* buffer = static_cast<uint8_t*>(p_packet->buffer);
    uint32_t timeout = TWIHS_TIMEOUT;

    /* Check argument */
    if (cnt == 0) {
        return TWIHS_INVALID_ARGUMENT;
    }

    /* Set read mode, slave address and 3 internal address byte lengths */
    p_twihs->TWIHS_MMR = 0;
    p_twihs->TWIHS_MMR = TWIHS_MMR_MREAD | TWIHS_MMR_DADR(p_packet->chip) |
            ((p_packet->addr_length << TWIHS_MMR_IADRSZ_Pos) &
            TWIHS_MMR_IADRSZ_Msk);

    /* Set internal address for remote chip */
    p_twihs->TWIHS_IADR = 0;
    p_twihs->TWIHS_IADR = twihs_mk_addr(p_packet->addr, p_packet->addr_length);

    /* Send a START Condition */
    p_twihs->TWIHS_CR = TWIHS_CR_START;

    while (cnt > 0) {
        status = p_twihs->TWIHS_SR;
        if (status & TWIHS_SR_NACK) {
            return TWIHS_RECEIVE_NACK;
        }
        if (!timeout--) {
            return TWIHS_ERROR_TIMEOUT;
        }
        /* Last byte ? */
        if (cnt == 1) {
            p_twihs->TWIHS_CR = TWIHS_CR_STOP;
        }

        if (!(status & TWIHS_SR_RXRDY)) {
            continue;
        }
        *buffer++ = p_twihs->TWIHS_RHR;

        cnt--;
        timeout = TWIHS_TIMEOUT;
    }

    while (!(p_twihs->TWIHS_SR & TWIHS_SR_TXCOMP)) {
    }

    p_twihs->TWIHS_SR;

    return TWIHS_SUCCESS;
}

/* Write multiple bytes to a TWIHS compatible slave device. */
uint32_t twihs_master_write(Twihs *p_twihs, twihs_packet_t *p_packet)
{
    uint32_t status, cnt = p_packet->length;
    uint8_t* buffer = static_cast<uint8_t*>(p_packet->buffer);

    /* Check argument */
    if (cnt == 0) {
        return TWIHS_INVALID_ARGUMENT;
    }

    /* Set write mode, slave address and 3 internal address byte lengths */
    p_twihs->TWIHS_MMR = 0;
    p_twihs->TWIHS_MMR = TWIHS_MMR_DADR(p_packet->chip) |
            ((p_packet->addr_length << TWIHS_MMR_IADRSZ_Pos) &
            TWIHS_MMR_IADRSZ_Msk);

    /* Set internal address for remote chip */
    p_twihs->TWIHS_IADR = 0;
    p_twihs->TWIHS_IADR = twihs_mk_addr(p_packet->addr, p_packet->addr_length);

    /* Send all bytes */
    while (cnt > 0) {
        status = p_twihs->TWIHS_SR;
        if (status & TWIHS_SR_NACK) {
            return TWIHS_RECEIVE_NACK;
        }

        if (!(status & TWIHS_SR_TXRDY)) {
            continue;
        }
        p_twihs->TWIHS_THR = *buffer++;

        cnt--;
    }

    while (1) {
        status = p_twihs->TWIHS_SR;
        if (status & TWIHS_SR_NACK) {
            return TWIHS_RECEIVE_NACK;
        }

        if (status & TWIHS_SR_TXRDY) {
            break;
        }
    }

    p_twihs->TWIHS_CR = TWIHS_CR_STOP;

    while (!(p_twihs->TWIHS_SR & TWIHS_SR_TXCOMP)) {
    }

    return TWIHS_SUCCESS;
}

/* Enable TWIHS interrupts. */
void twihs_enable_interrupt(Twihs* p_twihs, uint32_t ul_sources)
{
    /* Enable the specified interrupts */
    p_twihs->TWIHS_IER = ul_sources;
}

/* Disable TWIHS interrupts. */
void twihs_disable_interrupt(Twihs *p_twihs, uint32_t ul_sources)
{
    /* Disable the specified interrupts */
    p_twihs->TWIHS_IDR = ul_sources;
    /* Dummy read */
    p_twihs->TWIHS_SR;
}

/* Get TWIHS interrupt status. */
uint32_t twihs_get_interrupt_status(Twihs *p_twihs)
{
    return p_twihs->TWIHS_SR;
}

/* Read TWIHS interrupt mask. */
uint32_t twihs_get_interrupt_mask(Twihs *p_twihs)
{
    return p_twihs->TWIHS_IMR;
}

/* Reads a byte from the TWIHS bus. */
uint8_t twihs_read_byte(Twihs *p_twihs)
{
    return p_twihs->TWIHS_RHR;
}

/* Sends a byte of data to one of the TWIHS slaves on the bus. */
void twihs_write_byte(Twihs *p_twihs, uint8_t uc_byte)
{
    p_twihs->TWIHS_THR = uc_byte;
}

/* Enable TWIHS slave mode. */
void twihs_enable_slave_mode(Twihs *p_twihs)
{
    /* Set Master Disable bit and Slave Disable bit */
    p_twihs->TWIHS_CR = TWIHS_CR_MSDIS;
    p_twihs->TWIHS_CR = TWIHS_CR_SVDIS;

    /* Set Slave Enable bit */
    p_twihs->TWIHS_CR = TWIHS_CR_SVEN;
}

/* Disable TWIHS slave mode. */
void twihs_disable_slave_mode(Twihs *p_twihs)
{
    /* Set Slave Disable bit */
    p_twihs->TWIHS_CR = TWIHS_CR_SVDIS;
}

/* Initialize TWIHS slave mode. */
void twihs_slave_init(Twihs *p_twihs, uint32_t ul_device_addr)
{
    /* Disable TWIHS interrupts */
    p_twihs->TWIHS_IDR = ~0UL;
    p_twihs->TWIHS_SR;

    /* Reset TWIHS */
    twihs_reset(p_twihs);

    /* Set slave address in slave mode */
    p_twihs->TWIHS_SMR = TWIHS_SMR_SADR(ul_device_addr);

    /* Enable slave mode */
    twihs_enable_slave_mode(p_twihs);
}

/* Set TWIHS slave address. */
void twihs_set_slave_addr(Twihs *p_twihs, uint32_t ul_device_addr)
{
    /* Set slave address */
    p_twihs->TWIHS_SMR = TWIHS_SMR_SADR(ul_device_addr);
}

/* Read data from master. */
uint32_t twihs_slave_read(Twihs *p_twihs, uint8_t *p_data)
{
    uint32_t status, cnt = 0;

    do {
        status = p_twihs->TWIHS_SR;
        if (status & TWIHS_SR_SVACC) {
            if (!(status & (TWIHS_SR_GACC | TWIHS_SR_SVREAD)) &&
            (status & TWIHS_SR_RXRDY)) {
                *p_data++ = (uint8_t) p_twihs->TWIHS_RHR;
                cnt++;
            }
        } else if ((status & (TWIHS_SR_EOSACC | TWIHS_SR_TXCOMP))
                    == (TWIHS_SR_EOSACC | TWIHS_SR_TXCOMP)) {
            break;
        }
    } while (1);

    return cnt;
}

/* Write data to TWIHS bus. */
uint32_t twihs_slave_write(Twihs *p_twihs, uint8_t *p_data)
{
    uint32_t status, cnt = 0;

    do {
        status = p_twihs->TWIHS_SR;
        if (status & TWIHS_SR_SVACC) {
            if (!(status & (TWIHS_SR_GACC | TWIHS_SR_NACK)) &&
            ((status & (TWIHS_SR_SVREAD | TWIHS_SR_TXRDY))
            == (TWIHS_SR_SVREAD | TWIHS_SR_TXRDY))) {
                p_twihs->TWIHS_THR = *p_data++;
                cnt++;
            }
        } else if ((status & (TWIHS_SR_EOSACC | TWIHS_SR_TXCOMP))
                    == (TWIHS_SR_EOSACC | TWIHS_SR_TXCOMP)) {
            break;
        }
    } while (1);

    return cnt;
}

/* Reset TWIHS. */
void twihs_reset(Twihs *p_twihs)
{
    /* Set SWRST bit to reset TWIHS peripheral */
    p_twihs->TWIHS_CR = TWIHS_CR_SWRST;
    p_twihs->TWIHS_RHR;
}

/* Enables/Disables write protection mode. */
void twihs_set_write_protection(Twihs *p_twihs, bool flag)
{
    if (flag) {
        p_twihs->TWIHS_WPMR = TWIHS_WPMR_WPKEY_PASSWD | TWIHS_WPMR_WPEN;
    } else {
        p_twihs->TWIHS_WPMR = TWIHS_WPMR_WPKEY_PASSWD;
    }
}

/* Read the write protection status. */
void twihs_read_write_protection_status(Twihs *p_twihs, uint32_t *p_status)
{
    *p_status = p_twihs->TWIHS_WPSR;
}

/* Set the prescaler, TLOW:SEXT, TLOW:MEXT and clock high max cycles for SMBUS mode. */
void twihs_smbus_set_timing(Twihs *p_twihs, uint32_t ul_timing)
{
    p_twihs->TWIHS_SMBTR = ul_timing;;
}

/* Set the filter for TWIHS. */
void twihs_set_filter(Twihs *p_twihs, uint32_t ul_filter)
{
    p_twihs->TWIHS_FILTR = ul_filter;;
}

/*
    * A mask can be applied on the slave device address in slave mode in order to allow multiple
    * address answer. For each bit of the MASK field set to one the corresponding SADR bit will be masked.
    */
void twihs_mask_slave_addr(Twihs *p_twihs, uint32_t ul_mask)
{
    p_twihs->TWIHS_SMR |= TWIHS_SMR_MASK(ul_mask);
}

/*
    * Set sleepwalking match mode.
    *
    * \param p_twihs Pointer to a TWIHS instance.
    * \param ul_matching_addr1   Address 1 value.
    * \param ul_matching_addr2   Address 2 value.
    * \param ul_matching_addr3   Address 3 value.
    * \param ul_matching_data   Data value.
    * \param flag1 ture for set, false for no.
    * \param flag2 ture for set, false for no.
    * \param flag3 ture for set, false for no.
    * \param flag ture for set, false for no.
    */
void twihs_set_sleepwalking(Twihs *p_twihs,
    uint32_t ul_matching_addr1, bool flag1,
    uint32_t ul_matching_addr2, bool flag2,
    uint32_t ul_matching_addr3, bool flag3,
    uint32_t ul_matching_data, bool flag)
{
    uint32_t temp = 0;

    if (flag1) {
        temp |= TWIHS_SWMR_SADR1(ul_matching_addr1);
    }

    if (flag2) {
        temp |= TWIHS_SWMR_SADR2(ul_matching_addr2);
    }

    if (flag3) {
        temp |= TWIHS_SWMR_SADR3(ul_matching_addr3);
    }

    if (flag) {
        temp |= TWIHS_SWMR_DATAM(ul_matching_data);
    }

    p_twihs->TWIHS_SWMR = temp;
}

/*
    * Initializes the TWIHS interface.
    * If it was successful returns true, false otherwise.
    */
bool twihs_init(uint32_t speed)
{
    pmc_enable_periph_clk(ID_TWIHS0);

    twihs_options_t options;
    options.master_clk = sysclk_get_peripheral_hz();
    options.speed = speed;

    if (twihs_master_init(TWIHS0, &options) != TWIHS_SUCCESS)
        return false;

    delay_ms(10);

    return true;
}

/*
    * Sends data to the desired address.
    * If the writing was successful returns true, false otherwise;
    */
bool twihs_write(uint8_t chip_addr, uint8_t mem_addr, uint8_t* data, uint16_t length)
{
    twihs_packet_t packet;
    packet.chip			= chip_addr;
    packet.addr[0]		= mem_addr;
    packet.addr_length	= 1;
    packet.buffer		= data;
    packet.length		= length;

    return twihs_master_write(TWIHS0, &packet) == TWIHS_SUCCESS;
}

/*
    * Receives data from the desired address.
    * If the reading was successful returns true, false otherwise.
    */
bool twihs_read(uint8_t chip_addr, uint8_t mem_addr, uint8_t* data, uint16_t length)
{
    twihs_packet_t packet;
    packet.chip			= chip_addr;
    packet.addr[0]		= mem_addr;
    packet.addr_length	= 1;
    packet.buffer		= data;
    packet.length		= length;

    return twihs_master_read(TWIHS0, &packet) == TWIHS_SUCCESS;
}