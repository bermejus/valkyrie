#include "hsmci.hpp"

#include <sam.h>
#include "../clock/sysclk.hpp"
#include "../pmc/pmc.hpp"
#include "../xdmac/xdmac.hpp"

#define XDMAC_HW_ID_HSMCI			0
#define CONF_HSMCI_XDMAC_CHANNEL	XDMAC_CHANNEL_HWID_HSMCI

// Current position (byte) of the transfer started by hsmci_adtc_start()
static uint32_t hsmci_transfer_pos;
// Size block requested by last hsmci_adtc_start()
static uint16_t hsmci_block_size;
// Total number of block requested by last hsmci_adtc_start()
static uint16_t hsmci_nb_block;

static void hsmci_reset();
static void hsmci_set_speed(uint32_t speed, uint32_t mck);
static bool hsmci_wait_busy();
static bool hsmci_send_cmd_execute(uint32_t cmdr, sdmmc_cmd_def_t cmd, uint32_t arg);

static void hsmci_reset()
{
    uint32_t mr = HSMCI->HSMCI_MR;
    uint32_t dtor = HSMCI->HSMCI_DTOR;
    uint32_t sdcr = HSMCI->HSMCI_SDCR;
    uint32_t cstor = HSMCI->HSMCI_CSTOR;
    uint32_t cfg = HSMCI->HSMCI_CFG;
    HSMCI->HSMCI_CR = HSMCI_CR_SWRST;
    HSMCI->HSMCI_MR = mr;
    HSMCI->HSMCI_DTOR = dtor;
    HSMCI->HSMCI_SDCR = sdcr;
    HSMCI->HSMCI_CSTOR = cstor;
    HSMCI->HSMCI_CFG = cfg;
    HSMCI->HSMCI_DMA = 0;

    // Enable the HSMCI
    HSMCI->HSMCI_CR = HSMCI_CR_PWSEN | HSMCI_CR_MCIEN;
}

/*
    * Set speed of the HSMCI clock.
    *
    * \param speed    HSMCI clock speed in Hz.
    * \param mck      MCK clock speed in Hz.
    */
static void hsmci_set_speed(uint32_t speed, uint32_t mck)
{
    uint32_t clkdiv = 0;
    uint32_t clkodd = 0;
    // clock divider, represent (((clkdiv << 1) + clkodd) + 2)
    uint32_t div = 0;

    // Speed = MCK clock / (((clkdiv << 1) + clkodd) + 2)
    if ((speed * 2) < mck) {
        div = (mck / speed) - 2;
        if (mck % speed) {
            // Ensure that the card speed not be higher than expected.
            div++;
        }
        clkdiv = div >> 1;
        // clkodd is the last significant bit of the clock divider (div).
        clkodd = div % 2;
    } else {
        clkdiv = 0;
        clkodd = 0;
    }

    HSMCI->HSMCI_MR &= ~HSMCI_MR_CLKDIV_Msk;
    HSMCI->HSMCI_MR |= HSMCI_MR_CLKDIV(clkdiv);
    if (clkodd) {
        HSMCI->HSMCI_MR |= HSMCI_MR_CLKODD;
    }
    else {
        HSMCI->HSMCI_MR &= ~HSMCI_MR_CLKODD;
    }
}

/*
    * Wait the end of busy signal on data line.
    * Return true if success, otherwise false.
    */
static bool hsmci_wait_busy()
{
    uint32_t busy_wait = 0xFFFFFFFF;
    uint32_t sr;

    do {
        sr = HSMCI->HSMCI_SR;
        if (busy_wait-- == 0) {
            hsmci_reset();
            return false;
        }
    } while (!((sr & HSMCI_SR_NOTBUSY) && ((sr & HSMCI_SR_DTIP) == 0)));
    return true;
}

/*
    * Send a command.
    *
    * \param cmdr       CMDR register bit to use for this command
    * \param cmd        Command definition
    * \param arg        Argument of the command
    *
    * Return true if success, otherwise false.
    */
static bool hsmci_send_cmd_execute(uint32_t cmdr, sdmmc_cmd_def_t cmd, uint32_t arg)
{
    uint32_t sr;

    cmdr |= HSMCI_CMDR_CMDNB(cmd) | HSMCI_CMDR_SPCMD_STD;
    if (cmd & SDMMC_RESP_PRESENT) {
        cmdr |= HSMCI_CMDR_MAXLAT;
        if (cmd & SDMMC_RESP_136) {
            cmdr |= HSMCI_CMDR_RSPTYP_136_BIT;
        } else if (cmd & SDMMC_RESP_BUSY) {
            cmdr |= HSMCI_CMDR_RSPTYP_R1B;
        } else {
            cmdr |= HSMCI_CMDR_RSPTYP_48_BIT;
        }
    }
    if (cmd & SDMMC_CMD_OPENDRAIN) {
        cmdr |= HSMCI_CMDR_OPDCMD_OPENDRAIN;
    }

    // Write argument
    HSMCI->HSMCI_ARGR = arg;
    // Write and start command
    HSMCI->HSMCI_CMDR = cmdr;

    // Wait end of command
    do {
        sr = HSMCI->HSMCI_SR;
        if (cmd & SDMMC_RESP_CRC) {
            if (sr & (HSMCI_SR_CSTOE | HSMCI_SR_RTOE
                    | HSMCI_SR_RENDE | HSMCI_SR_RCRCE
                    | HSMCI_SR_RDIRE | HSMCI_SR_RINDE)) {
                hsmci_reset();
                return false;
            }
        } else {
            if (sr & (HSMCI_SR_CSTOE | HSMCI_SR_RTOE
                    | HSMCI_SR_RENDE
                    | HSMCI_SR_RDIRE | HSMCI_SR_RINDE)) {
                hsmci_reset();
                return false;
            }
        }
    } while (!(sr & HSMCI_SR_CMDRDY));

    if (cmd & SDMMC_RESP_BUSY) {
        if (!hsmci_wait_busy()) {
            return false;
        }
    }
    return true;
}

void hsmci_init()
{
    pmc_enable_periph_clk(ID_HSMCI);
    pmc_enable_periph_clk(ID_XDMAC);

    // Set the Data Timeout Register to 2 Mega Cycles
    HSMCI->HSMCI_DTOR = HSMCI_DTOR_DTOMUL_1048576 | HSMCI_DTOR_DTOCYC(2);
    // Set Completion Signal Timeout to 2 Mega Cycles
    HSMCI->HSMCI_CSTOR = HSMCI_CSTOR_CSTOMUL_1048576 | HSMCI_CSTOR_CSTOCYC(2);
    // Set Configuration Register
    HSMCI->HSMCI_CFG = HSMCI_CFG_FIFOMODE | HSMCI_CFG_FERRCTRL;
    // Set power saving to maximum value
    HSMCI->HSMCI_MR = HSMCI_MR_PWSDIV_Msk;

    // Enable the HSMCI and the Power Saving
    HSMCI->HSMCI_CR = HSMCI_CR_MCIEN | HSMCI_CR_PWSEN;
}

void hsmci_select_device(uint8_t slot, uint32_t clock, uint8_t bus_width, bool high_speed)
{
    uint32_t hsmci_bus_width = HSMCI_SDCR_SDCBUS_1;

    if (high_speed)
        HSMCI->HSMCI_CFG |= HSMCI_CFG_HSMODE;
    else
        HSMCI->HSMCI_CFG &= ~HSMCI_CFG_HSMODE;

    hsmci_set_speed(clock, sysclk_get_peripheral_hz());

    switch (bus_width)
    {
    case 1: hsmci_bus_width = HSMCI_SDCR_SDCBUS_1; break;
    case 4: hsmci_bus_width = HSMCI_SDCR_SDCBUS_4; break;
    case 8: hsmci_bus_width = HSMCI_SDCR_SDCBUS_8; break;
    default: break;
    }

    HSMCI->HSMCI_SDCR = HSMCI_SDCR_SDCSEL_SLOTA | hsmci_bus_width;
}

void hsmci_deselect_device(uint8_t slot) {}

uint8_t hsmci_get_bus_width(uint8_t slot)
{
    return 4;
}

bool hsmci_is_high_speed_capable()
{
    return true;
}

void hsmci_send_clock()
{
    HSMCI->HSMCI_MR &= ~(HSMCI_MR_WRPROOF | HSMCI_MR_RDPROOF | HSMCI_MR_FBYTE);
    HSMCI->HSMCI_ARGR = 0;

    HSMCI->HSMCI_CMDR = HSMCI_CMDR_RSPTYP_NORESP
            | HSMCI_CMDR_SPCMD_INIT
            | HSMCI_CMDR_OPDCMD_OPENDRAIN;
    
    while (!(HSMCI->HSMCI_SR & HSMCI_SR_CMDRDY));
}

bool hsmci_send_cmd(sdmmc_cmd_def_t cmd, uint32_t arg)
{
    HSMCI->HSMCI_MR &= ~(HSMCI_MR_WRPROOF | HSMCI_MR_RDPROOF | HSMCI_MR_FBYTE);
    HSMCI->HSMCI_DMA = 0;
    HSMCI->HSMCI_BLKR = 0;
    return hsmci_send_cmd_execute(0, cmd, arg);
}

uint32_t hsmci_get_response()
{
    return HSMCI->HSMCI_RSPR[0];
}

void hsmci_get_response_128(uint8_t* response)
{
    uint32_t response_32;

    for (uint8_t i = 0; i < 4; i++) {
        response_32 = HSMCI->HSMCI_RSPR[0];
        *response = (response_32 >> 24) & 0xFF;
        response++;
        *response = (response_32 >> 16) & 0xFF;
        response++;
        *response = (response_32 >>  8) & 0xFF;
        response++;
        *response = (response_32 >>  0) & 0xFF;
        response++;
    }
}

bool hsmci_adtc_start(sdmmc_cmd_def_t cmd, uint32_t arg, uint16_t block_size, uint16_t nb_block, bool access_block)
{
    uint32_t cmdr;

    if (access_block) {
        // Enable DMA for HSMCI
        HSMCI->HSMCI_DMA = HSMCI_DMA_DMAEN;
    } else {
        // Disable DMA for HSMCI
        HSMCI->HSMCI_DMA = 0;
    }

    HSMCI->HSMCI_MR |= HSMCI_MR_WRPROOF | HSMCI_MR_RDPROOF;
    /* Force byte transfer if needed */
    if (block_size & 0x3) {
        HSMCI->HSMCI_MR |= HSMCI_MR_FBYTE;
    } else {
        HSMCI->HSMCI_MR &= ~HSMCI_MR_FBYTE;
    }

    if (cmd & SDMMC_CMD_WRITE) {
        cmdr = HSMCI_CMDR_TRCMD_START_DATA | HSMCI_CMDR_TRDIR_WRITE;
    } else {
        cmdr = HSMCI_CMDR_TRCMD_START_DATA | HSMCI_CMDR_TRDIR_READ;
    }

    if (cmd & SDMMC_CMD_SDIO_BYTE) {
        cmdr |= HSMCI_CMDR_TRTYP_BYTE;
        // Value 0 corresponds to a 512-byte transfer
        HSMCI->HSMCI_BLKR = ((block_size % 512) << HSMCI_BLKR_BCNT_Pos);
    } else {
        HSMCI->HSMCI_BLKR = (block_size << HSMCI_BLKR_BLKLEN_Pos) |
                (nb_block << HSMCI_BLKR_BCNT_Pos);
        if (cmd & SDMMC_CMD_SDIO_BLOCK) {
            cmdr |= HSMCI_CMDR_TRTYP_BLOCK;
        } else if (cmd & SDMMC_CMD_STREAM) {
            cmdr |= HSMCI_CMDR_TRTYP_STREAM;
        } else if (cmd & SDMMC_CMD_SINGLE_BLOCK) {
            cmdr |= HSMCI_CMDR_TRTYP_SINGLE;
        } else if (cmd & SDMMC_CMD_MULTI_BLOCK) {
            cmdr |= HSMCI_CMDR_TRTYP_MULTIPLE;
        } else {
            return false;
        }
    }
    hsmci_transfer_pos = 0;
    hsmci_block_size = block_size;
    hsmci_nb_block = nb_block;

    return hsmci_send_cmd_execute(cmdr, cmd, arg);
}

bool hsmci_adtc_stop(sdmmc_cmd_def_t cmd, uint32_t arg)
{
    return hsmci_send_cmd_execute(HSMCI_CMDR_TRCMD_STOP_DATA, cmd, arg);
}

bool hsmci_read_word(uint32_t* value)
{
    uint32_t sr;
    uint8_t nbytes;

    // Wait data available
    nbytes = (hsmci_block_size & 0x3) ? 1 : 4;
    do {
        sr = HSMCI->HSMCI_SR;
        if (sr & (HSMCI_SR_UNRE | HSMCI_SR_OVRE | HSMCI_SR_DTOE | HSMCI_SR_DCRCE)) {
            hsmci_reset();
            return false;
        }
    } while (!(sr & HSMCI_SR_RXRDY));

    // Read data
    *value = HSMCI->HSMCI_RDR;
    hsmci_transfer_pos += nbytes;

    if (((uint64_t)hsmci_block_size * hsmci_nb_block) > hsmci_transfer_pos)
        return true;

    // Wait end of transfer
    do {
        sr = HSMCI->HSMCI_SR;

        if (sr & (HSMCI_SR_UNRE | HSMCI_SR_OVRE | HSMCI_SR_DTOE | HSMCI_SR_DCRCE)) {
            hsmci_reset();
            return false;
        }
    } while (!(sr & HSMCI_SR_XFRDONE));
    return true;
}

bool hsmci_write_word(uint32_t value)
{
    uint32_t sr;
    uint8_t nbytes;

    // Wait data available
    nbytes = (hsmci_block_size & 0x3) ? 1 : 4;
    do {
        sr = HSMCI->HSMCI_SR;
        if (sr & (HSMCI_SR_UNRE | HSMCI_SR_OVRE | HSMCI_SR_DTOE | HSMCI_SR_DCRCE)) {
            hsmci_reset();
            return false;
        }
    } while (!(sr & HSMCI_SR_TXRDY));

    // Write data
    HSMCI->HSMCI_TDR = value;
    hsmci_transfer_pos += nbytes;
    if (((uint64_t)hsmci_block_size * hsmci_nb_block) > hsmci_transfer_pos)
        return true;

    // Wait end of transfer
    do {
        sr = HSMCI->HSMCI_SR;

        if (sr & (HSMCI_SR_UNRE | HSMCI_SR_OVRE | HSMCI_SR_DTOE | HSMCI_SR_DCRCE)) {
            hsmci_reset();
            return false;
        }
    } while (!(sr & HSMCI_SR_NOTBUSY));
    return true;
}

bool hsmci_start_read_blocks(void* dest, uint16_t nb_block)
{
    uint32_t nb_data;
    uint8_t* ptr = (uint8_t*)dest;
    uint8_t nbytes;

    nb_data = nb_block * hsmci_block_size;
    nbytes = (hsmci_block_size & 0x3) ? 1 : 4;

    while (nb_data) {
        if (!hsmci_read_word((uint32_t*)ptr))
            return false;

        nb_data -= nbytes;
        ptr += nbytes;
    }

    return true;
}

bool hsmci_wait_end_of_read_blocks()
{
    return true;
}

bool hsmci_start_write_blocks(const void* src, uint16_t nb_block)
{
    uint32_t nb_data;
    uint8_t* ptr = (uint8_t*)src;

    nb_data = nb_block * hsmci_block_size;

    if (hsmci_block_size & 0x3) {
        while (nb_data) {
            if (!hsmci_write_word(*ptr))
                return false;
            
            nb_data--;
            ptr++;
        }
    } else {
        while (nb_data) {
            if (!hsmci_write_word(*(uint32_t*)ptr))
                return false;
            
            nb_data -= 4;
            ptr += 4;
        }
    }

    return true;
}

bool hsmci_wait_end_of_write_blocks()
{
    return true;
}