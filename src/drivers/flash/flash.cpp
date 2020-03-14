#include "flash.hpp"

#include <array>
#include "../clock/sysclk.hpp"
#include "../led/led.hpp"
#include "../spi/spi.hpp"

#define SST26_MANUFACTURER      0xBF
#define SST26_DEVICE_TYPE       0x26
#define SST26_DEVICE_ID_16MB    0x41
#define SST26_DEVICE_ID_32MB    0x42
#define SST26_DEVICE_ID_64MB    0x43

/*
    * SST26 Instructions.
    */
#define SST26_NOP       0x00
#define SST26_RSTEN     0x66
#define SST26_RST       0x99
#define SST26_EQIO      0x38
#define SST26_RSTQIO    0xFF
#define SST26_RDSR      0x05
#define SST26_WRSR      0x01
#define SST26_RDCR      0x35

#define SST26_READ      0x03
#define SST26_HS_READ   0x0B
#define SST26_SQOR      0x6B
#define SST26_SQIOR     0xEB
#define SST26_SDOR      0x3B
#define SST26_SDIOR     0xBB
#define SST26_SB        0xC0
#define SST26_RBSQI     0x0C
#define SST26_RBSPI     0xEC

#define SST26_JID       0x9F
#define SST26_QJID      0xAF
#define SST26_SFDP      0x5A

#define SST26_WREN      0x06
#define SST26_WRDI      0x04
#define SST26_SE        0x20
#define SST26_BE        0xD8
#define SST26_CE        0xC7
#define SST26_PP        0x02
#define SST26_QPP       0x32
#define SST26_WRSU      0xB0
#define SST26_WRRE      0x30

#define SST26_RBPR      0x72
#define SST26_WBPR      0x42
#define SST26_LBPR      0x8D
#define SST26_NVWLDR    0xE8
#define SST26_ULBPR     0x98
#define SST26_RSID      0x88
#define SST26_PSID      0xA5
#define SST26_LSID      0x85

/*
    * Status register bit definitions.
    */
#define SST26_SR_WIP    (1 << 0) /* Write in progress */
#define SST26_SR_WEL    (1 << 1) /* Write enable latch */
#define SST26_SR_WSE    (1 << 2) /* Write Suspend-Erase Status */
#define SST26_SR_WSP    (1 << 3) /* Write Suspend-Program Status */
#define SST26_SR_WPLD   (1 << 4) /* Write Protection Lock-Down Status */
#define SST26_SR_SEC    (1 << 5) /* Security ID Status */
#define SST26_SR_RES    (1 << 6) /* RFU */
#define SST26_SR_WIP2   (1 << 7) /* Write in progress */

static void wait_cs_released();
static void wait_write_completed();
static void wait_write_enabled();

static void set_spi_mode();

static bool write_in_progress();
static bool write_enabled();
static bool spi_mode();

/* Config instructions */
static void rsten();
static void rst();
static uint8_t rdsr();
static void wrsr(uint8_t reg);
static uint8_t rdcr();

/* Identification instructions */
static jedec_id_t read_id();
static void sfdp(uint32_t address, uint8_t* data, uint32_t length);

/* Write instructions */
static void write_enable();
static void write_disable();
static void sector_erase(uint32_t address);
static void block_erase(uint32_t address);
static void chip_erase();
static void page_program(uint32_t address, const uint8_t* data, uint32_t length);
static void wrsu();
static void wrre();

/* Protection instructions */
static void rbpr(uint8_t* reg);
static void wbpr(uint8_t* reg);
static void lbpr();
static void nvwldr(uint8_t* reg);
static void global_unlock();
static void rsid(uint16_t address, uint8_t* id, uint32_t length);
static void psid(uint16_t address, uint8_t* id, uint32_t length);
static void lsid();

void flash_init()
{
    spi_enable_clock(SPI0);
    spi_disable(SPI0);
    spi_reset(SPI0);
    spi_enable_tx_on_rx_empty(SPI0);
    spi_disable_mode_fault_detect(SPI0);
    spi_set_delay_between_chip_select(SPI0, 0x6);
    spi_set_master_mode(SPI0);
    spi_set_fixed_peripheral_select(SPI0);
    spi_set_peripheral_chip_select_value(SPI0, spi_get_pcs(0));
    spi_set_clock_polarity(SPI0, 0, 1);
    spi_set_clock_phase(SPI0, 0, 0);
    spi_set_bits_per_transfer(SPI0, 0, SPI_CSR_BITS_8_BIT);
    spi_configure_cs_behavior(SPI0, 0, SPI_CS_KEEP_LOW);
    spi_set_baudrate_div(SPI0, 0, sysclk_get_peripheral_hz() / 75000000);
    spi_set_transfer_delay(SPI0, 0, 0, 4); /* Set to 5 if it doesn't work */
    spi_enable(SPI0);
    
    jedec_id_t id = read_id();
    if (id.data[0] == SST26_MANUFACTURER && id.data[1] == SST26_DEVICE_TYPE)
    {
        switch (id.data[2])
        {
        case SST26_DEVICE_ID_16MB: break;
        case SST26_DEVICE_ID_32MB: break;
        case SST26_DEVICE_ID_64MB: break;
        default: Led::r_on(); while (true); break;
        }
    } else {
        Led::r_on();
        while (true);
    }

    write_enable();
    global_unlock();

    set_spi_mode();
}

void flash_write(uint32_t address, const uint8_t* data, uint32_t length)
{
    /*
        * Note: In order to write a FLASH chip, you will have to erase
        * the sector you are writing to, so let's read its content, then
        * erase it and finally modify it to write the required data.
        */
    
    if ((address + length) > SST26_32MBIT_SIZE)
    {
        Led::r_on();
        while (true);
    }

    /* Calculate how many sectors we have to manipulate. */
    const int first_sector = address >> 12;
    const int last_sector = (address + length - 1) >> 12;
    const int n_sectors = last_sector - first_sector + 1;

    int sector_base_addr;
    int w_length;
    int written_count = 0;

    /* Iterate through all requested sectors and populate them. */
    for (int i = first_sector; i <= last_sector; i++)
    {
        w_length = 4096 - (address & 0xFFF);
        sector_base_addr = (first_sector + i) << 12;

        /* If available block size is bigger than 4096 bytes, optimize the algorithm. */
        if ((w_length == 4096) && (w_length <= length))
        {
            sector_erase(address);

            for (int j = 0; j < 16; j++)
                page_program(address, &data[written_count + (j << 8)], 256);

            written_count += 4096;
            address += 4096;
            length -= 4096;
        }
        else
        {
            /* Read current sector */
            std::array<uint8_t, 4096> sector;
            flash_read(sector_base_addr, &sector[0], 4096);

            /* Erase sector. It is neccessary since we are dealing with a Flash device */
            sector_erase(sector_base_addr);

            /* Fill our buffer with the data we want to write. */
            if (w_length > length)
                w_length = length;

            for (int j = 0; j < w_length; j++)
                sector[(address & 0xFFF) + j] = data[written_count + j];
            
            written_count += w_length;
            address += w_length;
            length -= w_length;

            /* Page program */
            for (int j = 0; j < 16; j++)
                page_program((i << 12) + (j << 8), &sector[j << 8], 256);
        }
    }
}

static void wait_cs_released()
{
    while (!ioport_get_pin_level(SPI_NPCS0_GPIO));
}

static void wait_write_completed()
{
    while ((rdsr() & (SST26_SR_WIP)) != 0);
}

static void wait_write_enabled()
{
    while ((rdsr() & SST26_SR_WEL) != SST26_SR_WEL);
}

static void set_spi_mode()
{
    if (!spi_mode())
    {
        write_enable();
        wrsr(0x00);

        if (!spi_mode())
        {
            Led::r_on();
            while (true);
        }
    }
}

static bool write_enabled()
{
    return ((rdsr() & SST26_SR_WEL) == SST26_SR_WEL);
}

static bool spi_mode()
{
    return ((rdcr() & 0x2) != 0); 
}

static void rsten()
{
    flash_cmd_t opcode {
        .cmd = SST26_RSTEN
    };

    spi_cmd_t cmd {
        .data = nullptr,
        .cmd = &opcode.cmd,
        .cmd_size = 1,
        .dummy_size = 0,
        .rx = false,
        .data_size = 0
    };

    spi_select_cs(SPI0, 0);
    spi_send_cmd(SPI0, &cmd);
    wait_cs_released();
}

static void rst()
{
    flash_cmd_t opcode {
        .cmd = SST26_RST
    };

    spi_cmd_t cmd {
        .data = nullptr,
        .cmd = &opcode.cmd,
        .cmd_size = 1,
        .dummy_size = 0,
        .rx = false,
        .data_size = 0
    };

    spi_select_cs(SPI0, 0);
    spi_send_cmd(SPI0, &cmd);
    wait_cs_released();
}

static uint8_t rdsr()
{
    uint8_t reg = 0;

    flash_cmd_t opcode {
        .cmd = SST26_RDSR
    };

    spi_cmd_t cmd {
        .data = &reg,
        .cmd = &opcode.cmd,
        .cmd_size = 1,
        .dummy_size = 0,
        .rx = true,
        .data_size = 1
    };

    spi_select_cs(SPI0, 0);
    spi_send_cmd(SPI0, &cmd);
    wait_cs_released();

    return reg;
}

static void wrsr(uint8_t reg)
{
    uint8_t data[2] { 0x00, reg };

    flash_cmd_t opcode {
        .cmd = SST26_WRSR
    };

    spi_cmd_t cmd {
        .data = &reg,
        .cmd = &opcode.cmd,
        .cmd_size = 1,
        .dummy_size = 0,
        .rx = false,
        .data_size = 2
    };

    spi_select_cs(SPI0, 0);
    spi_send_cmd(SPI0, &cmd);
    wait_cs_released();
}

static uint8_t rdcr()
{
    uint8_t reg;

    flash_cmd_t opcode {
        .cmd = SST26_RDCR
    };

    spi_cmd_t cmd {
        .data = &reg,
        .cmd = &opcode.cmd,
        .cmd_size = 1,
        .dummy_size = 0,
        .rx = true,
        .data_size = 1
    };

    spi_select_cs(SPI0, 0);
    spi_send_cmd(SPI0, &cmd);
    wait_cs_released();

    return reg;
}

void flash_read(uint32_t address, uint8_t* data, uint32_t length)
{
    flash_cmd_t opcode {
        .cmd = SST26_HS_READ,
        .address_h = static_cast<uint8_t>((address & 0xFF0000) >> 16),
        .address_m = static_cast<uint8_t>((address & 0x00FF00) >> 8),
        .address_l = static_cast<uint8_t>((address & 0x0000FF))
    };

    spi_cmd_t cmd {
        .data = data,
        .cmd = &opcode.cmd,
        .cmd_size = 4,
        .dummy_size = 1,
        .rx = true,
        .data_size = length
    };

    spi_select_cs(SPI0, 0);
    spi_send_cmd(SPI0, &cmd);
    wait_cs_released();
}

static jedec_id_t read_id()
{
    jedec_id_t jid;

    flash_cmd_t opcode {
        .cmd = SST26_JID
    };

    spi_cmd_t cmd {
        .data = &jid.manufacturer_id,
        .cmd = &opcode.cmd,
        .cmd_size = 1,
        .dummy_size = 0,
        .rx = true,
        .data_size = 3
    };

    spi_select_cs(SPI0, 0);
    spi_send_cmd(SPI0, &cmd);
    wait_cs_released();

    return jid;
}

static void sfdp(uint32_t address, uint8_t* data, uint32_t length)
{
    flash_cmd_t opcode {
        .cmd = SST26_SFDP,
        .address_h = static_cast<uint8_t>((address & 0xFF0000) >> 16),
        .address_m = static_cast<uint8_t>((address & 0x00FF00) >> 8),
        .address_l = static_cast<uint8_t>((address & 0x0000FF))
    };

    spi_cmd_t cmd {
        .data = data,
        .cmd = &opcode.cmd,
        .cmd_size = 4,
        .dummy_size = 1,
        .rx = true,
        .data_size = length
    };

    spi_select_cs(SPI0, 0);
    spi_send_cmd(SPI0, &cmd);
    wait_cs_released();
}

/* Write instructions */
static void write_enable()
{
    if (write_enabled())
        return;
    
    flash_cmd_t opcode {
        .cmd = SST26_WREN
    };

    spi_cmd_t cmd {
        .data = nullptr,
        .cmd = &opcode.cmd,
        .cmd_size = 1,
        .dummy_size = 0,
        .rx = false,
        .data_size = 0
    };

    spi_select_cs(SPI0, 0);
    spi_send_cmd(SPI0, &cmd);
    wait_cs_released();

    wait_write_enabled();
}

static void write_disable()
{
    if (!write_enabled())
        return;
    
    flash_cmd_t opcode {
        .cmd = SST26_WRDI
    };

    spi_cmd_t cmd {
        .data = nullptr,
        .cmd = &opcode.cmd,
        .cmd_size = 1,
        .dummy_size = 0,
        .rx = false,
        .data_size = 0
    };

    spi_select_cs(SPI0, 0);
    spi_send_cmd(SPI0, &cmd);
    wait_cs_released();

    while (write_enabled());
}

static void sector_erase(uint32_t address)
{
    write_enable();

    flash_cmd_t opcode {
        .cmd = SST26_SE,
        .address_h = static_cast<uint8_t>((address >> 16) & 0xFF),
        .address_m = static_cast<uint8_t>((address >> 8) & 0xFF),
        .address_l = static_cast<uint8_t>(address & 0xFF)
    };

    spi_cmd_t cmd {
        .data = nullptr,
        .cmd = &opcode.cmd,
        .cmd_size = 4,
        .dummy_size = 0,
        .rx = false,
        .data_size = 0
    };

    spi_select_cs(SPI0, 0);
    spi_send_cmd(SPI0, &cmd);
    wait_cs_released();

    wait_write_completed();
}

static void block_erase(uint32_t address)
{
    write_enable();

    flash_cmd_t opcode {
        .cmd = SST26_BE,
        .address_h = static_cast<uint8_t>((address & 0xFF0000) >> 16),
        .address_m = static_cast<uint8_t>((address & 0x00FF00) >> 8),
        .address_l = static_cast<uint8_t>((address & 0x0000FF))
    };

    spi_cmd_t cmd {
        .data = nullptr,
        .cmd = &opcode.cmd,
        .cmd_size = 4,
        .dummy_size = 0,
        .rx = false,
        .data_size = 0
    };

    spi_select_cs(SPI0, 0);
    spi_send_cmd(SPI0, &cmd);
    wait_cs_released();

    wait_write_completed();
}

static void chip_erase()
{
    write_enable();

    flash_cmd_t opcode {
        .cmd = SST26_CE
    };

    spi_cmd_t cmd {
        .data = nullptr,
        .cmd = &opcode.cmd,
        .cmd_size = 1,
        .dummy_size = 0,
        .rx = false,
        .data_size = 0
    };

    spi_select_cs(SPI0, 0);
    spi_send_cmd(SPI0, &cmd);
    wait_cs_released();

    wait_write_completed();
}

static void page_program(uint32_t address, const uint8_t* data, uint32_t length)
{
    write_enable();

    flash_cmd_t opcode {
        .cmd = SST26_PP,
        .address_h = static_cast<uint8_t>((address & 0xFF0000) >> 16),
        .address_m = static_cast<uint8_t>((address & 0x00FF00) >> 8),
        .address_l = static_cast<uint8_t>((address & 0x0000FF))
    };

    spi_cmd_const_t cmd {
        .data = data,
        .cmd = &(opcode.cmd),
        .cmd_size = 4,
        .dummy_size = 0,
        .rx = false,
        .data_size = length
    };

    spi_select_cs(SPI0, 0);
    spi_send_cmd_const(SPI0, &cmd);
    wait_cs_released();

    wait_write_completed();
}

static void wrsu()
{
    flash_cmd_t opcode {
        .cmd = SST26_WRSU
    };

    spi_cmd_t cmd {
        .data = nullptr,
        .cmd = &opcode.cmd,
        .cmd_size = 1,
        .dummy_size = 0,
        .rx = false,
        .data_size = 0
    };

    spi_select_cs(SPI0, 0);
    spi_send_cmd(SPI0, &cmd);
    wait_cs_released();
}

static void wrre()
{
    flash_cmd_t opcode {
        .cmd = SST26_WRRE
    };

    spi_cmd_t cmd {
        .data = nullptr,
        .cmd = &opcode.cmd,
        .cmd_size = 1,
        .dummy_size = 0,
        .rx = false,
        .data_size = 0
    };

    spi_select_cs(SPI0, 0);
    spi_send_cmd(SPI0, &cmd);
    wait_cs_released();
}

/* Protection instructions */
static void rbpr(uint8_t* reg)
{
    uint8_t buffer[10];

    flash_cmd_t opcode {
        .cmd = SST26_RBPR
    };

    spi_cmd_t cmd {
        .data = &buffer[0],
        .cmd = &opcode.cmd,
        .cmd_size = 1,
        .dummy_size = 0,
        .rx = true,
        .data_size = 10
    };

    spi_select_cs(SPI0, 0);
    spi_send_cmd(SPI0, &cmd);
    wait_cs_released();

    for (int i = 0; i < 10; i++)
        reg[9-i] = buffer[i];
}

static void wbpr(uint8_t* reg)
{
    uint8_t buffer[10];
    for (int i = 0; i < 10; i++)
        buffer[9-i] = reg[i];

    flash_cmd_t opcode {
        .cmd = SST26_WBPR
    };

    spi_cmd_t cmd {
        .data = &buffer[0],
        .cmd = &opcode.cmd,
        .cmd_size = 1,
        .dummy_size = 0,
        .rx = false,
        .data_size = 10
    };

    spi_select_cs(SPI0, 0);
    spi_send_cmd(SPI0, &cmd);
    wait_cs_released();
}

static void lbpr()
{
    flash_cmd_t opcode {
        .cmd = SST26_LBPR
    };

    spi_cmd_t cmd {
        .data = nullptr,
        .cmd = &opcode.cmd,
        .cmd_size = 1,
        .dummy_size = 0,
        .rx = false,
        .data_size = 0
    };

    spi_select_cs(SPI0, 0);
    spi_send_cmd(SPI0, &cmd);
    wait_cs_released();
}

static void nvwldr(uint8_t* reg)
{
    uint8_t buffer[10];
    for (int i = 0; i < 10; i++)
        buffer[9-i] = reg[i];
    
    flash_cmd_t opcode {
        .cmd = SST26_NVWLDR
    };

    spi_cmd_t cmd {
        .data = &buffer[0],
        .cmd = &opcode.cmd,
        .cmd_size = 1,
        .dummy_size = 0,
        .rx = false,
        .data_size = 10
    };

    spi_select_cs(SPI0, 0);
    spi_send_cmd(SPI0, &cmd);
    wait_cs_released();
}

static void global_unlock()
{
    flash_cmd_t opcode {
        .cmd = SST26_ULBPR
    };

    spi_cmd_t cmd {
        .data = nullptr,
        .cmd = &opcode.cmd,
        .cmd_size = 1,
        .dummy_size = 0,
        .rx = false,
        .data_size = 0
    };

    spi_select_cs(SPI0, 0);
    spi_send_cmd(SPI0, &cmd);
    wait_cs_released();
}

static void rsid(uint16_t address, uint8_t* id, uint32_t length)
{
    flash_cmd_t opcode {
        .cmd = SST26_RSID,
        .address_h = static_cast<uint8_t>((address & 0xFF00) >> 8),
        .address_m = static_cast<uint8_t>((address & 0xFF))
    };

    spi_cmd_t cmd {
        .data = id,
        .cmd = &opcode.cmd,
        .cmd_size = 3,
        .dummy_size = 1,
        .rx = true,
        .data_size = length
    };

    spi_select_cs(SPI0, 0);
    spi_send_cmd(SPI0, &cmd);
    wait_cs_released();
}

static void psid(uint16_t address, uint8_t* id, uint32_t length)
{
    flash_cmd_t opcode {
        .cmd = SST26_PSID,
        .address_h = static_cast<uint8_t>((address & 0xFF00) >> 8),
        .address_m = static_cast<uint8_t>((address & 0xFF))
    };

    spi_cmd_t cmd {
        .data = id,
        .cmd = &opcode.cmd,
        .cmd_size = 3,
        .dummy_size = 0,
        .rx = false,
        .data_size = length
    };

    spi_select_cs(SPI0, 0);
    spi_send_cmd(SPI0, &cmd);
    wait_cs_released();
}

static void lsid()
{
    flash_cmd_t opcode {
        .cmd = SST26_LSID
    };

    spi_cmd_t cmd {
        .data = nullptr,
        .cmd = &opcode.cmd,
        .cmd_size = 1,
        .dummy_size = 0,
        .rx = false,
        .data_size = 0
    };

    spi_select_cs(SPI0, 0);
    spi_send_cmd(SPI0, &cmd);
    wait_cs_released();
}