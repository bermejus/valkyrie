#pragma once

#include <cstdint>
#include <sam.h>

/* Timeout value (number of attempts). */
#define TWIHS_TIMEOUT			30000

/*
    * Return codes for TWIHS API.
    */
#define TWIHS_SUCCESS			0
#define TWIHS_INVALID_ARGUMENT	1
#define TWIHS_ARBITRATION_LOST	2
#define TWIHS_NO_CHIP_FOUND		3
#define	TWIHS_RECEIVE_OVERRUN	4
#define TWIHS_RECEIVE_NACK		5
#define TWIHS_SEND_OVERRUN		6
#define TWIHS_SEND_NACK			7
#define TWIHS_BUSY				8
#define TWIHS_ERROR_TIMEOUT		9

/*
    * Input parameters when initializing the TWIHS module mode.
    */
struct twihs_options_t
{
    // MCK for TWIHS.
    uint32_t master_clk;

    // The baud rate of the TWIHS bus.
    uint32_t speed;

    // The desired address.
    uint8_t chip;

    // SMBUS mode (set 1 to use SMBUS quick command, otherwise don't).
    uint8_t smbus;
};

/*
    * Information about data transmission.
    */
struct twihs_packet_t
{
    // TWIHS address/commands to issue to the other chip (node).
    uint8_t addr[3];

    // Length of the TWIHS data address segment (1-3 bytes).
    uint32_t addr_length;

    // Where to find the data to be transfered;
    void* buffer;

    //How many bytes do we want to transfer;
    uint32_t length;

    // TWIHS chip address to communicate with.
    uint8_t chip;
};

static inline void twihs_enable_smbus_quick_command(Twihs* p_twihs)
{
    p_twihs->TWIHS_CR = TWIHS_CR_QUICK;
}

/* Enable high speed mode. */
static inline void twihs_enable_highspeed(Twihs* p_twihs)
{
    p_twihs->TWIHS_CR = TWIHS_CR_HSEN;
}

/* Disable high speed mode. */
static inline void twihs_disable_highspeed(Twihs* p_twihs)
{
    p_twihs->TWIHS_CR = TWIHS_CR_HSDIS;
}

/* Enable SMBus mode. */
static inline void twihs_enable_smbus(Twihs* p_twihs)
{
    p_twihs->TWIHS_CR = TWIHS_CR_SMBEN;
}

/* Disable SMBus mode. */
static inline void twihs_disable_smbus(Twihs* p_twihs)
{
    p_twihs->TWIHS_CR = TWIHS_CR_SMBDIS;
}

/* Enable packet error checking. */
static inline void twihs_enable_pec(Twihs* p_twihs)
{
    p_twihs->TWIHS_CR = TWIHS_CR_PECEN;
}

/* Disable packet error checking. */
static inline void twihs_disable_pec(Twihs* p_twihs)
{
    p_twihs->TWIHS_CR = TWIHS_CR_PECDIS;
}

/* Request a packet error checking. */
static inline void twihs_request_pec(Twihs *p_twihs)
{
    p_twihs->TWIHS_CR = TWIHS_CR_PECRQ;
}

/* If master mode is enabled, send a bus clear command. */
static inline void twihs_send_clear(Twihs *p_twihs)
{
    p_twihs->TWIHS_CR = TWIHS_CR_CLEAR;
}

/* Normal value to be returned in the ACK cycle of the data phase in slave receiver mode. */
static inline void twihs_disable_slave_nack(Twihs *p_twihs)
{
    p_twihs->TWIHS_SMR &= ~TWIHS_SMR_NACKEN;
}

/* NACK value to be returned in the ACK cycle of the data phase in slave receiver mode. */
static inline void twihs_enable_slave_nack(Twihs *p_twihs)
{
    p_twihs->TWIHS_SMR |= TWIHS_SMR_NACKEN;
}

/* Acknowledge of the SMBus Default Address disabled. */
static inline void twihs_disable_slave_default_addr(Twihs *p_twihs)
{
    p_twihs->TWIHS_SMR &= ~TWIHS_SMR_SMDA;
}

/* Acknowledge of the SMBus Default Address disabled. */
static inline void twihs_enable_slave_default_addr(Twihs *p_twihs)
{
    p_twihs->TWIHS_SMR |= TWIHS_SMR_SMDA;
}

/* Acknowledge of the SMBus Host Header disabled. */
static inline void twihs_disable_smbus_host_header(Twihs *p_twihs)
{
    p_twihs->TWIHS_SMR &= ~TWIHS_SMR_SMHH;
}

/* Acknowledge of the SMBus Host Header enabled. */
static inline void twihs_enable_smbus_host_header(Twihs *p_twihs)
{
    p_twihs->TWIHS_SMR |= TWIHS_SMR_SMHH;
}

/* Clock stretching disabled in slave mode, OVRE and UNRE will indicate overrun and underrun. */
static inline void twihs_disable_clock_wait_state(Twihs *p_twihs)
{
    p_twihs->TWIHS_SMR |= TWIHS_SMR_SCLWSDIS;
}

/* Clear clock wait state disable mode. */
static inline void twihs_clear_disable_clock_wait_state(Twihs *p_twihs)
{
    p_twihs->TWIHS_SMR &= ~TWIHS_SMR_SCLWSDIS;
}

/* Slave Address 1 matching disabled. */
static inline void twihs_disable_slave_addr1_matching(Twihs *p_twihs)
{
    p_twihs->TWIHS_SMR &= ~TWIHS_SMR_SADR1EN;
}

/* Slave Address 1 matching enabled. */
static inline void twihs_enable_slave_addr1_matching(Twihs *p_twihs)
{
    p_twihs->TWIHS_SMR |= TWIHS_SMR_SADR1EN;
}

/* Slave Address 2 matching disabled. */
static inline void twihs_disable_slave_addr2_matching(Twihs *p_twihs)
{
    p_twihs->TWIHS_SMR &= ~TWIHS_SMR_SADR2EN;
}

/* Slave Address 2 matching enabled. */
static inline void twihs_enable_slave_addr2_matching(Twihs *p_twihs)
{
    p_twihs->TWIHS_SMR |= TWIHS_SMR_SADR2EN;
}

/* Slave Address 3 matching disabled. */
static inline void twihs_disable_slave_addr3_matching(Twihs *p_twihs)
{
    p_twihs->TWIHS_SMR &= ~TWIHS_SMR_SADR3EN;
}

/* Slave Address 3 matching enabled. */
static inline void twihs_enable_slave_addr3_matching(Twihs *p_twihs)
{
    p_twihs->TWIHS_SMR |= TWIHS_SMR_SADR3EN;
}

/* First received data matching disabled. */
static inline void twihs_disable_slave_data_matching(Twihs *p_twihs)
{
    p_twihs->TWIHS_SMR &= ~TWIHS_SMR_DATAMEN;
}

/* First received data matching enabled. */
static inline void twihs_enable_slave_data_matching(Twihs *p_twihs)
{
    p_twihs->TWIHS_SMR |= TWIHS_SMR_DATAMEN;
}

void twihs_twihs_set_sleepwalking(Twihs* p_twihs,
    uint32_t ul_matching_addr1, bool flag1,
    uint32_t ul_matching_addr2, bool flag2,
    uint32_t ul_matching_addr3, bool flag3,
    uint32_t ul_matching_data, bool flag);

void twihs_enable_master_mode(Twihs* p_twihs);
void twihs_disable_master_mode(Twihs* p_twihs);
uint32_t twihs_master_init(Twihs* p_twihs, const twihs_options_t* p_opt);
uint32_t twihs_set_speed(Twihs* p_twihs, uint32_t ul_speed, uint32_t ul_mck);
uint32_t twihs_probe(Twihs* p_twihs, uint8_t uc_slave_addr);
uint32_t twihs_twihs_master_read(Twihs* p_twihs, twihs_packet_t* p_packet);
uint32_t twihs_master_write(Twihs* p_twihs, twihs_packet_t* p_packet);
void twihs_enable_interrupt(Twihs* p_twihs, uint32_t ul_sources);
void twihs_disable_interrupt(Twihs* p_twihs, uint32_t ul_sources);
uint32_t twihs_get_interrupt_status(Twihs* p_twihs);
uint32_t twihs_get_interrupt_mask(Twihs* p_twihs);
uint8_t twihs_read_byte(Twihs* p_twihs);
void twihs_write_byte(Twihs* p_twihs, uint8_t uc_byte);
void twihs_enable_slave_mode(Twihs* p_twihs);
void twihs_disable_slave_mode(Twihs* p_twihs);
void twihs_slave_init(Twihs* p_twihs, uint32_t ul_device_addr);
void twihs_set_slave_addr(Twihs* p_twihs, uint32_t ul_device_addr);
uint32_t twihs_slave_read(Twihs* p_twihs, uint8_t* p_data);
uint32_t twihs_slave_write(Twihs* p_twihs, uint8_t* p_data);
void twihs_reset(Twihs* p_twihs);
void twihs_set_write_protection(Twihs* p_twihs, bool flag);
void twihs_read_write_protection_status(Twihs* p_twihs, uint32_t* p_status);
void twihs_smbus_set_timing(Twihs* p_twihs, uint32_t ul_timing);
void twihs_set_filter(Twihs* p_twihs, uint32_t ul_filter);
void twihs_mask_slave_addr(Twihs* p_twihs, uint32_t ul_mask);

bool twihs_init(uint32_t speed);
bool twihs_write(uint8_t chip_addr, uint8_t mem_addr, uint8_t* data, uint16_t length);
bool twihs_read(uint8_t chip_addr, uint8_t mem_addr, uint8_t* data, uint16_t length);