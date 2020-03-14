#pragma once

#include <cstdint>
#include "../sdcard/sd_mmc_protocol.hpp"

/* This enable the clock required and the hardware interface. */
void hsmci_init();

/* Return the maximum bus width of a slot. */
uint8_t hsmci_get_bus_width(uint8_t slot);

/* Return the high speed capability of the driver. */
bool hsmci_is_high_speed_capable();

/* Select a slot and initialize it. */
void hsmci_select_device(uint8_t slot, uint32_t clock, uint8_t bus_width, bool high_speed);

/* Deselect a slot. */
void hsmci_deselect_device(uint8_t slot);

/*
    * Send 74 clock cycles on the line of selected slot.
    * Note: It is required after card plug and before card install.
    */
void hsmci_send_clock();

/* Send a command on the selected slot. */
bool hsmci_send_cmd(sdmmc_cmd_def_t cmd, uint32_t arg);

/* Return the 32 bits response of the last command. */
uint32_t hsmci_get_response();

/* Return the 128 bits response of the last command. */
void hsmci_get_response_128(uint8_t* response);

/*
    * Send an ADTC command on the selected slot
    * An ADTC (Addressed Data Transfer Commands) command is used
    * for read/write access.
    *
    * \param cmd          Command definition
    * \param arg          Argument of the command
    * \param block_size   Block size used for the transfer
    * \param nb_block     Total number of block for this transfer
    * \param access_block if true, the x_read_blocks() and x_write_blocks()
    * functions must be used after this function.
    * If false, the mci_read_word() and mci_write_word()
    * functions must be used after this function.
    *
    * return true if success, otherwise false
    */
bool hsmci_adtc_start(sdmmc_cmd_def_t cmd, uint32_t arg, uint16_t block_size, uint16_t nb_block, bool access_block);

/* Send a command to stop an ADTC command on the selected slot
    *
    * \param cmd        Command definition
    * \param arg        Argument of the command
    *
    * return true if success, otherwise false
    */
bool hsmci_adtc_stop(sdmmc_cmd_def_t cmd, uint32_t arg);

/* Read a word on the line. */
bool hsmci_read_word(uint32_t* value);

/* Write a word on the line. */
bool hsmci_write_word(uint32_t word);

/*
    * Start a read blocks transfer on the line
    * Note: The driver will use the DMA available to speed up the transfer.
    *
    * \param dest       Pointer on the buffer to fill
    * \param nb_block   Number of block to transfer
    *
    * return true if started, otherwise false
    */
bool hsmci_start_read_blocks(void* dest, uint16_t nb_block);

/* Wait the end of transfer initiated by mci_start_read_blocks(). */
bool hsmci_wait_end_of_read_blocks();

/*
    * Start a write blocks transfer on the line.
    * Note: The driver will use the DMA available to speed up the transfer.
    *
    * \param src        Pointer on the buffer to send
    * \param nb_block   Number of block to transfer
    *
    * return true if started, otherwise false
    */
bool hsmci_start_write_blocks(const void* src, uint16_t nb_block);

/* Wait the end of transfer initiated by mci_start_write_blocks() */
bool hsmci_wait_end_of_write_blocks();