#pragma once

#include <cstdint>
#include "sd_mmc_protocol.hpp"

/* Return error codes */
#define SD_MMC_OK 0           /* No error */
#define SD_MMC_INIT_ONGOING 1 /* Card not initialized */
#define SD_MMC_ERR_NO_CARD 2  /* No SD/MMC card inserted */
#define SD_MMC_ERR_UNUSABLE 3 /* Unusable card */
#define SD_MMC_ERR_SLOT 4     /* Slot unknow */
#define SD_MMC_ERR_COMM 5     /* General communication error */
#define SD_MMC_ERR_PARAM 6    /* Illeage input parameter */
#define SD_MMC_ERR_WP 7       /* Card write protected */

/* This SD MMC stack uses the maximum block size autorized (512 bytes) */
#define SD_MMC_BLOCK_SIZE 512

typedef uint8_t sd_mmc_err_t; /* Type of return error code */
typedef uint8_t card_type_t;
typedef uint8_t card_version_t;

/* Card detect settings */
struct sd_mmc_detect_t {
    int16_t  pin; /**< Detection pin, -1 if no such pin */
    uint16_t val; /**< Detection value */
};

/* Initialize the SD/MMC stack and low level driver required */
void sd_mmc_init();

/* Return the number of slots available. */
uint8_t sd_mmc_nb_slot();

/*
    * Performs a card check.
    * 
    * \retval SD_MMC_OK           Card ready
    * \retval SD_MMC_INIT_ONGOING Initialization on going
    * \retval SD_MMC_ERR_NO_CARD  Card not present in slot
    * \retval Other value for error cases, see \ref sd_mmc_err_t
    */
sd_mmc_err_t sd_mmc_check(uint8_t slot);

/* Get the card type */
card_type_t sd_mmc_get_type(uint8_t slot);

/* Get the card version */
card_version_t sd_mmc_get_version(uint8_t slot);

/* Get the memory capacity (KB) */
uint32_t sd_mmc_get_capacity(uint8_t slot);

/* Get the card write protection status */
bool sd_mmc_is_write_protected(uint8_t slot);

/*
    * Initialize the read blocks of data from the card.
    *
    * \param[in] slot     Card slot to use
    * \param[in] start    Start block number to to read.
    * \param[in] nb_block Total number of blocks to be read.
    *
    * \return return SD_MMC_OK if success, otherwise return an error code (\ref sd_mmc_err_t).
    */
sd_mmc_err_t sd_mmc_init_read_blocks(uint8_t slot, uint32_t start, uint16_t nb_block);

/*
    * Start the read blocks of data from the card.
    *
    * \param[out] dest     Pointer to read buffer.
    * \param[in]  nb_block Number of blocks to be read.
    *
    * \return return SD_MMC_OK if started, otherwise return an error code (\ref sd_mmc_err_t).
    */
sd_mmc_err_t sd_mmc_start_read_blocks(void* dest, uint16_t nb_block);

/*
    * Wait the end of read blocks of data from the card.
    *
    * \param[in] abort Abort reading process initialized by
    *              \ref sd_mmc_init_read_blocks() after the reading issued by
    *              \ref sd_mmc_start_read_blocks() is done
    *
    * \return return SD_MMC_OK if success, otherwise return an error code (\ref sd_mmc_err_t).
    */
sd_mmc_err_t sd_mmc_wait_end_of_read_blocks(bool abort);

/*
    * Initialize the write blocks of data
    *
    * \param[in] slot     Card slot to use
    * \param[in] start    Start block number to be written.
    * \param[in] nb_block Total number of blocks to be written.
    *
    * \return return SD_MMC_OK if success, otherwise return an error code (\ref sd_mmc_err_t).
    */
sd_mmc_err_t sd_mmc_init_write_blocks(uint8_t slot, uint32_t start, uint16_t nb_block);

/*
    * Start the write blocks of data
    *
    * \param[in] src      Pointer to write buffer.
    * \param[in] nb_block Number of blocks to be written.
    *
    * \return return SD_MMC_OK if started, otherwise return an error code (\ref sd_mmc_err_t).
    */
sd_mmc_err_t sd_mmc_start_write_blocks(const void* src, uint16_t nb_block);

/*
    * Wait the end of write blocks of data
    *
    * \param[in] abort Abort writing process initialized by
    *              \ref sd_mmc_init_write_blocks() after the writing issued by
    *              \ref sd_mmc_start_write_blocks() is done
    *
    * \return return SD_MMC_OK if success, otherwise return an error code (\ref sd_mmc_err_t).
    */
sd_mmc_err_t sd_mmc_wait_end_of_write_blocks(bool abort);