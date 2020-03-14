#pragma once

#include <cstdint>
#include <sam.h>

void tc_init(Tc* p_tc, uint32_t ul_channel, uint32_t ul_mode);
void tc_sync_trigger(Tc* p_tc);
void tc_set_block_mode(Tc* p_tc, uint32_t ul_blockmode);

uint32_t tc_init_2bit_gray(Tc* p_tc, uint32_t ul_channel, uint32_t ul_steppermode);

void tc_start(Tc* p_tc, uint32_t ul_channel);
void tc_stop(Tc* p_tc, uint32_t ul_channel);

uint32_t tc_read_cv(Tc* p_tc, uint32_t ul_channel);
uint32_t tc_read_ra(Tc* p_tc, uint32_t ul_channel);
uint32_t tc_read_rb(Tc* p_tc, uint32_t ul_channel);
uint32_t tc_read_rc(Tc* p_tc, uint32_t ul_channel);

void tc_write_ra(Tc* p_tc, uint32_t ul_channel, uint32_t ul_value);
void tc_write_rb(Tc* p_tc, uint32_t ul_channel, uint32_t ul_value);
void tc_write_rc(Tc* p_tc, uint32_t ul_channel, uint32_t ul_value);

uint32_t tc_find_mck_divisor(uint32_t ul_freq, uint32_t ul_mck, uint32_t* p_uldiv, uint32_t* ul_tcclks, uint32_t ul_boardmck);
void tc_enable_interrupt(Tc* p_tc, uint32_t ul_channel, uint32_t ul_sources);
void tc_disable_interrupt(Tc* p_tc, uint32_t ul_channel, uint32_t ul_sources);
uint32_t tc_get_interrupt_mask(Tc* p_tc, uint32_t ul_channel);
uint32_t tc_get_status(Tc* p_tc, uint32_t ul_channel);

void tc_enable_qdec_interrupt(Tc* p_tc, uint32_t ul_sources);
void tc_disable_qdec_interrupt(Tc* p_tc, uint32_t ul_sources);
uint32_t tc_get_qdec_interrupt_mask(Tc* p_tc);
uint32_t tc_get_qdec_interrupt_status(Tc* p_tc);

void tc_set_writeprotect(Tc* p_tc, bool enable);