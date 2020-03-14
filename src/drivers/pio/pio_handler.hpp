#pragma once

#include <cstdint>

void pio_handler_process(Pio *p_pio, uint32_t ul_id);
void pio_handler_set_priority(Pio *p_pio, IRQn_Type ul_irqn, uint32_t ul_priority);
uint32_t pio_handler_set(Pio *p_pio, uint32_t ul_id, uint32_t ul_mask,
        uint32_t ul_attr, void (*p_handler) (uint32_t, uint32_t));
uint32_t pio_handler_set_pin(uint32_t ul_pin, uint32_t ul_flag,
        void (*p_handler) (uint32_t, uint32_t));