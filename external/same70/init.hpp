#ifndef INIT_H_
#define INIT_H_

#include <drivers/ioport/ioport.hpp>

void ioport_set_port_peripheral_mode(ioport_port_t, ioport_port_mask_t, ioport_mode_t);
void ioport_set_pin_peripheral_mode(ioport_pin_t, ioport_mode_t);
void ioport_set_pin_input_mode(ioport_pin_t, ioport_mode_t, ioport_sense);

void board_init();

#endif /* INIT_H_ */