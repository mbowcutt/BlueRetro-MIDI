#ifndef _MIDI_H_
#define _MIDI_H_

#include <stdbool.h>
#include <stdint.h>

void midi_init(void);

void midi_parse_ble_packet(uint8_t * data, uint32_t len);

void midi_set_notify(bool enable);

uint32_t midi_out_read(uint8_t * buf, uint32_t max_len);

#endif /* _MIDI_H_ */