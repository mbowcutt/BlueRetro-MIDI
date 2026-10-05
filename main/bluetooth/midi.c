#include "midi.h"

#include <stdatomic.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "esp_attr.h"

#define MIDI_BUF_SIZE 256u
#define MIDI_BUF_MASK (MIDI_BUF_SIZE - 1u)

// Write to tail, read from head
typedef struct
{
    uint8_t data[MIDI_BUF_SIZE];
    atomic_uint head;
    atomic_uint tail;
} midi_ringbuf_t;

static midi_ringbuf_t midi_out;

static bool notify_en = false;

static inline void midi_buf_write_byte(midi_ringbuf_t * buf, uint8_t byte)
{
    uint32_t head = atomic_load(&buf->head);
    uint32_t next = (head + 1) & MIDI_BUF_MASK;
    if (next != atomic_load(&buf->tail))
    {
        buf->data[head] = byte;
        atomic_store(&buf->head, next);
    }
}

void midi_init(void)
{
    memset(midi_out.data, 0x00, MIDI_BUF_SIZE);
    atomic_store(&midi_out.head, 0);
    atomic_store(&midi_out.tail, 0);
}

void midi_set_notify(bool enable)
{
    notify_en = enable;
}

void midi_parse_ble_packet(uint8_t * data, uint32_t len)
{
    printf("[BLE-MIDI] Write: ");
    for (uint32_t idx = 0; idx < len; ++idx)
    {
        printf("%X", data[idx]);
    }
    printf("\n");

    // Discard the Header Byte, then discard only Timestamp Bytes.
    // Only MIDI message Status and Data bytes are written to the buffer.
    bool expect_timestamp = true;
    for (uint32_t idx = 1; idx < len; ++idx)
    {
        uint8_t byte = data[idx];

        if (byte & 0x80) // Status Byte or Timestamp
        {
            if (expect_timestamp)
            {
                expect_timestamp = false;
            }
            else
            {
                // Write the Status Byte
                midi_buf_write_byte(&midi_out, byte);

                // If the message has 0 Data Bytes, expect a Timestamp next.
                if (byte >= 0xF6)
                {
                    expect_timestamp = true;
                }
            }
        }
        else
        {
            // Always write the Data Byte
            midi_buf_write_byte(&midi_out, byte);
            expect_timestamp = true;
        }
    }
}

uint32_t IRAM_ATTR midi_out_read(uint8_t * buf, uint32_t max_len)
{
    uint32_t count = 0;
    uint32_t head = atomic_load(&midi_out.head);
    uint32_t tail = atomic_load(&midi_out.tail);

    while ((count < max_len) && (head != tail))
    {
        buf[count++] = midi_out.data[tail];
        tail = (tail + 1) & MIDI_BUF_MASK;
    }

    atomic_store(&midi_out.tail, tail);

    return count;
}
