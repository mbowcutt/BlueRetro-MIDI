#include "midi.h"

#include <stdatomic.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "esp_attr.h"

#include "bluetooth/att_cfg.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define MIDI_BUF_SIZE 256u
#define MIDI_BUF_MASK (MIDI_BUF_SIZE - 1u)

// Channel Voice Messages
#define MIDI_NOTE_OFF                           0x80
#define MIDI_NOTE_ON                            0x90
#define MIDI_POLY_KEY_PRESSURE                  0xA0
#define MIDI_CONTROL_CHANGE                     0xB0
#define MIDI_PROGRAM_CHANGE                     0xC0
#define MIDI_CHANNEL_PRESSURE                   0xD0
#define MIDI_PITCH_BEND                         0xE0

// Channel Mode Messages
#define MIDI_MODE_ALL_SOUND_OFF                 0x78
#define MIDI_MODE_RESET_ALL_CONTROLLERS         0x79
#define MIDI_MODE_LOCAL_CONTROL                 0x7A
#define MIDI_MODE_ALL_NOTES_OFF                 0x7B
#define MIDI_MODE_OMNI_OFF                      0x7C
#define MIDI_MODE_OMNI_ON                       0x7D
#define MIDI_MODE_POLY_OFF                      0x7E
#define MIDI_MODE_POLY_ON                       0x7F

// System Exclusive & System Common Messages
#define MIDI_SYS_EX                             0xF0
#define MIDI_SYS_COM_MIDI_TIME_CODE_QTR_FRAME   0xF1
#define MIDI_SYS_COM_SONG_POSITION_POINTER      0xF2
#define MIDI_SYS_COM_SONG_SELECT                0xF3
#define MIDI_SYS_COM_RSVD1                      0xF4
#define MIDI_SYS_COM_RSVD2                      0xF5
#define MIDI_SYS_COM_TUNE_REQUEST               0xF6
#define MIDI_SYS_EOX                            0xF7

// System Real Time Messages
#define MIDI_SYS_RT_TIMING_CLOCK                0xF8
#define MIDI_SYS_RT_RSVD1                       0xF9
#define MIDI_SYS_RT_START                       0xFA
#define MIDI_SYS_RT_CONTINUE                    0xFB
#define MIDI_SYS_RT_STOP                        0XFC
#define MIDI_SYS_RT_RSVD2                       0xFD
#define MIDI_SYS_RT_ACTIVE_SENSING              0xFE
#define MIDI_SYS_RT_SYSTEM_RESET                0xFF


typedef struct
{
    uint8_t data[MIDI_BUF_SIZE];
    atomic_uint head;
    atomic_uint tail;
} midi_ringbuf_t;


static void midi_in_task(void * param);
static uint8_t get_num_data_bytes(uint8_t status_byte);
static inline void midi_buf_write_byte(midi_ringbuf_t * buf, uint8_t byte);
static uint32_t encode_ble_packet(uint8_t * in_buffer, uint32_t in_size,
                                  uint8_t * out_pkt_buffer, uint32_t out_pkt_size,
                                  uint8_t * rewrite_buffer, uint32_t rewrite_buffer_size,
                                  uint32_t * rewrite_buffer_size_actual);


static midi_ringbuf_t midi_out;
static midi_ringbuf_t midi_in;
static bool notify_en = false;

static inline void IRAM_ATTR midi_buf_write_byte(midi_ringbuf_t * buf, uint8_t byte)
{
    uint32_t head = atomic_load(&buf->head);
    uint32_t next = (head + 1) & MIDI_BUF_MASK;
    if (next != atomic_load(&buf->tail))
    {
        buf->data[head] = byte;
        atomic_store(&buf->head, next);
    }
}

static void midi_in_task(void * param)
{
    static uint8_t in_buffer[32] = {0};
    static uint8_t midi_tx_buffer[64] = {0};
    static uint8_t rewrite_buffer[32] = {0};

    while (1)
    {
        uint32_t in_size = midi_in_read(in_buffer, 23);
        if ((in_size > 0) && notify_en)
        {
            printf("[BLE-MIDI] IN: ");
            for (uint8_t idx = 0; idx < in_size; ++idx)
            {
                printf("%02X", in_buffer[idx]);
            }
            printf("\n");

            uint32_t rewrite_buffer_size = 0;
            uint32_t pkt_size = encode_ble_packet(in_buffer, in_size,
                                                  midi_tx_buffer, bt_att_cfg_get_mtu_size(),
                                                  rewrite_buffer, sizeof(rewrite_buffer),
                                                  &rewrite_buffer_size);
            printf("[BLE-MIDI] BLE TX: ");
            for (uint8_t idx = 0; idx < in_size; ++idx)
            {
                printf("%02X", midi_tx_buffer[idx]);
            }
            printf("\n");

            bt_att_cfg_midi_tx(midi_tx_buffer, pkt_size);

            // TODO: handle rewrite buffer
        }

        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

static uint32_t encode_ble_packet(uint8_t * in_buffer, uint32_t in_size,
                                  uint8_t * out_pkt_buffer, uint32_t out_pkt_size,
                                  uint8_t * rewrite_buffer, uint32_t rewrite_buffer_size,
                                  uint32_t * rewrite_buffer_size_actual)
{
    size_t midi_tx_size = 0;
    bool header_written = false;

    for (uint32_t idx = 0; idx < in_size; ++idx)
    {
        uint8_t status = in_buffer[idx++];

        // Got Status byte
        if (status & 0x80)
        {
            uint8_t num_data_bytes = get_num_data_bytes(status);

            if ((midi_tx_size
                 + 1 /* Status byte */
                 + num_data_bytes
                 + (header_written ? 0 : 1)
                 + 1 /* Timestamp */ <= out_pkt_size)   // Enough room left in packet (not including unexpected/RT bytes)
                && ((idx + num_data_bytes) <= in_size)) // Enough room left in input buffer
            {
                uint16_t ts_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

                // Write packet header if not written yet
                if (!header_written)
                {
                    out_pkt_buffer[midi_tx_size++] = (0x80 | ((ts_ms & 0x1F80) >> 7));
                    header_written = true;
                }

                // Write timestamp and Status byte
                out_pkt_buffer[midi_tx_size++] = (0x80 | (ts_ms & 0x7F));
                out_pkt_buffer[midi_tx_size++] = status;

                // Write Data bytes
                uint8_t num_rt_bytes = 0;
                while (num_data_bytes > 0)
                {
                    uint8_t byte = in_buffer[idx++];

                    // TODO: de-interleave System Realtime byte
                    if (byte >= MIDI_SYS_RT_TIMING_CLOCK)
                    {
                        // Check packet has 2 extra bytes for overhead
                        if (midi_tx_size + num_data_bytes + 2 <= out_pkt_size)
                        {
                            out_pkt_buffer[midi_tx_size + num_data_bytes + num_rt_bytes++]
                                = (0x80 | (ts_ms & 0x7F));
                            out_pkt_buffer[midi_tx_size + num_data_bytes + num_rt_bytes++]
                                = byte;
                        }
                        else
                        {
                            // TODO: Send to rewrite buffer
                        }

                        // TODO: Check input buffer has additional byte
                    }
                    else if (0x80 & byte)
                    {
                        // This shouldn't be a Status byte...
                        // TODO: Discard previous bytes & set new status
                    }
                    else
                    {
                        // Got Data byte
                        out_pkt_buffer[midi_tx_size++] = byte;
                        num_data_bytes -= 1;
                    }
                }

                // Add Realtime overhead
                midi_tx_size += num_rt_bytes;
            }

            // TODO: Reinsert bytes back at head of buffer
        }
    }

    return midi_tx_size;
}

static uint8_t get_num_data_bytes(uint8_t status_byte)
{
    if (0 == (0x80 & status_byte))
    {
        // Data Byte, not valid.
        return 0;
    }
    else if ((MIDI_SYS_COM_RSVD1 == status_byte)
             || (MIDI_SYS_COM_RSVD2 == status_byte)
             || (MIDI_SYS_RT_RSVD1 == status_byte)
             || (MIDI_SYS_RT_RSVD2 == status_byte))
    {
        // Byte is undefined, not valid.
        return 0;
    }
    else if ((MIDI_SYS_EX == status_byte)
             || (MIDI_SYS_EOX == status_byte))
    {
        /// TODO: SysEx not yet implemented.
        return 0;
    }
    else if (MIDI_SYS_COM_TUNE_REQUEST <= status_byte)
    {
        return 0;
    }
    else if ((MIDI_SYS_COM_MIDI_TIME_CODE_QTR_FRAME == status_byte)
             || (MIDI_SYS_COM_SONG_SELECT == status_byte)
             || ((status_byte & 0xF0) == MIDI_PROGRAM_CHANGE)
             || ((status_byte & 0xF0) == MIDI_CHANNEL_PRESSURE))
    {
        return 1;
    }
    else
    {
        return 2;
    }
}

bool midi_init(void)
{
    memset(midi_out.data, 0x00, MIDI_BUF_SIZE);
    atomic_store(&midi_out.head, 0);
    atomic_store(&midi_out.tail, 0);

    memset(midi_in.data, 0x00, MIDI_BUF_SIZE);
    atomic_store(&midi_in.head, 0);
    atomic_store(&midi_in.tail, 0);

    xTaskCreatePinnedToCore(&midi_in_task, "midi_in_task", 2048, NULL, 5, NULL, 0);

    return true;
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

uint32_t IRAM_ATTR midi_in_read(uint8_t * buf, uint32_t max_len)
{
    uint32_t count = 0;
    uint32_t head = atomic_load(&midi_in.head);
    uint32_t tail = atomic_load(&midi_in.tail);

    while ((count < max_len) && (head != tail))
    {
        buf[count++] = midi_in.data[tail];
        tail = (tail + 1) & MIDI_BUF_MASK;
    }

    atomic_store(&midi_in.tail, tail);

    return count;
}

void IRAM_ATTR midi_in_write(uint8_t * buf, uint32_t len)
{
    for (uint32_t idx = 0; idx < len; ++idx)
    {
        midi_buf_write_byte(&midi_in, buf[idx]);
    }
}
