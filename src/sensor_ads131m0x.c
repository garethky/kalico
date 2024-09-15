// Support for ADS131M02 and ADS131M04 ADC chips
//
// Copyright (C) 2025
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "board/irq.h" // irq_disable
#include "board/gpio.h" // gpio_out_write
#include "board/misc.h" // timer_read_time
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "sched.h" // sched_add_timer
#include "sensor_bulk.h" // sensor_bulk_report
#include "load_cell_probe.h" // load_cell_probe_report_sample
#include "spicmds.h" // spidev_transfer
#include <stdint.h>

struct ads131m0x_model {
    uint8_t channel_count;
    uint8_t sample_bytes;
    uint8_t frame_size;
    uint8_t crc_data_bytes;
    uint8_t crc_offset;
};

struct ads131m0x_adc {
    struct timer timer;
    uint32_t rest_ticks;
    uint32_t hard_error_latch;
    struct gpio_in data_ready;
    struct spidev_s *spi;
    uint8_t pending_flag;
    uint8_t channel_mask;
    uint8_t output_channel_count;
    struct ads131m0x_model model;
    struct sensor_bulk sb;
    struct load_cell_probe *lce;
};

// Error codes sent as sample values (use high bits to distinguish from valid data)
#define SAMPLE_ERROR_CRC       (1L << 31)
#define SAMPLE_ERROR_RESET     (1L << 30)

#define BYTES_PER_SAMPLE 4
#define BYTES_PER_WORD 3
#define MAX_ADC_CHANNELS 4
#define MAX_FRAME_SIZE 18
#define STATUS_RESET_BIT (1 << 2)  // Bit 10 of 16-bit status, in byte 0 (bits 15:8)
#define CRC_INITIAL 0xFFFF
#define CRC_POLY 0x1021  // CCITT polynomial: x^16 + x^12 + x^5 + 1

static struct task_wake wake_ads131m0x;

// Add a measurement to the buffer
static inline void
buffer_append_int32(struct sensor_bulk *sb, int32_t val)
{
    sb->data[sb->data_count] = val;
    sb->data[sb->data_count + 1] = val >> 8;
    sb->data[sb->data_count + 2] = val >> 16;
    sb->data[sb->data_count + 3] = val >> 24;
    sb->data_count += BYTES_PER_SAMPLE;
}

static inline void
ads131m0x_flush(struct ads131m0x_adc *adc, uint8_t oid)
{
    if (adc->sb.data_count > ARRAY_SIZE(adc->sb.data)
            - adc->model.sample_bytes) {
        sensor_bulk_report(&adc->sb, oid);
    }
}

/****************************************************************
 * ads131m0x sensor support
 ****************************************************************/

// Calculate CCITT CRC-16 over data bytes
static uint16_t
calc_crc16(uint8_t *data, uint8_t len)
{
    uint16_t crc = CRC_INITIAL;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ CRC_POLY;
            else
                crc <<= 1;
        }
    }
    return crc;
}

static inline uint8_t
ads131m0x_is_data_ready(struct ads131m0x_adc *adc) {
    return gpio_in_read(adc->data_ready) == 0;
}

static inline int32_t
ads131m0x_extract_counts(uint8_t *msg, uint8_t offset)
{
    uint32_t counts = ((uint32_t)msg[offset] << 16)
                    | ((uint32_t)msg[offset + 1] << 8)
                    | ((uint32_t)msg[offset + 2]);
    if (counts & 0x800000)
        counts |= 0xFF000000;
    return (int32_t)counts;
}

static inline uint8_t
ads131m0x_has_crc_error(struct ads131m0x_adc *adc, uint8_t *msg)
{
    uint16_t calc_crc = calc_crc16(msg, adc->model.crc_data_bytes);
    uint8_t crc_offset = adc->model.crc_offset;
    uint16_t recv_crc = ((uint16_t)msg[crc_offset] << 8) | msg[crc_offset + 1];
    return calc_crc != recv_crc;
}

static void
ads131m0x_publish_samples(struct ads131m0x_adc *adc, uint8_t oid
                        , uint8_t *msg)
{
    int32_t sum = 0;
    uint8_t i;
    for (i = 0; i < adc->model.channel_count; i++) {
        if (adc->channel_mask & (1 << i)) {
            uint8_t offset = BYTES_PER_WORD + i * BYTES_PER_WORD;
            int32_t val = ads131m0x_extract_counts(msg, offset);
            sum += val;
            buffer_append_int32(&adc->sb, val);
        }
    }
    if (adc->lce)
        load_cell_probe_report_sample(adc->lce, sum);
    ads131m0x_flush(adc, oid);
}

// Event handler that wakes wake_ads131m0x() periodically
static uint_fast8_t
ads131m0x_event(struct timer *timer)
{
    struct ads131m0x_adc *adc = container_of(timer, struct ads131m0x_adc,
                                              timer);
    uint32_t rest_ticks = adc->rest_ticks;
    if (adc->pending_flag) {
        adc->sb.possible_overflows++;
        rest_ticks *= 4;
    } else if (ads131m0x_is_data_ready(adc)) {
        adc->pending_flag = 1;
        sched_wake_task(&wake_ads131m0x);
        rest_ticks *= 8;
    }
    adc->timer.waketime += rest_ticks;
    return SF_RESCHEDULE;
}

static void
ads131m0x_publish_error(struct ads131m0x_adc *adc, uint8_t oid
                    , int32_t sample_error)
{
    for (uint8_t i = 0; i < adc->output_channel_count; i++) {
        buffer_append_int32(&adc->sb, sample_error);
    }
    ads131m0x_flush(adc, oid);
}

static void
ads131m0x_read_adc(struct ads131m0x_adc *adc, uint8_t oid)
{
    uint8_t msg[MAX_FRAME_SIZE] = {0};
    spidev_transfer(adc->spi, 1, adc->model.frame_size, msg);
    adc->pending_flag = 0;
    barrier();

    // Check for unexpected device reset (RESET bit in status byte 0)
    // This is a hard error - once set, keep sending until measurement restart
    if (msg[0] & STATUS_RESET_BIT)
        adc->hard_error_latch = SAMPLE_ERROR_RESET;
    if (adc->hard_error_latch) {
        ads131m0x_publish_error(adc, oid, adc->hard_error_latch);
    }
    else if (ads131m0x_has_crc_error(adc, msg)) {
        ads131m0x_publish_error(adc, oid, SAMPLE_ERROR_CRC);
    }
    else {
        ads131m0x_publish_samples(adc, oid, msg);
    }
}

void
command_config_ads131m0x(uint32_t *args)
{
    struct ads131m0x_adc *adc = oid_alloc(args[0], command_config_ads131m0x
                                         , sizeof(*adc));
    adc->timer.func = ads131m0x_event;
    adc->pending_flag = 0;
    adc->spi = spidev_oid_lookup(args[1]);
    adc->data_ready = gpio_in_setup(args[2], 0);
    adc->model.channel_count = args[3];
    if (!adc->model.channel_count || adc->model.channel_count > MAX_ADC_CHANNELS)
        shutdown("ads131m0x invalid total_channels");
    adc->channel_mask = args[4];
    uint8_t max_mask = (1 << adc->model.channel_count) - 1;
    if (adc->channel_mask == 0 || (adc->channel_mask & ~max_mask)) {
        shutdown("ads131m0x invalid channel_mask");
    }
    adc->output_channel_count = 0;
    uint8_t i;
    for (i = 0; i < adc->model.channel_count; i++) {
        if (adc->channel_mask & (1 << i))
            adc->output_channel_count++;
    }
    adc->model.sample_bytes = BYTES_PER_SAMPLE * adc->output_channel_count;
    adc->model.frame_size = (adc->model.channel_count + 2) * BYTES_PER_WORD;
    adc->model.crc_data_bytes = adc->model.frame_size - BYTES_PER_WORD;
    adc->model.crc_offset = adc->model.crc_data_bytes;
}
DECL_COMMAND(command_config_ads131m0x,
    "config_ads131m0x oid=%c spi_oid=%c data_ready_pin=%u total_channels=%c"
    " channel_mask=%c");

void
ads131m0x_attach_load_cell_probe(uint32_t *args) {
    uint8_t oid = args[0];
    struct ads131m0x_adc *adc = oid_lookup(oid, command_config_ads131m0x);
    adc->lce = load_cell_probe_oid_lookup(args[1]);
}
DECL_COMMAND(ads131m0x_attach_load_cell_probe,
    "ads131m0x_attach_load_cell_probe oid=%c load_cell_probe_oid=%c");

void
command_query_ads131m0x(uint32_t *args)
{
    uint8_t oid = args[0];
    struct ads131m0x_adc *adc = oid_lookup(oid, command_config_ads131m0x);
    sched_del_timer(&adc->timer);
    adc->pending_flag = 0;
    adc->rest_ticks = args[1];
    if (!adc->rest_ticks) {
        return;
    }
    adc->hard_error_latch = 0;
    sensor_bulk_reset(&adc->sb);
    irq_disable();
    adc->timer.waketime = timer_read_time() + adc->rest_ticks;
    sched_add_timer(&adc->timer);
    irq_enable();
}
DECL_COMMAND(command_query_ads131m0x, "query_ads131m0x oid=%c rest_ticks=%u");

void
command_query_ads131m0x_status(const uint32_t *args)
{
    uint8_t oid = args[0];
    struct ads131m0x_adc *adc = oid_lookup(oid, command_config_ads131m0x);
    irq_disable();
    const uint32_t start_t = timer_read_time();
    uint8_t is_data_ready = ads131m0x_is_data_ready(adc);
    irq_enable();
    uint8_t pending_bytes = is_data_ready ? adc->model.sample_bytes : 0;
    sensor_bulk_status(&adc->sb, oid, start_t, 0, pending_bytes);
}
DECL_COMMAND(command_query_ads131m0x_status, "query_ads131m0x_status oid=%c");

// Background task that performs measurements
void
ads131m0x_capture_task(void)
{
    if (!sched_check_wake(&wake_ads131m0x))
        return;
    uint8_t oid;
    struct ads131m0x_adc *adc;
    foreach_oid(oid, adc, command_config_ads131m0x) {
        if (adc->pending_flag)
            ads131m0x_read_adc(adc, oid);
    }
}
DECL_TASK(ads131m0x_capture_task);
