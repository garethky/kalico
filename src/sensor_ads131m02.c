// Support for ADS131M02 ADC Chip
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

struct ads131m02_adc {
    struct timer timer;
    uint32_t rest_ticks;
    uint32_t last_error;
    struct gpio_in data_ready;
    struct spidev_s *spi;
    uint8_t pending_flag;
    uint8_t channel_offset; // 3 or 6
    struct sensor_bulk sb;
    struct load_cell_probe *lce;
};

// Error codes sent as sample values (use high bits to distinguish from valid data)
#define SAMPLE_ERROR_CRC       (1L << 31)
#define SAMPLE_ERROR_RESET     (1L << 30)

#define BYTES_PER_SAMPLE 4
#define STATUS_RESET_BIT (1 << 2)  // Bit 10 of 16-bit status, in byte 0 (bits 15:8)
#define CRC_INITIAL 0xFFFF
#define CRC_POLY 0x1021  // CCITT polynomial: x^16 + x^12 + x^5 + 1

static struct task_wake wake_ads131m02;

/****************************************************************
 * ADS131M02 Sensor Support
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
ads131m02_is_data_ready(struct ads131m02_adc *adc) {
    return gpio_in_read(adc->data_ready) == 0;
}

// Event handler that wakes wake_ads131m02() periodically
static uint_fast8_t
ads131m02_event(struct timer *timer)
{
    struct ads131m02_adc *adc = container_of(timer, struct ads131m02_adc,
                                              timer);
    uint32_t rest_ticks = adc->rest_ticks;
    if (adc->pending_flag) {
        adc->sb.possible_overflows++;
        rest_ticks *= 4;
    } else if (ads131m02_is_data_ready(adc)) {
        adc->pending_flag = 1;
        sched_wake_task(&wake_ads131m02);
        rest_ticks *= 8;
    }
    adc->timer.waketime += rest_ticks;
    return SF_RESCHEDULE;
}

// Add a measurement to the buffer
static void
add_sample(struct ads131m02_adc *adc, uint8_t oid, uint_fast32_t counts)
{
    adc->sb.data[adc->sb.data_count] = counts;
    adc->sb.data[adc->sb.data_count + 1] = counts >> 8;
    adc->sb.data[adc->sb.data_count + 2] = counts >> 16;
    adc->sb.data[adc->sb.data_count + 3] = counts >> 24;
    adc->sb.data_count += BYTES_PER_SAMPLE;

    if ((adc->sb.data_count + BYTES_PER_SAMPLE) > ARRAY_SIZE(adc->sb.data)) {
        sensor_bulk_report(&adc->sb, oid);
    }
}

// ADS131M02 ADC query
static void
ads131m02_read_adc(struct ads131m02_adc *adc, uint8_t oid)
{
    // Typical communication frame at 24-bit word length:
    // 3-byte STATUS + 3-byte CH0 + 3-byte CH1 + 3-byte CRC.
    // Clock out with NULL command bytes on DIN.
    uint8_t rx[12] = {0};
    spidev_transfer(adc->spi, 1, sizeof(rx), rx);
    adc->pending_flag = 0;
    barrier();

    // Check for unexpected device reset (RESET bit in status byte 0)
    // This is a hard error - once set, keep sending until measurement restart
    if (rx[0] & STATUS_RESET_BIT)
        adc->last_error = SAMPLE_ERROR_RESET;

    // Validate CRC (covers first 9 bytes: STATUS + CH0 + CH1)
    uint16_t calc_crc = calc_crc16(rx, 9);
    uint16_t recv_crc = ((uint16_t)rx[9] << 8) | rx[10];
    uint8_t crc_error = (calc_crc != recv_crc);

    // Read counts from selected channel
    uint32_t counts = ((uint32_t)rx[adc->channel_offset] << 16)
                    | ((uint32_t)rx[adc->channel_offset + 1] << 8)
                    | ((uint32_t)rx[adc->channel_offset + 2]);

    // Sign-extend 24-bit two's complement to 32-bit
    if (counts & 0x800000)
        counts |= 0xFF000000;

    // Hard errors persist, CRC errors are transient (one sample only)
    if (adc->last_error)
        counts = adc->last_error;
    else if (crc_error)
        counts = SAMPLE_ERROR_CRC;

    if (adc->lce)
        load_cell_probe_report_sample(adc->lce, counts);
    add_sample(adc, oid, counts);
}

// Create an ads131m02 sensor
void
command_config_ads131m02(uint32_t *args)
{
    struct ads131m02_adc *adc = oid_alloc(args[0]
                , command_config_ads131m02, sizeof(*adc));
    adc->timer.func = ads131m02_event;
    adc->pending_flag = 0;
    adc->spi = spidev_oid_lookup(args[1]);
    adc->data_ready = gpio_in_setup(args[2], 0);
    uint8_t ch = args[3];
    adc->channel_offset = (ch > 0) ? 6 : 3;
}
DECL_COMMAND(command_config_ads131m02, "config_ads131m02 oid=%c spi_oid=%c data_ready_pin=%u channel=%c");

void
ads131m02_attach_load_cell_probe(uint32_t *args) {
    uint8_t oid = args[0];
    struct ads131m02_adc *adc = oid_lookup(oid, command_config_ads131m02);
    adc->lce = load_cell_probe_oid_lookup(args[1]);
}
DECL_COMMAND(ads131m02_attach_load_cell_probe,
    "ads131m02_attach_load_cell_probe oid=%c load_cell_probe_oid=%c");

// start/stop capturing ADC data
void
command_query_ads131m02(uint32_t *args)
{
    uint8_t oid = args[0];
    struct ads131m02_adc *adc = oid_lookup(oid, command_config_ads131m02);
    sched_del_timer(&adc->timer);
    adc->pending_flag = 0;
    adc->rest_ticks = args[1];
    if (!adc->rest_ticks) {
        // End measurements
        return;
    }
    // Start new measurements
    adc->last_error = 0;
    sensor_bulk_reset(&adc->sb);
    irq_disable();
    adc->timer.waketime = timer_read_time() + adc->rest_ticks;
    sched_add_timer(&adc->timer);
    irq_enable();
}
DECL_COMMAND(command_query_ads131m02, "query_ads131m02 oid=%c rest_ticks=%u");

void
command_query_ads131m02_status(const uint32_t *args)
{
    uint8_t oid = args[0];
    struct ads131m02_adc *adc = oid_lookup(oid, command_config_ads131m02);
    irq_disable();
    const uint32_t start_t = timer_read_time();
    uint8_t is_data_ready = ads131m02_is_data_ready(adc);
    irq_enable();
    uint8_t pending_bytes = is_data_ready ? BYTES_PER_SAMPLE : 0;
    sensor_bulk_status(&adc->sb, oid, start_t, 0, pending_bytes);
}
DECL_COMMAND(command_query_ads131m02_status, "query_ads131m02_status oid=%c");

// Background task that performs measurements
void
ads131m02_capture_task(void)
{
    if (!sched_check_wake(&wake_ads131m02))
        return;
    uint8_t oid;
    struct ads131m02_adc *adc;
    foreach_oid(oid, adc, command_config_ads131m02) {
        if (adc->pending_flag)
            ads131m02_read_adc(adc, oid);
    }
}
DECL_TASK(ads131m02_capture_task);
