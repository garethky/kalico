# ADS131Mx Support
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

from klippy import ConfigWrapper, Printer
from klippy.extras import bus
from klippy.extras.bulk_sensor import BatchBulkHelper, FixedFreqReader
from klippy.extras.load_cell.interfaces import (
    BulkAdcDataCallback,
    LoadCellSensor,
)
from klippy.pins import PrinterPins
from klippy.reactor import Reactor

# Constants
WORD_SIZE = 3  # there are 3 bytes in a word in the protocol
BYTES_PER_SAMPLE = 4
UPDATE_INTERVAL = 0.10
NULL_CMD = 0x0000
RESET_CMD = 0x0011
RREG_CMD = 0b101 << 13
WREG_CMD = 0b011 << 13
STATUS_REG = 0x01
MODE_REG = 0x02
CLOCK_REG = 0x03
GAIN_REG = 0x04
PWR_HR = 0b10  # High resolution mode
STATUS_RESET_BIT = 1 << 10  # RESET bit in STATUS register
# Error codes from MCU (match sensor_ads131m02.c)
SAMPLE_ERROR_CRC = -0x80000000  # 1 << 31 as signed
SAMPLE_ERROR_RESET = 0x40000000  # 1 << 30
ADC_FACTOR = 1.0 / (1 << 23)


def hexify(byte_array):
    return "[%s]" % (", ".join([hex(b) for b in byte_array]))


# Chip Documentation: https://www.ti.com/lit/ds/symlink/ads131m02.pdf
#                     https://www.ti.com/lit/ds/symlink/ads131m04.pdf


def channels_to_mask(channels):
    channel_mask = 0
    for channel in channels:
        channel_mask |= 1 << channel
    return channel_mask


class ADS131MxBase(LoadCellSensor):
    def __init__(
        self,
        config: ConfigWrapper,
        sensor_type,
        sample_rate_options,
        default_sample_rate,
        channel_count,
    ):
        self.printer: Printer = config.get_printer()
        self.reactor: Reactor = self.printer.get_reactor()
        self.name = config.get_name().split()[-1]
        self.sensor_type = sensor_type
        self.channel_count = channel_count
        self.last_error_count = 0
        self.consecutive_fails = 0
        self.sps = config.getchoice(
            "sample_rate",
            sample_rate_options,
            default=default_sample_rate,
        )
        gain_options = {
            1: 0,
            2: 1,
            4: 2,
            8: 3,
            16: 4,
            32: 5,
            64: 6,
            128: 7,
        }
        self.gain = config.getchoice("gain", gain_options, default=128)
        self.spi = bus.MCU_SPI_from_config(config, 1, default_speed=8192000)
        self.mcu = mcu = self.spi.get_mcu()
        self.oid = mcu.create_oid()
        # Data Ready (DRDY) Pin
        drdy_pin: str = config.get("data_ready_pin")
        ppins: PrinterPins = self.printer.lookup_object("pins")
        drdy_ppin = ppins.lookup_pin(drdy_pin)
        self.data_ready_pin = drdy_ppin["pin"]
        if drdy_ppin["chip"] != self.mcu:
            raise config.error(
                f"{self.sensor_type} config error: SPI communication and "
                "data_ready_pin must be on the same MCU"
            )
        self.channels = self._read_channels(config)
        self.output_channel_count = len(self.channels)
        self.channel_mask = channels_to_mask(self.channels)
        # Bulk Sensor Setup
        chip_smooth = self.sps * UPDATE_INTERVAL * 2
        self.unpack_format = "<" + ("i" * self.output_channel_count)
        self.ffreader: FixedFreqReader = FixedFreqReader(
            mcu, chip_smooth, self.unpack_format
        )
        self.batch_bulk: BatchBulkHelper = BatchBulkHelper(
            self.printer,
            self._process_batch,
            self._start_measurements,
            self._finish_measurements,
            UPDATE_INTERVAL,
        )
        mcu.register_config_callback(self._build_config)
        self.attach_probe_cmd = None
        self.query_ads131m0x_cmd = None

    def _build_config(self):
        cq = self.spi.get_command_queue()
        self.mcu.add_config_cmd(
            f"config_ads131m0x oid={self.oid} spi_oid={self.spi.get_oid()} "
            f"data_ready_pin={self.data_ready_pin} "
            f"total_channels={self.channel_count} "
            f"channel_mask={self.channel_mask}"
        )
        self.mcu.add_config_cmd(
            f"query_ads131m0x oid={self.oid} rest_ticks=0", on_restart=True
        )
        self.query_ads131m0x_cmd = self.mcu.lookup_command(
            "query_ads131m0x oid=%c rest_ticks=%u", cq=cq
        )
        self.attach_probe_cmd = self.mcu.lookup_command(
            "ads131m0x_attach_load_cell_probe oid=%c load_cell_probe_oid=%c"
        )
        self.ffreader.setup_query_command(
            "query_ads131m0x_status oid=%c", oid=self.oid, cq=cq
        )

    def _read_channels(self, config: ConfigWrapper):
        channels = config.getintlist("channels", default=(0,))
        unique_channels = sorted(set(channels))
        max_channel = self.channel_count - 1
        for channel in unique_channels:
            if channel < 0 or channel > max_channel:
                raise config.error(
                    f"{self.sensor_type.upper()} channels must be in range "
                    f"0..{max_channel}"
                )
        if not unique_channels:
            raise config.error(
                f"{self.sensor_type.upper()} channels must contain at least one"
            )
        return unique_channels

    def get_mcu(self):
        return self.mcu

    def get_samples_per_second(self):
        return self.sps

    def get_range(self):
        # 24-bit signed range
        return -0x800000, 0x7FFFFF

    def get_channel_count(self):
        return self.output_channel_count

    def add_client(self, callback: BulkAdcDataCallback):
        self.batch_bulk.add_client(callback)

    def attach_load_cell_probe(self, load_cell_probe_oid: int):
        self.attach_probe_cmd.send([self.oid, load_cell_probe_oid])

    # Measurement decoding
    def _convert_samples(self, samples):
        samples_out = []
        for sample in samples:
            ptime = sample[0]
            channel_counts = sample[1:]
            val = channel_counts[0]
            if val == SAMPLE_ERROR_CRC or val == SAMPLE_ERROR_RESET:
                self.last_error_count += 1
                logging.error(
                    f"{self.sensor_type} sample error: {channel_counts[0]}"
                )
                break
            converted_sample = [round(ptime, 6)]
            for channel in channel_counts:
                converted_sample.append(channel)
                converted_sample.append(channel * ADC_FACTOR)
            samples_out.append(converted_sample)
        return list(samples_out)

    def _start_measurements(self):
        self.last_error_count = 0
        self.consecutive_fails = 0
        self.reset_chip()
        self.setup_chip()
        rest_ticks = self.mcu.seconds_to_clock(1.0 / (10.0 * self.sps))
        self.query_ads131m0x_cmd.send([self.oid, rest_ticks])
        logging.info(
            "%s starting '%s' measurements", self.sensor_type, self.name
        )
        self.ffreader.note_start()

    def _finish_measurements(self):
        if self.printer.is_shutdown():
            return
        self.query_ads131m0x_cmd.send_wait_ack([self.oid, 0])
        self.ffreader.note_end()
        logging.info(
            "%s finished '%s' measurements", self.sensor_type, self.name
        )

    def _process_batch(self, eventtime):
        samples = self.ffreader.pull_samples()
        return {
            "data": self._convert_samples(samples),
            "errors": self.last_error_count,
            "overflows": self.ffreader.get_last_overflows(),
        }

    # --- SPI Communication Helpers ---
    # The ADS131M02 uses 24-bit SPI words. Commands and register data are
    # 16-bit values, MSB-aligned with zero padding: [MSB, LSB, 0x00]
    @staticmethod
    def _to_words(*values_16bit):
        """Convert 16-bit values to 24-bit words as a byte array."""
        payload = []
        for val in values_16bit:
            payload.extend([(val >> 8) & 0xFF, val & 0xFF, 0x00])
        return payload

    def _send_command(self, cmd_16bit):
        """Send a frame of 16-bit values as 24-bit words."""
        self.spi.spi_send(
            self._to_words(cmd_16bit, NULL_CMD, NULL_CMD, NULL_CMD)
        )

    def _transfer_frame(self, *values_16bit):
        """Send frame and return response as list of 16-bit values."""
        # send 2 frames (8x24 bits), the response is in the second frame
        target_size = 2 * 4 * WORD_SIZE
        transfer_words = self._to_words(*values_16bit)
        # pad to target size with 0's
        while len(transfer_words) < target_size:
            transfer_words.extend(self._to_words(NULL_CMD))
        params = self.spi.spi_transfer(transfer_words)
        resp = params.get("response", [])
        logging.info("%s transfer response: %s", self.sensor_type, hexify(resp))
        result = []
        for i in range(0, len(resp), 3):
            if i + 1 < len(resp):
                result.append((resp[i] << 8) | resp[i + 1])
        return result

    # --- Command Helpers ---
    # WREG: 011a_aaaa_annn_nnnn (a=6-bit address, n=7-bit count-1)
    # RREG: 101a_aaaa_annn_nnnn

    def _build_register_command(self, cmd, addr, count=1):
        return cmd | ((addr & 0x3F) << 7) | ((count - 1) & 0x7F)

    def _wreg_cmd(self, addr, count=1):
        return self._build_register_command(WREG_CMD, addr, count)

    def _rreg_cmd(self, addr, count=1):
        return self._build_register_command(RREG_CMD, addr, count)

    def _read_reg(self, addr):
        resp = self._transfer_frame(self._rreg_cmd(addr))
        if len(resp) < 5:
            raise self.printer.command_error(
                f"{self.sensor_type} {self.name}: no response reading reg "
                f"0x{addr:02x}"
            )
        return resp[4]

    def _write_reg(self, addr, value):
        """Write a single register."""
        self._transfer_frame(self._wreg_cmd(addr), value)

    def _write_and_verify_reg(self, addr, value):
        """Write a register and verify the value was set."""
        self._write_reg(addr, value)
        actual = self._read_reg(addr)
        if actual != value:
            raise self.printer.command_error(
                f"{self.sensor_type} {self.name}: reg 0x{addr:02x} write "
                f"failed: wrote 0x{value:04x}, read 0x{actual:04x}"
            )

    def reset_chip(self):
        self._send_command(RESET_CMD)
        self.reactor.pause(self.reactor.monotonic() + 0.020)
        self._send_command(NULL_CMD)
        self.reactor.pause(self.reactor.monotonic() + 0.002)
        status = self._read_reg(STATUS_REG)
        logging.info(
            "%s %s: reset complete, STATUS=0x%04x",
            self.sensor_type,
            self.name,
            status,
        )

    def _clock_channel_enable_bits(self):
        return self.channel_mask << 8

    def _gain_register_value(self):
        gain_val = 0
        for channel in range(self.channel_count):
            gain_val |= self.gain << (channel * 4)
        return gain_val

    def setup_chip(self):
        # MODE register (0x02): clear RESET bit (bit 10), set 24-bit word length (bits 9:8 = 01)
        # Reset default is 0x0510: RESET=1, WLENGTH=01, TIMEOUT=1
        wlength_24 = 0b01 << 8
        timeout_en = 1 << 4
        mode_write_val = wlength_24 | timeout_en
        self._write_reg(MODE_REG, mode_write_val)
        mode_val = self._read_reg(MODE_REG)
        if (mode_val & 0x0300) != wlength_24:
            raise self.printer.command_error(
                f"{self.sensor_type} {self.name}: MODE reg write failed: "
                f"got 0x{mode_val:04x}"
            )
        # CLOCK register (0x03): set OSR for sample rate, high resolution mode
        # OSR[2:0] at bits 4:2, PWR[1:0] at bits 1:0
        # PWR=10 for high-resolution mode, CH0_EN and CH1_EN at bits 8:9
        channel_enable = self._clock_channel_enable_bits()
        # SPS -> OSR code: OSR=fMOD/fDATA, fMOD=fCLKIN/2=4.096MHz
        osr_codes = {
            32000: 0,  # OSR=128
            16000: 1,  # OSR=256
            8000: 2,  # OSR=512
            4000: 3,  # OSR=1024
            2000: 4,  # OSR=2048
            1000: 5,  # OSR=4096
            500: 6,  # OSR=8192
            250: 7,  # OSR=16384
        }
        osr_code = osr_codes.get(self.sps)
        clock_val = channel_enable | (osr_code << 2) | PWR_HR
        self._write_and_verify_reg(CLOCK_REG, clock_val)
        # GAIN register (0x04): PGAGAIN0[2:0] at bits 2:0, PGAGAIN1[2:0] at bits 6:4
        self._write_and_verify_reg(GAIN_REG, self._gain_register_value())
        self.reactor.pause(self.reactor.monotonic() + 0.050)  # 50ms delay
        status = self._read_reg(STATUS_REG)
        logging.info(
            "ADS131M0X %s: post-WAKEUP STATUS=0x%04x", self.name, status
        )


class ADS131M02(ADS131MxBase):
    def __init__(self, config: ConfigWrapper):
        super().__init__(
            config,
            sensor_type="ads131m02",
            sample_rate_options={
                250: 250,
                500: 500,
                1000: 1000,
                2000: 2000,
                4000: 4000,
                8000: 8000,
                16000: 16000,
                32000: 32000,
            },
            default_sample_rate=500,
            channel_count=2,
        )


class ADS131M04(ADS131MxBase):
    def __init__(self, config: ConfigWrapper):
        super().__init__(
            config,
            sensor_type="ads131m04",
            sample_rate_options={
                250: 250,
                500: 500,
                1000: 1000,
                2000: 2000,
                4000: 4000,
                8000: 8000,
                16000: 16000,
                32000: 32000,
            },
            default_sample_rate=500,
            channel_count=4,
        )


ADS131M0X_SENSOR_TYPES = {
    "ads131m02": ADS131M02,
    "ads131m04": ADS131M04,
}
