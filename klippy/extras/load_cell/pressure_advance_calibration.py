# Load Cell PA Capture
#
# Copyright (C) 2026  Gareth Farrington <gareth@waves.ky>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from sys import float_info

import numpy as np
from scipy import signal

from klippy.configfile import ConfigWrapper
from klippy.extras.gcode_move import GCodeMove
from klippy.extras.heaters import Heater
from klippy.extras.load_cell.load_cell import (
    LoadCell,
    LoadCellSampleCollector,
)
from klippy.gcode import GCodeCommand, GCodeDispatch
from klippy.kinematics.extruder import PrinterExtruder
from klippy.printer import Printer
from klippy.toolhead import ToolHead

## TODO/Ideas
# * Save the results in a printer state object to support scripting,
# e.g. printing a test patters with the value afterwards
# * Have a flag that sets the current PA value to the calculated value at the
# end of the test
# Support round beds by printing in an arc about the center.

## Testing
# * Validation with several filaments, speeds and accelerations
# * Validation on Bowden systems
# * Test how the length parameter impacts accuracy. Is there any way to tell
# if it is too short and warn the user?
# * Validate that the speed compression works as expected. Compare force
# plots of compressed to uncompressed runs.
# * Is the synthetic elbow accurate?


MIN_PEAK_PTS = 5
SAVGOL_POLYORDER: int = 3
SAVGOL_WINDOW_LENGTH = 25
TEST_PATTERN_MARGIN = 10.0
JUNCTION_LENGTH = 0.01
MAX_NOZZLE_WIDTH_MULTIPLE = 8.0


@dataclass(frozen=True)
class Junction:
    index: int
    accel_start: float
    decel_start: float

    def to_dict(self) -> dict[str, int | float]:
        return asdict(self)


@dataclass(frozen=True)
class FitResult:
    junction_index: int
    tau: float
    f_init: float
    f_final: float
    elbow_idx: int
    elbow_time: float
    n_transient: int
    n_steady: int
    peak_time: float
    peak_force: float

    def to_dict(self) -> dict[str, int | float]:
        return asdict(self)


@dataclass(frozen=True)
class TrapqMove:
    print_time: float
    move_t: float
    start_v: float
    accel: float
    start_x: float
    start_y: float
    start_z: float

    @classmethod
    def from_move(cls, move) -> TrapqMove:
        return cls(
            print_time=float(move.print_time),
            move_t=float(move.move_t),
            start_v=float(move.start_v),
            accel=float(move.accel),
            start_x=float(move.start_x),
            start_y=float(move.start_y),
            start_z=float(move.start_z),
        )

    @property
    def end_time(self) -> float:
        return self.print_time + self.move_t

    @property
    def distance(self) -> float:
        return (self.start_v + (0.5 * self.accel * self.move_t)) * self.move_t

    def is_nano_coast(self) -> bool:
        return self.move_t < float_info.epsilon and self.accel == 0.0

    # Bed mesh splits moves in the TRAPQ. This method re-combine sequential
    # moves that have the same acceleration value into 1 move.
    @staticmethod
    def combine_split_moves(trapq_moves: list[TrapqMove]) -> list[TrapqMove]:
        combined_moves: list[TrapqMove] = []
        i: int = 0
        while i < len(trapq_moves):
            first_move: TrapqMove = trapq_moves[i]
            accel_value = first_move.accel
            # find the last move in the split sequence
            while i + 1 < len(trapq_moves) and (
                trapq_moves[i + 1].accel == accel_value
                # sometimes sub-nanosecond coasting moves are inserted?!
                or trapq_moves[i + 1].is_nano_coast()
            ):
                i += 1
            last_move = trapq_moves[i]
            if last_move == first_move:
                combined_moves.append(first_move)
            else:
                combined_moves.append(
                    TrapqMove(
                        print_time=first_move.print_time,
                        move_t=last_move.end_time - first_move.print_time,
                        start_v=first_move.start_v,
                        accel=first_move.accel,
                        start_x=first_move.start_x,
                        start_y=first_move.start_y,
                        start_z=first_move.start_z,
                    )
                )
            i += 1
        return combined_moves

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


@dataclass(frozen=True)
class PADataRecord:
    speed: float
    accel: float
    flow_rate: float
    time: list[float]
    force: list[float]
    junctions: list[Junction]
    fit_results: list[FitResult]
    trapq_moves: list[TrapqMove]
    analysis_error: str | None = None

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


class PressureAdvanceAnalyzer:
    def __init__(self, printer: Printer, load_cell: LoadCell, cmd_err):
        self._printer: Printer = printer
        self._toolhead: ToolHead = printer.lookup_object("toolhead")
        self._load_cell = load_cell
        self.cmd_err = cmd_err
        self.collector: LoadCellSampleCollector | None = None
        self.time: list[float] = []
        self.force: list[float] = []
        self.t_zero: float = 0.0
        self._capture_start_time: float = 0.0

    def start_capture(self):
        self.collector = self._load_cell.get_collector()
        self._capture_start_time = self._toolhead.get_last_move_time()
        self.time = []
        self.force = []
        self.collector.start_collecting(min_time=self._capture_start_time)

    def abort_capture(self):
        if self.collector is not None:
            self.collector.stop_collecting()
            self.collector = None

    def _extract_trapq(
        self, start_time: float, end_time: float
    ) -> list[TrapqMove]:
        trapq = self._printer.lookup_object("motion_report").trapqs["toolhead"]
        moves, _ = trapq.extract_trapq(start_time, end_time)
        return [
            TrapqMove.from_move(move)
            for move in moves
            if move.move_t > 0.0 and (move.start_v > 0.0 or move.accel != 0.0)
        ]

    # apply a smoothing filter to the measured force
    def filter_force(self) -> list[float]:
        return signal.savgol_filter(
            self.force,
            window_length=SAVGOL_WINDOW_LENGTH,
            polyorder=SAVGOL_POLYORDER,
            mode="interp",
        )

    def detect_sample_rate(self) -> float:
        return (self.time[-1] - self.time[0]) / (len(self.time) - 1)

    def finish_capture(self) -> list[TrapqMove]:
        self._toolhead.flush_step_generation()
        move_end_time = self._toolhead.get_last_move_time()
        trapq_moves = self._extract_trapq(
            self._capture_start_time, move_end_time
        )
        if not trapq_moves:
            raise self.cmd_err("No toolhead moves captured")
        self.t_zero = trapq_moves[0].print_time
        try:
            raw_samples, errors = self.collector.collect_until(move_end_time)
        except Exception:
            self.collector.stop_collecting()
            raise
        self.collector = None
        if errors:
            error_count, overflow_count = errors
            raise self.cmd_err(
                "Sensor errors during capture: "
                "%d errors, %d overflows" % (error_count, overflow_count)
            )
        if not raw_samples:
            raise self.cmd_err("No samples captured")
        for sample in raw_samples:
            self.time.append(round(sample[0] - self.t_zero, 6))
            self.force.append(sample[1])
        return trapq_moves

    # make force go up for extrusion pressure
    def normalize_force_polarity(self, junctions: list[Junction]):
        if len(self.time) < 3 or len(self.force) < 3 or not junctions:
            return
        junc = junctions[0]
        time = np.asarray(self.time, dtype=np.float64)
        accel_idx: int = int(np.abs(time - junc.accel_start).argmin())
        decel_idx = int(np.abs(time - junc.decel_start).argmin())
        # this is dubious
        if decel_idx <= accel_idx:
            raise ValueError("your time data for the junction is wrong!")
        force = np.asarray(self.force, dtype=np.float64)
        start_force = float(force[accel_idx])
        end_force = float(force[decel_idx])
        if end_force < start_force:
            self.force = (-force).tolist()

    @staticmethod
    def estimate_junction_pa(
        times,
        forces,
        junction_index: int,
        junction_time: float,
        sample_delay: float,
    ) -> FitResult | None:
        n = len(times)
        if n < MIN_PEAK_PTS:
            return None
        reversal_idx = int(np.argmin(forces))
        reversal_time = float(times[reversal_idx])
        tau = (reversal_time - junction_time) - sample_delay
        if tau <= 0.0:
            return None
        reversal_force = float(forces[reversal_idx])
        n_transient = reversal_idx + 1
        return FitResult(
            junction_index=junction_index,
            tau=float(tau),
            f_init=float(forces[0]),
            f_final=reversal_force,
            elbow_idx=reversal_idx,
            elbow_time=reversal_time,
            n_transient=n_transient,
            n_steady=max(n - n_transient, 0),
            peak_time=reversal_time,
            peak_force=reversal_force,
        )

    def estimate_pa(self, junctions: list[Junction]) -> list[FitResult]:
        if not junctions:
            return []
        fit_results: list[FitResult] = []
        junc: Junction
        filtered_force = self.filter_force()
        sample_delay = self.detect_sample_rate() / 2.0
        for junc in junctions:
            times_win = []
            forces_win = []
            for i in range(len(self.time)):
                t = self.time[i]
                f = filtered_force[i]
                if f is None:
                    continue
                if junc.accel_start <= t < junc.decel_start:
                    times_win.append(t)
                    forces_win.append(f)
            if len(forces_win) < MIN_PEAK_PTS:
                continue
            times_arr = np.array(times_win)
            forces_arr = np.array(forces_win)
            result = self.estimate_junction_pa(
                times_arr,
                forces_arr,
                junc.index,
                junc.accel_start,
                sample_delay,
            )
            if result is None:
                continue
            fit_results.append(result)
        return fit_results


# this just grabs params from the Gcode command and its environment
class ParamsGrabber:
    def __init__(self, printer, gcmd: GCodeCommand):
        self.printer: Printer = printer
        self.gcmd: GCodeCommand = gcmd
        self.gcode: GCodeDispatch = printer.lookup_object("gcode")
        self.toolhead: ToolHead = printer.lookup_object("toolhead")
        # check for round beds, we don't handle these yet
        # TODO: round bed support
        kinematics = self.toolhead.get_kinematics()
        if hasattr(kinematics, "radius"):
            raise gcmd.error("Sorry Deltas: round beds not yet implemented")
        self.extruder: PrinterExtruder = self.toolhead.get_extruder()
        if self.extruder is None:
            raise gcmd.error("No active extruder for PA capture")
        # capture PA settings so they can be restored
        self.original_pa: float = (
            self.extruder.extruder_stepper.pressure_advance
        )
        self.original_smooth_time: float = (
            self.extruder.extruder_stepper.pressure_advance_smooth_time
        )
        heater = self.extruder.get_heater()
        self.temp: float = gcmd.get_float(
            "TEMP", minval=heater.min_extrude_temp, maxval=heater.max_temp
        )
        self.nozzle_diameter: float = gcmd.get_float(
            "NOZZLE_DIAMETER", self.extruder.nozzle_diameter, above=0.0
        )
        self.filament_area: float = self.extruder.filament_area
        # Extrusion parameters
        self.speed: list[float] = self._get_float_list("SPEED", above=0.0)
        self.accel: list[float] = self._get_float_list(
            "ACCEL", above=0.0, maxval=self.toolhead.max_accel
        )
        self.advance: list[float] = self._get_float_list("ADVANCE", above=0.0)
        if len(self.speed) != len(self.accel):
            raise gcmd.error("Speed and accel nees to be the same length")
        # if the advance parameter is missing, initialize to 0.0's
        if len(self.speed) != len(self.advance):
            self.advance = [0.0] * len(self.speed)
        self.length: float = gcmd.get_float("LENGTH", 100.0, above=0.0)
        self.layer_height: float = gcmd.get_float(
            "LAYER_HEIGHT", 0.2, above=0.0
        )
        self.line_width: float = gcmd.get_float(
            "LINE_WIDTH", self.nozzle_diameter * 1.125, above=0.0
        )
        self.flow_multiplier: float = gcmd.get_float(
            "FLOW_MULTIPLIER", 1.0, above=0.0
        )
        # Motion parameters
        self.square_corner_velocity: float = (
            self.toolhead.square_corner_velocity
        )
        self.junctions: int = gcmd.get_int("JUNCTIONS", 10, minval=1)
        available_width: float = self._get_available_width()
        default_width: float = available_width - TEST_PATTERN_MARGIN
        self.width: float = gcmd.get_float(
            "WIDTH", above=0.0, maxval=available_width, default=default_width
        )
        self.output_path = gcmd.get("OUTPUT_PATH", default=None)

    def _get_float_list(
        self, name: str, above: float = None, maxval: float = None
    ) -> list[float]:
        value = self.gcmd.get(name, default="")
        # Return an empty list for empty value
        if len(value.strip()) == 0:
            return []
        try:
            float_list: list[float] = [
                float(p.strip()) for p in value.split(",")
            ]
        except:
            raise self.gcmd.error(
                f"Error on '{self.gcmd.get_commandline()}': unable to parse"
                f" {value}"
            )
        for val in float_list:
            if above is not None and val <= above:
                raise self.gcmd.error(
                    f"{name} value {val} must be above {above}"
                )
            if maxval is not None and val > maxval:
                raise self.gcmd.error(
                    f"{name} value {val} must be less than {maxval}"
                )
        return float_list

    def _get_bed_mesh_x_max(self) -> float | None:
        bed_mesh = self.printer.lookup_object("bed_mesh", default=None)
        if bed_mesh is None:
            return None
        mesh_config = bed_mesh.bmc.orig_config
        return float(mesh_config["mesh_max"][0])

    def _get_x_axis_max(self) -> float:
        kinematics = self.toolhead.get_kinematics()
        kin_status = kinematics.get_status(None)
        return float(kin_status["axis_maximum"][0])

    def _get_available_width(
        self,
    ) -> float:
        x_max: float | None = self._get_bed_mesh_x_max()
        if x_max is None:
            x_max = self._get_x_axis_max()
        x_pos: float = self.toolhead.get_position()[0]
        return x_max - x_pos


class PATestPattern:
    def __init__(
        self,
        printer: Printer,
        params_grabber: ParamsGrabber,
        speed: float,
        accel: float,
        advance: float,
        cmd_err,
        x_direction: float = 1.0,
        y_direction: float = 1.0,
    ):
        self.speed: float = speed
        self.accel: float = accel
        self.advance: float = advance
        self.x_direction: float = x_direction
        self.y_direction: float = y_direction
        self._printer: Printer = printer
        self._toolhead: ToolHead = printer.lookup_object("toolhead")
        self._gcode: GCodeDispatch = printer.lookup_object("gcode")
        self._heaters = printer.lookup_object("heaters")
        self._extruder: PrinterExtruder = params_grabber.extruder
        self._extruder_heater: Heater = params_grabber.extruder.get_heater()
        self.layer_height: float = params_grabber.layer_height
        self.flow_multiplier: float = params_grabber.flow_multiplier
        self.line_width: float = params_grabber.line_width
        self.filament_area: float = params_grabber.filament_area
        self.corner_length: float = JUNCTION_LENGTH
        self.purge_segments: int = 1  # TODO: let the user decide how many
        virtual_length: float = params_grabber.length
        junctions = params_grabber.junctions
        self.segments = junctions + 1
        total_virtual_length = (
            (self.segments * virtual_length)
            + (junctions * self.corner_length)
            + (self.purge_segments * virtual_length)
        )
        self.compression_factor = max(
            total_virtual_length / params_grabber.width, 1.0
        )
        self._validate_cross_section(params_grabber, cmd_err)
        self.segment_line_width = self.line_width * self.compression_factor
        # Corner move setup
        self.corner_speed: float = params_grabber.square_corner_velocity
        self.corner_extrusion_length = (
            self.extrusion_mm_per_mm(self.segment_line_width)
            * self.corner_length
        )
        # Compress the physical pattern to fit WIDTH while preserving the
        # requested extrusion conditions in the analysis.
        self.segment_speed = speed / self.compression_factor
        self.segment_accel = accel / self.compression_factor
        self.segment_length = params_grabber.length / self.compression_factor
        self.segment_extrusion_length = (
            self.extrusion_mm_per_mm(self.segment_line_width)
            * self.segment_length
        )
        # purge needs to include the length of all the corner junctions
        self.purge_length = self.purge_segments * self.segment_length
        self.purge_extrusion_length = (
            self.extrusion_mm_per_mm(self.segment_line_width)
            * self.purge_length
        )
        # use the print speed as the travel speed
        self.travel_speed: float = speed
        self.extruder_temp: float = params_grabber.temp
        self.original_pa = params_grabber.original_pa
        self.original_smooth_time = params_grabber.original_smooth_time

    # the mm of filament to extrude for every mm of printed length
    def extrusion_mm_per_mm(self, line_width: float) -> float:
        return (
            line_width * self.layer_height * self.flow_multiplier
        ) / self.filament_area

    @property
    def cross_sectional_area(self) -> float:
        return self.line_width * self.layer_height * self.flow_multiplier

    # Flow rate in mm^3/s
    @property
    def flow_rate(self) -> float:
        return self.cross_sectional_area * self.speed

    def _validate_cross_section(self, params: ParamsGrabber, cmd_err):
        gcode_move: GCodeMove = self._printer.lookup_object("gcode_move")
        current_z: float = gcode_move.last_position[2]
        if current_z <= 0.0:
            raise cmd_err(
                "PA test pattern requires positive Z height,"
                " current Z=%.3f" % (current_z,)
            )
        max_cross_section: float = (
            MAX_NOZZLE_WIDTH_MULTIPLE * params.nozzle_diameter * current_z
        )
        line_area: float = (
            params.line_width * params.layer_height * params.flow_multiplier
        )
        segment_volume: float = line_area * params.length
        segment_length: float = params.length / self.compression_factor
        segment_cross_section: float = segment_volume / segment_length
        if segment_cross_section <= max_cross_section:
            return
        raise cmd_err(
            "compressed PA pattern cross-section %.3fmm^2 exceeds "
            "safe limit %.3fmm^2 at current Z=%.1fmm; raise Z, "
            "increase available WIDTH"
            % (segment_cross_section, max_cross_section, current_z)
        )

    def get_junction_list(self, trapq_moves: list[TrapqMove]) -> list[Junction]:
        if self.segments < 2:
            return []
        trapq_moves = TrapqMove.combine_split_moves(trapq_moves)
        t_zero = trapq_moves[0].print_time
        junctions: list[Junction] = []
        junction_index = 0
        for index in range(3, len(trapq_moves) - 3, 4):
            # corner_move = trapq_moves[index]
            segment_acceleration_move = trapq_moves[index + 1]
            accel_start = segment_acceleration_move.print_time - t_zero
            segment_deceleration_move = trapq_moves[index + 3]
            decel_start = segment_deceleration_move.print_time - t_zero
            junctions.append(Junction(junction_index, accel_start, decel_start))
            junction_index += 1
        return junctions

    def set_pressure_advance(self, advance: float):
        self._extruder.extruder_stepper.set_pressure_advance(advance, self.original_smooth_time)

    def heat_extruder(self):
        self._heaters.set_temperature(
            self._extruder_heater, self.extruder_temp, wait=True
        )

    def travel_move(self, x: float, y: float):
        self._gcode.run_script_from_command(
            f"G1 X{x:.6f} Y{y:.6f} F{self.travel_speed * 60.0:.1f}"
        )

    def extruding_move(
        self, x: float, y: float, extrusion_length: float, speed: float
    ):
        self._gcode.run_script_from_command(
            f"G1 X{x:.6f} Y{y:.6f} E{extrusion_length:.6f} F{speed * 60.0:.1f}"
        )

    def restore_gcode_state(self):
        pass

    def setup_gcode_state(self):
        self.set_pressure_advance(self.advance)
        self._gcode.run_script_from_command("G91")
        self._gcode.run_script_from_command("M83")
        self._gcode.run_script_from_command("G92 E0")
        # turn off any speed or extrusion overrides the user is doing:
        self._gcode.run_script_from_command("M220 S100")
        self._gcode.run_script_from_command("M221 S100")

    def set_acceleration(self, acceleration: float):
        self._gcode.run_script_from_command(f"M204 S{acceleration:.1f}")

    # emit a purge line from left to right
    def purge(self):
        self.set_acceleration(self.segment_accel)
        self.extruding_move(
            self.purge_length * self.x_direction,
            0.0,
            self.purge_extrusion_length,
            self.segment_speed,
        )

    def step_move(self):
        self.travel_move(0.0, 2.0 * self.y_direction)

    # print the test pattern from right to left:
    def print_test_pattern(self):
        for segment in range(self.segments):
            # print one segment
            self.extruding_move(
                self.segment_length * self.x_direction,
                0.0,
                self.segment_extrusion_length,
                self.segment_speed,
            )
            if segment < self.segments - 1:
                # print a small "stalling move"
                self.extruding_move(
                    self.corner_length * self.x_direction,
                    0.0,
                    self.corner_extrusion_length,
                    self.corner_speed,
                )


class Reporter:
    def __init__(
        self,
        fit_results: list[FitResult],
        junctions: list[Junction],
        output_path: str | None = None,
        analysis_error: str | None = None,
        record: PADataRecord = None,
    ):
        self.fit_results = fit_results
        self.junctions: list[Junction] = junctions
        self.output_path: str | None = output_path
        self.analysis_error = analysis_error
        self.record = record
        self.pa: float | None = None

    def console_report(self, gcmd):
        if self.analysis_error is not None:
            gcmd.respond_info(
                f"pressure advance analysis failed: {self.analysis_error}"
            )
            return
        fitted = {result.junction_index for result in self.fit_results}
        rejected = [junc for junc in self.junctions if junc.index not in fitted]
        if len(rejected) > 1:
            raise gcmd.error(f"{len(rejected)} junctions failed to estimate")
        taus = [result.tau for result in self.fit_results]
        if not taus:
            raise gcmd.error("No junctions produced a PA estimate")
        min_tau = min(taus)
        max_tau = max(taus)
        range_tau = max_tau - min_tau
        avg_tau = float(np.mean(taus))
        self.pa = avg_tau
        std_tau = float(np.std(taus))
        gcmd.respond_info(
            f"pressure advance results: minimum {min_tau:.4f}, "
            f"maximum {max_tau:.4f}, range {range_tau:.4f}, "
            f"average {avg_tau:.4f}, standard deviation {std_tau:.4f}"
        )
        gcmd.respond_info(f"SET_PRESSURE_ADVANCE ADVANCE={avg_tau:.4f}")

    def save_json(self, gcmd: GCodeCommand):
        if self.output_path is None:
            return
        base = Path(self.output_path).expanduser()
        json_path = base.with_suffix(".jsonl")
        json_path.parent.mkdir(parents=True, exist_ok=True)
        try:
            with json_path.open("a", encoding="utf-8") as f:
                f.write(json.dumps(self.record.to_dict()) + "\n")

        except Exception as ex:
            gcmd.respond_info(str(ex))

    @classmethod
    def adaptive_pa_report(cls, gcmd: GCodeCommand, reports: list[Reporter]):
        if len(reports) < 2:
            return
        out: list[str] = ["| ADVANCE | FLOW RATE | ACCELERATION |"]
        for report in reports:
            record = report.record
            out.append(f"{report.pa:.4f}, {record.flow_rate:.1f},"
                       f" {record.accel:.0f}")
        gcmd.respond_info("\n".join(out))


class PressureAdvanceCalibration:
    def __init__(self, config: ConfigWrapper, load_cell: LoadCell):
        self._config: ConfigWrapper = config
        self._printer: Printer = self._config.get_printer()
        self._load_cell: LoadCell = load_cell

    def single_pattern(
        self, gcmd: GCodeCommand, test_pattern: PATestPattern, output_path: str
    ) -> Reporter:
        pa_analysis: PressureAdvanceAnalyzer = PressureAdvanceAnalyzer(
            self._printer, self._load_cell, gcmd.error
        )
        trapq_moves: list[TrapqMove]
        try:
            test_pattern.setup_gcode_state()
            test_pattern.heat_extruder()
            test_pattern.purge()
            pa_analysis.start_capture()
            test_pattern.print_test_pattern()
            trapq_moves = pa_analysis.finish_capture()
        except Exception:
            pa_analysis.abort_capture()
            raise
        finally:
            test_pattern.restore_gcode_state()
        junctions: list[Junction] = []
        fit_results: list[FitResult] = []
        analysis_error: str | None = None
        try:
            junctions = test_pattern.get_junction_list(trapq_moves)
            pa_analysis.normalize_force_polarity(junctions)
            fit_results = pa_analysis.estimate_pa(junctions)
        except Exception as ex:
            analysis_error = str(ex)
        report: PADataRecord = PADataRecord(
            speed=test_pattern.speed,
            accel=test_pattern.accel,
            flow_rate=test_pattern.flow_rate,
            fit_results=fit_results,
            junctions=junctions,
            time=pa_analysis.time,
            force=pa_analysis.force,
            trapq_moves=trapq_moves,
            analysis_error=analysis_error,
        )
        reporter: Reporter = Reporter(
            fit_results, junctions, output_path, analysis_error, report
        )
        reporter.save_json(gcmd)
        reporter.console_report(gcmd)
        return reporter

    def calibrate(self, gcmd: GCodeCommand):
        cmd_params: ParamsGrabber = ParamsGrabber(self._printer, gcmd)
        patterns: list[PATestPattern] = []
        x_direction = 1.0
        for speed, accel, advance in zip(cmd_params.speed, cmd_params.accel,
                cmd_params.advance):
            patterns.append(
                PATestPattern(
                    self._printer,
                    cmd_params,
                    speed,
                    accel,
                    advance,
                    gcmd.error,
                    x_direction=x_direction,
                )
            )
            x_direction = -1.0 * x_direction
        reports: list[Reporter] = []
        output_path: str = cmd_params.output_path
        for i, test_pattern in enumerate(patterns):
            report = self.single_pattern(gcmd, test_pattern, output_path)
            reports.append(report)
            if i < len(patterns) - 1:
                test_pattern.step_move()
        Reporter.adaptive_pa_report(gcmd, reports)
