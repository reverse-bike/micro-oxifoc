# /// script
# requires-python = ">=3.12"
# dependencies = [
#   "python-can[gs_usb]>=4.6,<5",
# ]
# ///

"""Inspect or explicitly run the STM32F103 calibration firmware over CAN."""

from __future__ import annotations

import argparse
import re
import sys
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from pathlib import Path

import can
from can_bootloader_flash import normalized_channel, prepare_gs_usb_backend

BITRATE = 250_000
COMMAND_ID = 0x2F2
STATUS_ID = 0x2F7
SCHEMA = 3
TRAILER = bytes.fromhex("a55a")

RESISTANCE_STATE_NAMES = {
    0: "idle",
    1: "ramp-low",
    2: "settle-low",
    3: "sample-low",
    4: "ramp-high",
    5: "settle-high",
    6: "sample-high",
    7: "ramp-down",
    8: "complete",
    9: "failed",
}

INDUCTANCE_STATE_NAMES = {
    0: "idle",
    1: "ramp-hold",
    2: "settle-hold",
    3: "sample-hold",
    4: "discharge",
    5: "pulse-command",
    6: "pulse-scan",
    7: "ramp-down",
    8: "axis-pause",
    9: "complete",
    10: "failed",
}

FLUX_STATE_NAMES = {
    0: "idle",
    1: "capture",
    2: "speed-ramp",
    3: "settle",
    4: "sample",
    5: "ramp-down",
    6: "complete",
    7: "failed",
}

HALL_STATE_NAMES = {
    0: "idle",
    1: "ramp-up",
    2: "settle",
    3: "sweep",
    4: "ramp-down",
    5: "complete",
    6: "failed",
}

ROUTINE_NAMES = {
    0: "none",
    1: "resistance",
    2: "inductance",
    3: "flux-linkage",
    4: "hall",
}

FAILURE_NAMES = {
    0: "none",
    1: "stopped",
    2: "local interlock",
    3: "current sample",
    4: "phase overcurrent",
    5: "hardware fault",
    6: "PWM output",
    7: "control timing",
    8: "current did not settle",
    9: "invalid resistance slope",
    10: "bus voltage",
    11: "missing prerequisite",
    12: "pulse response",
    13: "inductance range",
    14: "motor not responding",
    15: "flux range",
    16: "Hall states",
}

STATUS_PAGE_COUNT = 15


class CalibrationError(RuntimeError):
    pass


@dataclass(frozen=True)
class HallGeometry:
    electrical_states: tuple[int, ...]
    boundaries_q16: tuple[int, ...]
    positive_angle_direction: int


def load_hall_geometry(path: Path) -> HallGeometry:
    source = path.read_text()
    match = re.search(
        r"pub static HALL_GEOMETRY:\s*HallGeometry\s*=\s*HallGeometry::new\("
        r"\s*\[([^]]+)\],\s*\[([^]]+)\],\s*(-?1)\s*,?\s*\)",
        source,
        re.DOTALL,
    )
    if match is None:
        raise CalibrationError(f"could not read HALL_GEOMETRY from {path}")

    def integers(value: str) -> tuple[int, ...]:
        return tuple(
            int(part.strip().replace("_", ""))
            for part in value.split(",")
            if part.strip()
        )

    geometry = HallGeometry(integers(match[1]), integers(match[2]), int(match[3]))
    if len(geometry.electrical_states) != 6 or len(geometry.boundaries_q16) != 6:
        raise CalibrationError(f"invalid HALL_GEOMETRY in {path}")
    return geometry


CURRENT_HALL_GEOMETRY = load_hall_geometry(
    Path(__file__).resolve().parents[1] / "oxifoc-f103" / "src" / "config.rs"
)


def circular_delta_q16(left: int, right: int) -> int:
    return (left - right + 32_768) % 65_536 - 32_768


def cyclically_equal(left: Sequence[int], right: Sequence[int]) -> bool:
    if len(left) != len(right):
        return False
    doubled = tuple(right) * 2
    return any(tuple(left) == doubled[offset : offset + len(left)] for offset in range(len(left)))


def build_hall_geometry(
    centers_q16: Sequence[int],
    valid_mask: int,
    current: HallGeometry = CURRENT_HALL_GEOMETRY,
) -> HallGeometry:
    if valid_mask != 0x7E or len(centers_q16) != 8:
        raise CalibrationError("Hall centers do not contain exactly raw states 1 through 6")
    ordered_centers = sorted((centers_q16[raw] & 0xFFFF, raw) for raw in range(1, 7))
    observed_order = tuple(raw for _, raw in ordered_centers)
    if not cyclically_equal(observed_order, current.electrical_states):
        raise CalibrationError(
            "calibrated Hall cyclic order does not match the current hardware geometry: "
            f"{observed_order}"
        )

    boundary_by_raw: list[tuple[int, int]] = []
    for index, (center, raw) in enumerate(ordered_centers):
        previous = ordered_centers[index - 1][0]
        spacing = (center - previous) % 65_536
        if not 5_461 <= spacing <= 16_384:
            raise CalibrationError(
                f"Hall center spacing for raw state {raw} is outside 30..90 degrees"
            )
        boundary_by_raw.append(((previous + spacing // 2) % 65_536, raw))

    boundary_by_raw.sort()
    geometry = HallGeometry(
        electrical_states=tuple(raw for _, raw in boundary_by_raw),
        boundaries_q16=tuple(boundary for boundary, _ in boundary_by_raw),
        positive_angle_direction=current.positive_angle_direction,
    )
    if not cyclically_equal(geometry.electrical_states, current.electrical_states):
        raise CalibrationError("Hall boundary conversion changed the calibrated cyclic order")

    maximum_center_error = 0
    for index, raw in enumerate(geometry.electrical_states):
        boundary = geometry.boundaries_q16[index]
        next_boundary = geometry.boundaries_q16[(index + 1) % 6]
        width = (next_boundary - boundary) % 65_536
        if not 5_461 <= width <= 16_384:
            raise CalibrationError(
                f"Hall sector width for raw state {raw} is outside 30..90 degrees"
            )
        reconstructed = (boundary + width // 2) % 65_536
        maximum_center_error = max(
            maximum_center_error,
            abs(circular_delta_q16(reconstructed, centers_q16[raw])),
        )
    if maximum_center_error > 2_731:
        raise CalibrationError("Hall boundary conversion moves a center by more than 15 degrees")
    return geometry


def format_hall_geometry(geometry: HallGeometry) -> str:
    states = ", ".join(str(value) for value in geometry.electrical_states)
    boundaries = ", ".join(f"{value:_}" for value in geometry.boundaries_q16)
    return (
        "HallGeometry::new(\n"
        f"    [{states}],\n"
        f"    [{boundaries}],\n"
        f"    {geometry.positive_angle_direction},\n"
        ")"
    )


def hall_geometry_deltas_degrees(
    candidate: HallGeometry, current: HallGeometry = CURRENT_HALL_GEOMETRY
) -> list[float]:
    candidate_by_raw = dict(zip(candidate.electrical_states, candidate.boundaries_q16, strict=True))
    current_by_raw = dict(zip(current.electrical_states, current.boundaries_q16, strict=True))
    return [
        circular_delta_q16(candidate_by_raw[raw], current_by_raw[raw]) * 360 / 65_536
        for raw in current.electrical_states
    ]


def validate_flux_hall_speed(status: Status) -> None:
    commanded = abs(status.flux_measurement_erpm)
    measured = abs(status.hall_measurement_erpm)
    if commanded == 0 or measured == 0 or abs(measured - commanded) * 100 > commanded * 15:
        raise CalibrationError(
            "Hall speed does not confirm the flux-linkage command "
            f"({status.hall_measurement_erpm} measured vs "
            f"{status.flux_measurement_erpm} commanded eRPM)"
        )


@dataclass
class Status:
    schema: int = 0
    routine: int = 0
    state: int = 0
    failure: int = 0
    challenge: int = 0
    flags: int = 0
    low_current_counts: int = 0
    low_voltage_ticks: int = 0
    high_current_counts: int = 0
    high_voltage_ticks: int = 0
    effective_uv_per_count: int = 0
    nominal_resistance_milliohm: int = 0
    nominal_current_ma_per_count: int = 0
    target_d_counts: int = 0
    measured_d_counts: int = 0
    applied_d_ticks: int = 0
    maximum_phase_current_abs: int = 0
    bus_voltage_mv: int = 0
    offset_a: int = 0
    offset_b: int = 0
    environment_reasons: int = 0
    sample_progress: int = 0
    fault_flags: int = 0
    maximum_control_cycles: int = 0
    timing_overruns: int = 0
    firmware_version: tuple[int, int, int] = (0, 0, 0)
    inductance_d_nwb_per_count: int = 0
    inductance_q_nwb_per_count: int = 0
    residual_dead_time_uv: int = 0
    pulse_step_d_ticks: int = 0
    pulse_step_q_ticks: int = 0
    last_pulse_di_counts: int = 0
    proportional_d_q16: int = 0
    proportional_q_q16: int = 0
    integral_per_cycle_q16: int = 0
    gain_bus_voltage_mv: int = 0
    tuning_bandwidth_rad_s: int = 0
    flux_linkage_nwb: int = 0
    average_bemf_d_uv: int = 0
    average_bemf_q_uv: int = 0
    flux_measurement_erpm: int = 0
    hall_measurement_erpm: int = 0
    sync_minimum_percent: int = 0
    hall_centers_q16: list[int] = field(default_factory=lambda: [0] * 8)
    hall_valid_mask: int = 0
    hall_minimum_samples: int = 0
    page_zero_updates: int = 0
    pages_seen: set[int] = field(default_factory=set)

    @property
    def armed(self) -> bool:
        return bool(self.flags & 1)

    @property
    def local_ready(self) -> bool:
        return bool(self.flags & 2)

    @property
    def output_active(self) -> bool:
        return bool(self.flags & 4)

    @property
    def hall_quiet(self) -> bool:
        return bool(self.flags & 0x10)

    @property
    def motor_outputs_disabled(self) -> bool:
        return bool(self.flags & 0x20)

    @property
    def can_rx_fifo_overrun(self) -> bool:
        return bool(self.flags & 0x40)

    @property
    def can_command_queue_drop(self) -> bool:
        return bool(self.flags & 0x80)

    @property
    def pulse_step_ticks(self) -> int:
        """Compatibility name for schema-2 logs, whose pulse was the q-axis value."""
        return self.pulse_step_q_ticks

    def update(self, message: can.Message) -> bool:
        data = bytes(message.data)
        if (
            message.is_extended_id
            or message.arbitration_id != STATUS_ID
            or len(data) != 8
            or data[0] & 0xF0 != 0xC0
        ):
            return False
        page = data[0] & 0x0F
        if page >= STATUS_PAGE_COUNT:
            return False
        if page != 0 and self.schema == 0:
            return True
        self.pages_seen.add(page)
        if page == 0:
            self.page_zero_updates += 1
            self.schema = data[1]
            self.state = data[2]
            self.failure = data[3]
            self.challenge = int.from_bytes(data[4:6], "little")
            self.flags = data[6]
            self.routine = data[7]
        elif page == 1:
            self.low_current_counts = int.from_bytes(data[1:3], "little", signed=True)
            self.low_voltage_ticks = int.from_bytes(data[3:5], "little", signed=True)
            self.high_current_counts = int.from_bytes(data[5:7], "little", signed=True)
        elif page == 2:
            self.effective_uv_per_count = int.from_bytes(data[1:5], "little")
            self.nominal_resistance_milliohm = int.from_bytes(data[5:7], "little")
            self.nominal_current_ma_per_count = data[7]
        elif page == 3:
            self.target_d_counts = int.from_bytes(data[1:3], "little", signed=True)
            self.measured_d_counts = int.from_bytes(data[3:5], "little", signed=True)
            self.applied_d_ticks = int.from_bytes(data[5:7], "little", signed=True)
            self.maximum_phase_current_abs = data[7] * 4
        elif page == 4:
            self.bus_voltage_mv = int.from_bytes(data[1:3], "little")
            self.offset_a = int.from_bytes(data[3:5], "little")
            self.offset_b = int.from_bytes(data[5:7], "little")
            self.environment_reasons = data[7]
        elif page == 5:
            self.fault_flags = int.from_bytes(data[1:5], "little")
            self.maximum_control_cycles = int.from_bytes(data[5:7], "little")
            self.timing_overruns = data[7]
        elif page == 6:
            self.high_voltage_ticks = int.from_bytes(data[1:3], "little", signed=True)
            self.sample_progress = int.from_bytes(data[3:5], "little")
            self.firmware_version = (data[5], data[6], data[7])
        elif page == 7:
            self.inductance_d_nwb_per_count = int.from_bytes(data[1:5], "little")
            self.inductance_q_nwb_per_count = int.from_bytes(data[5:8], "little")
        elif page == 8:
            if self.schema >= 3:
                self.residual_dead_time_uv = int.from_bytes(data[1:3], "little") * 1_000
                self.pulse_step_d_ticks = int.from_bytes(
                    data[3:5], "little", signed=True
                )
                self.pulse_step_q_ticks = int.from_bytes(
                    data[5:7], "little", signed=True
                )
            else:
                self.residual_dead_time_uv = int.from_bytes(data[1:5], "little")
                self.pulse_step_q_ticks = int.from_bytes(
                    data[5:7], "little", signed=True
                )
            self.last_pulse_di_counts = int.from_bytes(data[7:8], "little", signed=True)
        elif page == 9:
            self.proportional_d_q16 = int.from_bytes(data[1:5], "little", signed=True)
            self.proportional_q_q16 = signed_i24(data[5:8])
        elif page == 10:
            self.integral_per_cycle_q16 = int.from_bytes(
                data[1:5], "little", signed=True
            )
            self.gain_bus_voltage_mv = int.from_bytes(data[5:7], "little")
            self.tuning_bandwidth_rad_s = data[7] * 10
        elif page == 11:
            if self.schema >= 3:
                self.flux_linkage_nwb = int.from_bytes(data[1:3], "little") * 10_000
                self.flux_measurement_erpm = int.from_bytes(
                    data[3:5], "little", signed=True
                )
                self.hall_measurement_erpm = int.from_bytes(
                    data[5:7], "little", signed=True
                )
            else:
                self.flux_linkage_nwb = int.from_bytes(data[1:5], "little")
                self.flux_measurement_erpm = int.from_bytes(
                    data[5:7], "little", signed=True
                )
            self.sync_minimum_percent = data[7]
        elif page == 12:
            self.average_bemf_d_uv = int.from_bytes(data[1:5], "little", signed=True)
            self.average_bemf_q_uv = signed_i24(data[5:8])
        elif page == 13:
            for raw in range(1, 4):
                offset = 1 + (raw - 1) * 2
                self.hall_centers_q16[raw] = int.from_bytes(
                    data[offset : offset + 2], "little"
                )
            self.hall_minimum_samples = data[7]
        else:
            for raw in range(4, 7):
                offset = 1 + (raw - 4) * 2
                self.hall_centers_q16[raw] = int.from_bytes(
                    data[offset : offset + 2], "little"
                )
            self.hall_valid_mask = data[7]
        return True


def signed_i24(data: bytes) -> int:
    return int.from_bytes(
        data + (b"\xff" if data[2] & 0x80 else b"\x00"), "little", signed=True
    )


def state_name(status: Status) -> str:
    names = {
        1: RESISTANCE_STATE_NAMES,
        2: INDUCTANCE_STATE_NAMES,
        3: FLUX_STATE_NAMES,
        4: HALL_STATE_NAMES,
    }.get(status.routine, RESISTANCE_STATE_NAMES)
    return names.get(status.state, str(status.state))


def command_frame(tag: bytes, challenge: int) -> can.Message:
    if len(tag) != 4:
        raise ValueError("calibration command tag must be four bytes")
    return can.Message(
        arbitration_id=COMMAND_ID,
        is_extended_id=False,
        is_rx=False,
        data=tag + challenge.to_bytes(2, "little") + TRAILER,
    )


def stop_frame() -> can.Message:
    return can.Message(
        arbitration_id=COMMAND_ID,
        is_extended_id=False,
        is_rx=False,
        data=b"STOP",
    )


def default_log_path() -> Path:
    stamp = time.strftime("%Y%m%d-%H%M%S")
    return Path(f"scratch/can_log_f103_calibration-{stamp}.log")


def print_status(status: Status) -> None:
    print(f"pages_seen                 {sorted(status.pages_seen)}")
    print(f"schema                     {status.schema}")
    print(f"firmware_version           {'.'.join(map(str, status.firmware_version))}")
    print(
        f"routine                    {ROUTINE_NAMES.get(status.routine, status.routine)}"
    )
    print(f"state                      {state_name(status)}")
    print(
        f"failure                    {FAILURE_NAMES.get(status.failure, status.failure)}"
    )
    print(f"challenge                  0x{status.challenge:04x}")
    print(f"armed                      {status.armed}")
    print(f"local_ready                {status.local_ready}")
    print(f"hall_quiet                {status.hall_quiet}")
    print(f"motor_outputs_disabled    {status.motor_outputs_disabled}")
    print(f"can_rx_fifo_overrun       {status.can_rx_fifo_overrun}")
    print(f"can_command_queue_drop    {status.can_command_queue_drop}")
    print(f"output_active              {status.output_active}")
    print(f"bus_voltage_mv             {status.bus_voltage_mv}")
    print(f"current_offsets            [{status.offset_a}, {status.offset_b}]")
    print(f"environment_reasons       0x{status.environment_reasons:02x}")
    print(f"target_d_counts            {status.target_d_counts}")
    print(f"measured_d_counts          {status.measured_d_counts}")
    print(f"applied_d_ticks            {status.applied_d_ticks}")
    print(f"maximum_phase_current_abs  {status.maximum_phase_current_abs}")
    print(
        f"low_point                  [{status.low_current_counts}, {status.low_voltage_ticks}]"
    )
    print(
        f"high_point                 [{status.high_current_counts}, {status.high_voltage_ticks}]"
    )
    print(f"effective_uv_per_count     {status.effective_uv_per_count}")
    print(f"nominal_resistance_mohm    {status.nominal_resistance_milliohm}")
    print(f"nominal_current_ma/count   {status.nominal_current_ma_per_count}")
    print(f"inductance_d_nwb/count     {status.inductance_d_nwb_per_count}")
    print(f"inductance_q_nwb/count     {status.inductance_q_nwb_per_count}")
    print(f"residual_dead_time_uv      {status.residual_dead_time_uv}")
    print(f"pulse_step_d_ticks         {status.pulse_step_d_ticks}")
    print(f"pulse_step_q_ticks         {status.pulse_step_q_ticks}")
    print(f"last_pulse_di_counts       {status.last_pulse_di_counts}")
    print(f"proportional_d_q16         {status.proportional_d_q16}")
    print(f"proportional_q_q16         {status.proportional_q_q16}")
    print(f"integral_per_cycle_q16     {status.integral_per_cycle_q16}")
    print(f"gain_bus_voltage_mv        {status.gain_bus_voltage_mv}")
    print(f"tuning_bandwidth_rad_s     {status.tuning_bandwidth_rad_s}")
    print(f"flux_linkage_mwb           {status.flux_linkage_nwb / 1_000_000:.4f}")
    print(
        f"average_bemf_uv            [{status.average_bemf_d_uv}, "
        f"{status.average_bemf_q_uv}]"
    )
    print(f"flux_measurement_erpm      {status.flux_measurement_erpm}")
    print(f"hall_measurement_erpm      {status.hall_measurement_erpm}")
    print(f"sync_minimum_percent       {status.sync_minimum_percent}")
    hall_degrees = [round(value * 360 / 65_536, 2) for value in status.hall_centers_q16]
    print(f"hall_centers_degrees       {hall_degrees}")
    print(f"hall_valid_mask            0x{status.hall_valid_mask:02x}")
    print(f"hall_minimum_samples       {status.hall_minimum_samples}")
    if status.hall_valid_mask == 0x7E:
        geometry = build_hall_geometry(
            status.hall_centers_q16,
            status.hall_valid_mask,
            CURRENT_HALL_GEOMETRY,
        )
        deltas = hall_geometry_deltas_degrees(geometry, CURRENT_HALL_GEOMETRY)
        print("hall_geometry_candidate")
        print(format_hall_geometry(geometry))
        print(
            "hall_boundary_delta_deg    "
            + str([round(delta, 2) for delta in deltas])
        )
    print(f"control_max_cycles         {status.maximum_control_cycles}")
    print(f"timing_overruns            {status.timing_overruns}")
    print(f"fault_flags                0x{status.fault_flags:08x}")


class Client:
    def __init__(self, bus: can.BusABC, logger: can.Logger) -> None:
        self.bus = bus
        self.logger = logger
        self.status = Status()

    def send(self, message: can.Message) -> None:
        self.bus.send(message, timeout=1.0)
        self.logger(message)
        print(f"sent {message.arbitration_id:03x}#{bytes(message.data).hex()}")

    def receive_until(
        self, predicate: Callable[[Status], bool], timeout: float
    ) -> Status:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            message = self.bus.recv(timeout=min(0.25, remaining))
            if message is None:
                continue
            self.logger(message)
            if self.status.update(message) and predicate(self.status):
                return self.status
        raise CalibrationError("timed out waiting for calibration telemetry")

    def receive_all_pages(self, timeout: float) -> Status:
        return self.receive_until(
            lambda status: status.pages_seen == set(range(STATUS_PAGE_COUNT)), timeout
        )


def arm_with_retry(
    client: Client,
    challenge: int,
    timeout: float,
    retry_interval: float = 0.1,
) -> Status:
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            status = client.status
            raise CalibrationError(
                "timed out waiting for arm acknowledgement "
                f"(local_ready={status.local_ready}, hall_quiet={status.hall_quiet}, "
                "motor_outputs_disabled="
                f"{status.motor_outputs_disabled}, "
                f"can_rx_fifo_overrun={status.can_rx_fifo_overrun}, "
                f"can_command_queue_drop={status.can_command_queue_drop}, "
                f"faults=0x{status.fault_flags:08x})"
            )
        client.send(command_frame(b"ARMC", challenge))
        try:
            return client.receive_until(
                lambda status: status.armed, min(retry_interval, remaining)
            )
        except CalibrationError:
            continue


def start_routine_with_retry(
    client: Client,
    tag: bytes,
    challenge: int,
    routine: int,
    complete_state: int,
    failed_state: int,
    timeout: float,
    retry_interval: float = 0.1,
) -> Status:
    page_zero_at_start = client.status.page_zero_updates
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            status = client.status
            raise CalibrationError(
                f"timed out waiting for {tag.decode()} acknowledgement "
                f"(routine={ROUTINE_NAMES.get(status.routine, status.routine)}, "
                f"state={state_name(status)}, "
                f"can_rx_fifo_overrun={status.can_rx_fifo_overrun}, "
                f"can_command_queue_drop={status.can_command_queue_drop})"
            )
        client.send(command_frame(tag, challenge))
        try:
            status = client.receive_until(
                lambda current: (
                    current.page_zero_updates != page_zero_at_start
                    and current.routine == routine
                    and current.state != complete_state
                ),
                min(retry_interval, remaining),
            )
        except CalibrationError:
            continue
        if status.state == failed_state:
            raise CalibrationError(
                "calibration failed: "
                f"{FAILURE_NAMES.get(status.failure, status.failure)}"
            )
        return status


def submit_stop_safely(client: Client) -> bool:
    try:
        client.send(stop_frame())
    except can.CanError:
        return False
    return True


def run_routine(
    client: Client,
    initial: Status,
    tag: bytes,
    routine: int,
    complete_state: int,
    failed_state: int,
    timeout: float,
) -> Status:
    arm_with_retry(client, initial.challenge, 6.0)
    started = start_routine_with_retry(
        client,
        tag,
        initial.challenge,
        routine,
        complete_state,
        failed_state,
        timeout=6.0,
    )
    last_update = started.page_zero_updates
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        status = client.receive_until(
            lambda current, previous=last_update: (
                current.routine == routine and current.page_zero_updates != previous
            ),
            3.0,
        )
        last_update = status.page_zero_updates
        print(f"{ROUTINE_NAMES[routine]} state {state_name(status)}")
        if status.state == failed_state:
            raise CalibrationError(
                f"calibration failed: {FAILURE_NAMES.get(status.failure, status.failure)}"
            )
        if status.state == complete_state:
            status.pages_seen.clear()
            return client.receive_all_pages(3.0)
    raise CalibrationError("calibration run timed out")


def run(args: argparse.Namespace) -> int:
    prepare_gs_usb_backend(args.interface)
    log_path = args.log or default_log_path()
    log_path.parent.mkdir(parents=True, exist_ok=True)
    logger = can.Logger(str(log_path))
    bus: can.BusABC | None = None
    client: Client | None = None
    run_authorized = False
    try:
        bus = can.Bus(
            interface=args.interface,
            channel=normalized_channel(args.interface, args.channel),
            bitrate=args.bitrate,
            can_filters=[{"can_id": STATUS_ID, "can_mask": 0x7FF, "extended": False}],
        )
        client = Client(bus, logger)
        if args.action == "stop":
            client.send(stop_frame())
            status = client.receive_until(lambda status: not status.output_active, 3.0)
            print_status(status)
        elif args.action == "status":
            print_status(client.receive_all_pages(args.timeout))
        else:
            initial = client.receive_all_pages(args.timeout)
            print_status(initial)
            if initial.schema != SCHEMA:
                raise CalibrationError(
                    f"expected calibration telemetry schema {SCHEMA}, got {initial.schema}"
                )
            if not args.yes:
                print(
                    "\nNothing was energized. Add --yes only with the wheel elevated and "
                    "the drivetrain clear."
                )
                return 0
            if not initial.local_ready:
                raise CalibrationError(
                    "firmware interlocks are not ready; check throttle rest, brake, bus, "
                    "faults, and motor motion"
                )
            run_authorized = True
            final = initial
            if args.action in ("resistance", "full"):
                final = run_routine(client, final, b"RUNR", 1, 8, 9, args.run_timeout)
            if args.action in ("inductance", "full"):
                if final.effective_uv_per_count == 0:
                    raise CalibrationError(
                        "inductance requires a successful resistance result from this boot"
                    )
                final = run_routine(client, final, b"RUNL", 2, 9, 10, args.run_timeout)
            if args.action in ("flux", "full"):
                if (
                    final.effective_uv_per_count == 0
                    or final.inductance_d_nwb_per_count == 0
                    or final.inductance_q_nwb_per_count == 0
                ):
                    raise CalibrationError(
                        "flux linkage requires successful resistance and "
                        "inductance results from this boot"
                    )
                final = run_routine(client, final, b"RUNF", 3, 6, 7, args.run_timeout)
                validate_flux_hall_speed(final)
            if args.action in ("hall", "full"):
                final = run_routine(client, final, b"RUNH", 4, 5, 6, args.run_timeout)
            print_status(final)
        print(f"CAN log                    {log_path}")
        return 0
    except (CalibrationError, can.CanError) as error:
        stopped = run_authorized and client is not None and submit_stop_safely(client)
        print(f"calibration error: {error}", file=sys.stderr)
        if stopped:
            print("STOP submitted after calibration error", file=sys.stderr)
        print(f"CAN log: {log_path}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        stopped = client is not None and submit_stop_safely(client)
        outcome = "STOP submitted" if stopped else "STOP could not be submitted"
        print(f"interrupted; {outcome}", file=sys.stderr)
        return 130
    finally:
        if bus is not None:
            bus.shutdown()
        logger.stop()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "action",
        choices=(
            "status",
            "resistance",
            "inductance",
            "flux",
            "hall",
            "full",
            "stop",
        ),
    )
    parser.add_argument("--interface", default="gs_usb")
    parser.add_argument("--channel", default="0")
    parser.add_argument("--bitrate", type=int, default=BITRATE)
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument("--run-timeout", type=float, default=30.0)
    parser.add_argument("--log", type=Path)
    parser.add_argument(
        "--yes",
        action="store_true",
        help="confirm a calibration run that energizes and aligns the motor",
    )
    return parser


if __name__ == "__main__":
    raise SystemExit(run(build_parser().parse_args()))
