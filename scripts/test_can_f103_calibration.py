# /// script
# requires-python = ">=3.12"
# dependencies = [
#   "python-can[gs_usb]>=4.6,<5",
# ]
# ///

"""Host tests for the STM32F103 calibration client."""

from __future__ import annotations

import unittest

import can
from can_f103_calibration import (
    CURRENT_HALL_GEOMETRY,
    CalibrationError,
    Status,
    arm_with_retry,
    build_hall_geometry,
    format_hall_geometry,
    parse_hall_geometry,
    start_routine_with_retry,
    stop_and_confirm,
    submit_stop_safely,
    validate_flux_hall_speed,
)


def status_message(data: bytes) -> can.Message:
    return can.Message(arbitration_id=0x2F7, is_extended_id=False, data=data)


class StatusTests(unittest.TestCase):
    def test_page_zero_exposes_every_arm_predicate(self) -> None:
        status = Status()
        message = can.Message(
            arbitration_id=0x2F7,
            is_extended_id=False,
            data=bytes([0xC0, 2, 0, 0, 0x34, 0x12, 0xF3, 0]),
        )
        self.assertTrue(status.update(message))
        self.assertTrue(status.armed)
        self.assertTrue(status.local_ready)
        self.assertTrue(status.hall_quiet)
        self.assertTrue(status.motor_outputs_disabled)
        self.assertTrue(status.can_rx_fifo_overrun)
        self.assertTrue(status.can_command_queue_drop)
        self.assertFalse(status.output_active)

    def test_pulse_positions_and_hall_speed_decode(self) -> None:
        status = Status()
        for data in (
            bytes([0xC0, 4, 0, 0, 0x34, 0x12, 0, 0]),
            bytes([0xC8, 25, 0, 53, 0, 53, 0, 27]),
            bytes([0xCB, 0x3C, 0x05, 0x70, 0x17, 0x48, 0x17, 93]),
        ):
            self.assertTrue(
                status.update(
                    can.Message(
                        arbitration_id=0x2F7,
                        is_extended_id=False,
                        data=data,
                    )
                )
            )
        self.assertEqual(status.residual_dead_time_uv, 25_000)
        self.assertEqual((status.pulse_step_d_ticks, status.pulse_step_q_ticks), (53, 53))
        self.assertEqual(status.last_pulse_di_counts, 27)
        self.assertEqual(status.flux_linkage_nwb, 13_400_000)
        self.assertEqual(status.flux_measurement_erpm, 6_000)
        self.assertEqual(status.hall_measurement_erpm, 5_960)
        self.assertEqual(status.sync_minimum_percent, 93)

    def test_other_schemas_are_not_decoded(self) -> None:
        status = Status()
        self.assertTrue(status.update(status_message(bytes([0xC0, 3, 0, 0, 0, 0, 0, 0]))))
        self.assertTrue(
            status.update(status_message(bytes([0xC8, 25, 0, 53, 0, 53, 0, 27])))
        )
        self.assertEqual(status.schema, 3)
        self.assertEqual(status.pulse_step_d_ticks, 0)
        self.assertNotIn(8, status.pages_seen)

    def test_page_zero_precedes_every_other_page(self) -> None:
        status = Status()
        pulse_page = can.Message(
            arbitration_id=0x2F7,
            is_extended_id=False,
            data=bytes([0xC8, 25, 0, 53, 0, 53, 0, 27]),
        )
        self.assertTrue(status.update(pulse_page))
        self.assertNotIn(8, status.pages_seen)
        self.assertTrue(
            status.update(
                can.Message(
                    arbitration_id=0x2F7,
                    is_extended_id=False,
                    data=bytes([0xC0, 4, 0, 0, 0x34, 0x12, 0, 0]),
                )
            )
        )
        self.assertTrue(status.update(pulse_page))
        self.assertEqual(status.pulse_step_d_ticks, 53)
        self.assertIn(8, status.pages_seen)

    def test_directional_hall_and_pulse_grid_decode(self) -> None:
        status = Status()
        pages = (
            bytes([0xC0, 4, 0, 0, 0x34, 0x12, 0, 0]),
            bytes([0xC9, 0xE8, 0xFD, 0x20, 0x4E, 0x30, 0x75, 100]),
            bytes([0xCA, 0x40, 0x9C, 0x50, 0xC3, 0x60, 0xEA, 0x7E]),
            bytes([0xCD, 0xE8, 0x03, 0x84, 0x4E, 0x94, 0x75, 90]),
            bytes([0xCE, 0xA4, 0x9C, 0xB4, 0xC3, 0xC4, 0xEA, 0x7E]),
            bytes([0xCF, 7, 12, 40, 0, 18, 19, 1]),
        )
        for data in pages:
            self.assertTrue(
                status.update(
                    can.Message(
                        arbitration_id=0x2F7,
                        is_extended_id=False,
                        data=data,
                    )
                )
            )

        self.assertEqual(status.hall_centers_q16[1], 232)
        self.assertEqual(status.hall_forward_minimum_samples, 100)
        self.assertEqual(status.hall_reverse_minimum_samples, 90)
        self.assertEqual(status.hall_minimum_samples, 90)
        self.assertEqual(status.hall_valid_mask, 0x7E)
        diagnostic = status.pulse_diagnostics[7]
        self.assertEqual(diagnostic.position_q8, 128)
        self.assertEqual(diagnostic.pulse_step_ticks, 40)
        self.assertEqual(diagnostic.average_di_counts, 18)
        self.assertEqual(diagnostic.inductance_nwb_per_count, 2_750)


class HallGeometryTests(unittest.TestCase):
    def test_source_parser_ignores_struct_field_types(self) -> None:
        source = """
pub struct MotorConfig {
    pub hall_sequence: [u8; 6],
    pub hall_boundaries_electrical_millidegrees: [u32; 6],
    pub forward_hall_direction: i8,
}
pub const BIKE_CONFIG: BikeConfig = BikeConfig {
    motor: MotorConfig {
        hall_sequence: [5, 1, 3, 2, 6, 4],
        hall_boundaries_electrical_millidegrees: [
            30_125, 84_683, 145_448, 209_877, 273_384, 334_396,
        ],
        forward_hall_direction: -1,
    },
};
"""
        geometry = parse_hall_geometry(source)
        self.assertEqual(geometry.electrical_states, (5, 1, 3, 2, 6, 4))
        self.assertEqual(
            geometry.boundaries_q16,
            (5_484, 15_416, 26_478, 38_207, 49_768, 60_875),
        )
        self.assertEqual(geometry.positive_angle_direction, -1)

    def test_centers_compile_to_rotation_ordered_boundaries(self) -> None:
        centers = [0, 20_000, 40_000, 30_000, 60_000, 10_000, 50_000, 0]
        geometry = build_hall_geometry(centers, 0x7E, CURRENT_HALL_GEOMETRY)
        self.assertEqual(geometry.electrical_states, (5, 1, 3, 2, 6, 4))
        self.assertEqual(geometry.boundaries_q16, (2_232, 15_000, 25_000, 35_000, 45_000, 55_000))
        self.assertEqual(geometry.positive_angle_direction, -1)
        self.assertIn("HallGeometry::new(", format_hall_geometry(geometry))

    def test_reversed_hall_order_is_rejected(self) -> None:
        centers = [0, 50_000, 30_000, 40_000, 10_000, 60_000, 20_000, 0]
        with self.assertRaisesRegex(CalibrationError, "cyclic order"):
            build_hall_geometry(centers, 0x7E, CURRENT_HALL_GEOMETRY)


class FluxHallSpeedTests(unittest.TestCase):
    def test_close_hall_speed_is_accepted(self) -> None:
        validate_flux_hall_speed(
            Status(flux_measurement_erpm=6_000, hall_measurement_erpm=5_960)
        )

    def test_missing_or_slipping_hall_speed_is_rejected(self) -> None:
        for hall_erpm in (0, 4_500):
            with self.subTest(hall_erpm=hall_erpm):
                with self.assertRaisesRegex(CalibrationError, "Hall speed"):
                    validate_flux_hall_speed(
                        Status(
                            flux_measurement_erpm=6_000,
                            hall_measurement_erpm=hall_erpm,
                        )
                    )


class FakeArmClient:
    def __init__(self) -> None:
        self.status = Status(flags=0x32)
        self.sent: list[can.Message] = []
        self.receives = 0

    def send(self, message: can.Message) -> None:
        self.sent.append(message)

    def receive_until(self, predicate, timeout: float) -> Status:
        self.receives += 1
        if self.receives == 1:
            raise CalibrationError("retry")
        self.status.flags |= 1
        if predicate(self.status):
            return self.status
        raise AssertionError("arm predicate rejected an armed status")


class FakeStopClient:
    def __init__(self) -> None:
        self.sent: list[can.Message] = []

    def send(self, message: can.Message) -> None:
        self.sent.append(message)


class FakeStopStatusClient:
    """Replays status frames after STOP, starting from an empty Status."""

    def __init__(self, frames: list[bytes]) -> None:
        self.status = Status()
        self.frames = frames
        self.sent: list[can.Message] = []

    def send(self, message: can.Message) -> None:
        self.sent.append(message)

    def receive_until(self, predicate, timeout: float) -> Status:
        while self.frames:
            if self.status.update(status_message(self.frames.pop(0))) and predicate(
                self.status
            ):
                return self.status
        raise CalibrationError("timed out")


class FakeRunClient:
    def __init__(self) -> None:
        self.status = Status(routine=1, state=8, page_zero_updates=10)
        self.sent: list[can.Message] = []
        self.receives = 0

    def send(self, message: can.Message) -> None:
        self.sent.append(message)

    def receive_until(self, predicate, timeout: float) -> Status:
        self.receives += 1
        if self.receives == 1:
            raise CalibrationError("retry")
        self.status.routine = 2
        self.status.state = 2
        self.status.page_zero_updates += 1
        if predicate(self.status):
            return self.status
        raise AssertionError("routine predicate rejected a new active routine")


class FakeRerunClient:
    """Reports queued page 0 snapshots of (routine, state, failure)."""

    def __init__(
        self, initial: tuple[int, int, int], pages: list[tuple[int, int, int]]
    ) -> None:
        routine, state, failure = initial
        self.status = Status(
            routine=routine, state=state, failure=failure, page_zero_updates=10
        )
        self.pages = list(pages)
        self.sent: list[can.Message] = []

    def send(self, message: can.Message) -> None:
        self.sent.append(message)

    def receive_until(self, predicate, timeout: float) -> Status:
        while self.pages:
            routine, state, failure = self.pages.pop(0)
            self.status.routine = routine
            self.status.state = state
            self.status.failure = failure
            self.status.page_zero_updates += 1
            if predicate(self.status):
                return self.status
        raise CalibrationError("timed out")


class RecoveryTests(unittest.TestCase):
    def test_arm_retries_until_the_firmware_acknowledges(self) -> None:
        client = FakeArmClient()
        status = arm_with_retry(client, 0x1234, timeout=1.0, retry_interval=0.01)
        self.assertTrue(status.armed)
        self.assertEqual(len(client.sent), 2)
        self.assertTrue(all(bytes(frame.data[:4]) == b"ARMC" for frame in client.sent))

    def test_failure_stop_uses_the_immediate_stop_frame(self) -> None:
        client = FakeStopClient()
        self.assertTrue(submit_stop_safely(client))
        self.assertEqual(len(client.sent), 1)
        self.assertEqual(bytes(client.sent[0].data), b"STOP")
        self.assertFalse(client.sent[0].is_rx)

    def test_stop_waits_for_page_zero_with_outputs_inactive(self) -> None:
        client = FakeStopStatusClient(
            [
                bytes([0xC3, 0, 0, 0, 0, 0, 0, 0]),
                bytes([0xC0, 4, 3, 0, 0, 0, 0x04, 1]),
                bytes([0xC5, 0, 0, 0, 0, 0, 0, 0]),
            ]
        )
        with self.assertRaises(CalibrationError):
            stop_and_confirm(client, timeout=1.0)
        self.assertEqual(bytes(client.sent[0].data), b"STOP")

        client = FakeStopStatusClient(
            [
                bytes([0xC0, 4, 3, 0, 0, 0, 0x04, 1]),
                bytes([0xC0, 4, 9, 1, 0, 0, 0x20, 1]),
            ]
        )
        status = stop_and_confirm(client, timeout=1.0)
        self.assertFalse(status.output_active)
        self.assertEqual(status.state, 9)
        self.assertEqual(status.page_zero_updates, 2)

    def test_run_retries_until_a_new_active_routine_is_reported(self) -> None:
        client = FakeRunClient()
        status = start_routine_with_retry(
            client,
            b"RUNL",
            0x1234,
            routine=2,
            complete_state=9,
            failed_state=10,
            timeout=1.0,
            retry_interval=0.01,
        )
        self.assertEqual((status.routine, status.state), (2, 2))
        self.assertEqual(len(client.sent), 2)
        self.assertTrue(all(bytes(frame.data[:4]) == b"RUNL" for frame in client.sent))

    def test_rerun_ignores_the_failed_status_reported_before_run(self) -> None:
        client = FakeRerunClient(
            initial=(2, 10, 4),
            pages=[(2, 10, 4), (2, 2, 0)],
        )
        status = start_routine_with_retry(
            client,
            b"RUNL",
            0x1234,
            routine=2,
            complete_state=9,
            failed_state=10,
            timeout=1.0,
            retry_interval=0.01,
        )
        self.assertEqual((status.routine, status.state, status.failure), (2, 2, 0))

    def test_rerun_reports_a_new_failure_after_run(self) -> None:
        client = FakeRerunClient(initial=(2, 10, 4), pages=[(2, 10, 4), (2, 10, 6)])
        with self.assertRaisesRegex(CalibrationError, "calibration failed"):
            start_routine_with_retry(
                client,
                b"RUNL",
                0x1234,
                routine=2,
                complete_state=9,
                failed_state=10,
                timeout=1.0,
                retry_interval=0.01,
            )


    def test_rerun_reports_an_identical_failure_on_the_second_page_after_run(
        self,
    ) -> None:
        client = FakeRerunClient(initial=(3, 7, 11), pages=[(3, 7, 11), (3, 7, 11)])
        with self.assertRaisesRegex(CalibrationError, "missing prerequisite"):
            start_routine_with_retry(
                client,
                b"RUNF",
                0x1234,
                routine=3,
                complete_state=6,
                failed_state=7,
                timeout=1.0,
                retry_interval=0.01,
            )
        self.assertEqual(len(client.sent), 1)


if __name__ == "__main__":
    unittest.main()
