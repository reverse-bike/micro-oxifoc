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
    start_routine_with_retry,
    submit_stop_safely,
    validate_flux_hall_speed,
)


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

    def test_schema_three_decodes_pulse_positions_and_hall_speed(self) -> None:
        status = Status()
        for data in (
            bytes([0xC0, 3, 0, 0, 0x34, 0x12, 0, 0]),
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

    def test_schema_dependent_page_waits_for_page_zero(self) -> None:
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
                    data=bytes([0xC0, 3, 0, 0, 0x34, 0x12, 0, 0]),
                )
            )
        )
        self.assertTrue(status.update(pulse_page))
        self.assertEqual(status.pulse_step_d_ticks, 53)
        self.assertIn(8, status.pages_seen)

    def test_schema_four_decodes_directional_hall_and_pulse_grid(self) -> None:
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


if __name__ == "__main__":
    unittest.main()
