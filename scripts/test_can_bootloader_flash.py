# /// script
# requires-python = ">=3.12"
# dependencies = [
#   "python-can[gs_usb]>=4.6,<5",
# ]
# ///

"""Host tests for the direct CAN bootloader client."""

from __future__ import annotations

import unittest
from collections import deque

import can
from can_bootloader_flash import (
    APPLICATION_BASE,
    APPLICATION_BYTES,
    AmbiguousTransferError,
    BootloaderClient,
    FirmwareValidationError,
    crc16_xmodem,
    decode_abort,
    firmware_segments,
    validate_firmware,
)


def valid_image() -> bytes:
    image = bytearray(b"\xff" * APPLICATION_BYTES)
    image[0:4] = (0x2000_1000).to_bytes(4, "little")
    image[4:8] = (APPLICATION_BASE + 0x131).to_bytes(4, "little")
    return bytes(image)


class FirmwareValidationTests(unittest.TestCase):
    def test_crc_matches_the_xmodem_check_value(self) -> None:
        self.assertEqual(crc16_xmodem(b"123456789"), 0x31C3)

    def test_accepts_the_fixed_flash_region_image(self) -> None:
        image = validate_firmware(valid_image())
        self.assertEqual(image.initial_stack_pointer, 0x2000_1000)
        self.assertEqual(image.reset_vector, APPLICATION_BASE + 0x131)
        self.assertEqual(image.crc16, crc16_xmodem(valid_image()))

    def test_rejects_nonstandard_length(self) -> None:
        with self.assertRaisesRegex(FirmwareValidationError, "26,200"):
            validate_firmware(valid_image()[:-1])

    def test_rejects_a_non_thumb_reset_vector(self) -> None:
        image = bytearray(valid_image())
        image[4:8] = (APPLICATION_BASE + 0x130).to_bytes(4, "little")
        with self.assertRaisesRegex(FirmwareValidationError, "Thumb"):
            validate_firmware(bytes(image))

    def test_rejects_a_reset_vector_outside_the_application_slot(self) -> None:
        image = bytearray(valid_image())
        image[4:8] = (0x0801_0001).to_bytes(4, "little")
        with self.assertRaisesRegex(FirmwareValidationError, "application slot"):
            validate_firmware(bytes(image))

    def test_rejects_a_reset_vector_in_the_erased_slot_tail(self) -> None:
        image = bytearray(valid_image())
        image[4:8] = (APPLICATION_BASE + APPLICATION_BYTES + 1).to_bytes(4, "little")
        with self.assertRaisesRegex(FirmwareValidationError, "transferred image"):
            validate_firmware(bytes(image))


class SegmentTests(unittest.TestCase):
    def test_full_final_segment_retains_all_seven_bytes(self) -> None:
        segments = list(firmware_segments(bytes(range(14))))
        self.assertEqual(
            segments, [bytes([0x00, *range(7)]), bytes([0x11, *range(7, 14)])]
        )

    def test_partial_final_segment_encodes_the_unused_byte_count(self) -> None:
        segments = list(firmware_segments(bytes(range(13))))
        self.assertEqual(segments[-1], bytes([0x13, *range(7, 13), 0]))

    def test_single_byte_image_uses_six_unused_bytes(self) -> None:
        self.assertEqual(
            list(firmware_segments(b"\xa5")),
            [b"\x0d\xa5\x00\x00\x00\x00\x00\x00"],
        )

    def test_every_can_segment_has_dlc_eight(self) -> None:
        self.assertTrue(
            all(len(segment) == 8 for segment in firmware_segments(bytes(29)))
        )


class ResponseTests(unittest.TestCase):
    def test_decodes_an_abort_response(self) -> None:
        message = can.Message(
            arbitration_id=0x5FF,
            is_extended_id=False,
            data=bytes.fromhex("80501f0100000305"),
        )
        abort = decode_abort(message)
        self.assertIsNotNone(abort)
        assert abort is not None
        self.assertEqual(abort.index, 0x1F50)
        self.assertEqual(abort.subindex, 1)
        self.assertEqual(abort.code, 0x0503_0000)

    def test_ignores_non_abort_frames(self) -> None:
        message = can.Message(
            arbitration_id=0x5FF,
            is_extended_id=False,
            data=bytes.fromhex("60501f0100000000"),
        )
        self.assertIsNone(decode_abort(message))


class NullCapture:
    def record(self, message: can.Message) -> None:
        pass


class FakeBus:
    def __init__(self, responses: list[can.Message]) -> None:
        self.responses = deque(responses)
        self.sent: list[can.Message] = []

    def send(self, message: can.Message, timeout: float | None = None) -> None:
        self.sent.append(message)

    def recv(self, timeout: float | None = None) -> can.Message | None:
        return self.responses.popleft() if self.responses else None


class StopAndWaitTests(unittest.TestCase):
    def test_download_waits_for_the_exact_segment_ack(self) -> None:
        bus = FakeBus(
            [
                can.Message(
                    arbitration_id=0x5FF,
                    is_extended_id=False,
                    data=b"\x2d" + b"\x00" * 7,
                )
            ]
        )
        client = BootloaderClient(bus, NullCapture())  # type: ignore[arg-type]
        client.download(b"\xa5", segment_timeout=0.01, final_timeout=0.01)
        self.assertEqual(bytes(bus.sent[0].data), b"\x0d\xa5" + b"\x00" * 6)

    def test_timeout_never_retransmits_an_ambiguous_segment(self) -> None:
        bus = FakeBus([])
        client = BootloaderClient(bus, NullCapture())  # type: ignore[arg-type]
        with self.assertRaises(AmbiguousTransferError):
            client.download(b"\xa5", segment_timeout=0.001, final_timeout=0.001)
        self.assertEqual(len(bus.sent), 1)


if __name__ == "__main__":
    unittest.main()
