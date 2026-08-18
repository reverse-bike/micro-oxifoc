# /// script
# requires-python = ">=3.12"
# dependencies = [
#   "python-can[gs_usb]>=4.6,<5",
# ]
# ///

"""Flash one validated controller application image through the resident CAN bootloader.

The transfer is deliberately stop-and-wait. A segment is never retransmitted after
an ambiguous acknowledgement because the controller treats duplicates as a toggle
error; rerun the command to erase staging and restart the whole transfer instead.
"""

from __future__ import annotations

import argparse
import hashlib
import math
import sys
import time
from collections.abc import Callable, Iterator
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import can

BITRATE = 250_000
REQUEST_ID = 0x67F
HANDSHAKE_ID = 0x5AA
RESPONSE_ID = 0x5FF
HEARTBEAT_ID = 0x77F
IDENTITY_ID = 0x211

APPLICATION_BASE = 0x0800_3800
APPLICATION_BYTES = 26_200
APPLICATION_SLOT_BYTES = 0xC800
SRAM_BASE = 0x2000_0000
SRAM_END = 0x2000_5000

HANDSHAKE = bytes.fromhex("aa552a002a0055aa")
CANCEL_TRANSFER = b"\x80"
ERASE_STAGING = bytes.fromhex("2f511f0103000000")
ERASE_ACK = bytes.fromhex("60511f0100000000")
READ_PENDING_MARKER = bytes.fromhex("40551f0400000000")
PENDING_MARKER_PREFIX = bytes.fromhex("4f551f04")
PENDING_MARKER = 0xAA55_AA55
INSTALL_AND_RESET = bytes.fromhex("2f511f0101000000")
IDENTITY_RESPONSE = b"05060002"

ABORT_NAMES = {
    0x0503_0000: "toggle or segment sequence mismatch",
    0x0601_0002: "attempted write to a read-only object",
    0x0606_0000: "staging flash program failure",
    0x0607_0012: "segment exceeded the declared image length",
    0x0608_0000: "staging erase requested without a bootloader handshake",
    0x0800_0000: "invalid segment command or erase/program failure",
}


def normalized_channel(interface: str, channel: str) -> int | str:
    if interface == "gs_usb" and channel.isdecimal():
        return int(channel)
    return channel


def prepare_gs_usb_backend(interface: str) -> None:
    if interface != "gs_usb":
        return

    import usb.core
    from gs_usb import GsUsb

    if sys.platform == "darwin":
        usb.core.Device.is_kernel_driver_active = lambda self, number: False

    original_from_sample_point = can.BitTiming.from_sample_point

    def from_sample_point(
        cls: type[can.BitTiming],
        f_clock: int,
        bitrate: int,
        sample_point: float = 69.0,
    ) -> can.BitTiming:
        if f_clock == 160_000_000 and bitrate == BITRATE:
            return cls(f_clock=f_clock, brp=40, tseg1=13, tseg2=2, sjw=2)
        return original_from_sample_point(f_clock, bitrate, sample_point)

    can.BitTiming.from_sample_point = classmethod(from_sample_point)

    # Candlelight uses the echo ID as a transmit-context slot. Rotate through
    # the ten slots used by Linux gs_usb while the receive loop drains their
    # completion echoes.
    if not hasattr(GsUsb, "_oxifoc_original_send"):
        original_send = GsUsb.send
        GsUsb._oxifoc_original_send = original_send

        def send_with_rotating_echo_id(self: GsUsb, frame: object) -> bool:
            echo_id = getattr(self, "_oxifoc_next_echo_id", 0)
            frame.echo_id = echo_id
            self._oxifoc_next_echo_id = (echo_id + 1) % 10
            return original_send(self, frame)

        GsUsb.send = send_with_rotating_echo_id


class UpdateError(RuntimeError):
    pass


class FirmwareValidationError(UpdateError):
    pass


class ResponseTimeout(UpdateError):
    pass


class AmbiguousTransferError(UpdateError):
    pass


@dataclass(frozen=True)
class FirmwareImage:
    payload: bytes
    initial_stack_pointer: int
    reset_vector: int
    crc16: int
    sha256: str


@dataclass(frozen=True)
class AbortResponse:
    index: int
    subindex: int
    code: int

    def __str__(self) -> str:
        meaning = ABORT_NAMES.get(self.code, "unknown abort")
        return (
            f"object 0x{self.index:04x}:{self.subindex:02x} aborted with "
            f"0x{self.code:08x} ({meaning})"
        )


class Capture:
    def __init__(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        self._writer = can.Logger(str(path))

    def record(self, message: can.Message) -> None:
        self._writer(message)

    def close(self) -> None:
        self._writer.stop()


def crc16_xmodem(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
    return crc


def validate_firmware(payload: bytes) -> FirmwareImage:
    if len(payload) != APPLICATION_BYTES:
        raise FirmwareValidationError(
            f"image is {len(payload):,} bytes; expected the fixed {APPLICATION_BYTES:,}-byte "
            "controller flash-region image"
        )
    initial_stack_pointer = int.from_bytes(payload[0:4], "little")
    reset_vector = int.from_bytes(payload[4:8], "little")
    if not SRAM_BASE <= initial_stack_pointer <= SRAM_END:
        raise FirmwareValidationError(
            f"initial stack pointer 0x{initial_stack_pointer:08x} is outside SRAM"
        )
    if initial_stack_pointer % 8:
        raise FirmwareValidationError(
            f"initial stack pointer 0x{initial_stack_pointer:08x} is not 8-byte aligned"
        )
    if reset_vector & 1 == 0:
        raise FirmwareValidationError(
            f"reset vector 0x{reset_vector:08x} does not select Thumb state"
        )
    reset_address = reset_vector & ~1
    if (
        not APPLICATION_BASE
        <= reset_address
        < APPLICATION_BASE + APPLICATION_SLOT_BYTES
    ):
        raise FirmwareValidationError(
            f"reset vector 0x{reset_vector:08x} is outside the resident bootloader's "
            "application slot"
        )
    if reset_address >= APPLICATION_BASE + len(payload):
        raise FirmwareValidationError(
            f"reset vector 0x{reset_vector:08x} is outside the transferred image; "
            "the resident installer erases the remainder of the application slot"
        )
    return FirmwareImage(
        payload=payload,
        initial_stack_pointer=initial_stack_pointer,
        reset_vector=reset_vector,
        crc16=crc16_xmodem(payload),
        sha256=hashlib.sha256(payload).hexdigest(),
    )


def load_firmware(path: Path) -> FirmwareImage:
    try:
        return validate_firmware(path.read_bytes())
    except OSError as error:
        raise FirmwareValidationError(f"cannot read {path}: {error}") from error


def firmware_segments(payload: bytes) -> Iterator[bytes]:
    if not payload:
        raise ValueError("firmware transfer cannot be empty")
    for index, offset in enumerate(range(0, len(payload), 7)):
        chunk = payload[offset : offset + 7]
        final = offset + len(chunk) == len(payload)
        toggle = (index & 1) << 4
        unused = 7 - len(chunk) if final else 0
        command = toggle | (unused << 1) | int(final)
        yield bytes([command]) + chunk + bytes(7 - len(chunk))


def decode_abort(message: can.Message) -> AbortResponse | None:
    data = bytes(message.data)
    if (
        message.is_extended_id
        or message.arbitration_id != RESPONSE_ID
        or len(data) != 8
        or data[0] != 0x80
    ):
        return None
    return AbortResponse(
        index=int.from_bytes(data[1:3], "little"),
        subindex=data[3],
        code=int.from_bytes(data[4:8], "little"),
    )


def exact_frame(can_id: int, payload: bytes) -> Callable[[can.Message], bool]:
    def matches(message: can.Message) -> bool:
        return (
            not message.is_extended_id
            and message.arbitration_id == can_id
            and bytes(message.data) == payload
        )

    return matches


class BootloaderClient:
    def __init__(self, bus: can.BusABC, capture: Capture) -> None:
        self.bus = bus
        self.capture = capture
        self.transfer_active = False

    def send(self, payload: bytes, can_id: int = REQUEST_ID) -> None:
        message = can.Message(
            arbitration_id=can_id,
            is_extended_id=False,
            is_rx=False,
            data=payload,
        )
        self.bus.send(message, timeout=1.0)
        self.capture.record(message)

    def receive(self, deadline: float) -> can.Message | None:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None
        message = self.bus.recv(timeout=remaining)
        if message is not None:
            self.capture.record(message)
        return message

    def wait_for(
        self,
        description: str,
        matcher: Callable[[can.Message], bool],
        timeout: float,
    ) -> can.Message:
        deadline = time.monotonic() + timeout
        while message := self.receive(deadline):
            abort = decode_abort(message)
            if abort is not None:
                raise UpdateError(str(abort))
            if matcher(message):
                return message
        raise ResponseTimeout(f"timed out waiting for {description}")

    def settle(self, seconds: float = 0.25) -> None:
        deadline = time.monotonic() + seconds
        while self.receive(deadline) is not None:
            pass

    def enter_bootloader(self, timeout: float) -> None:
        # This makes rerunning after an interrupted transfer self-recovering. It
        # is ignored by the application and has no response in the bootloader.
        self.send(CANCEL_TRANSFER)
        deadline = time.monotonic() + timeout
        next_request = 0.0
        matcher = exact_frame(HANDSHAKE_ID, HANDSHAKE)
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_request:
                self.send(HANDSHAKE)
                next_request = now + 0.1
            receive_deadline = min(deadline, next_request)
            message = self.receive(receive_deadline)
            if message is not None and matcher(message):
                print("bootloader handshake acknowledged")
                return
        raise ResponseTimeout(
            "bootloader handshake was not acknowledged; ensure the wheel is stationary, "
            "the throttle is released, the bridge is off, and no other CAN client owns "
            "the adapter"
        )

    def erase_staging(self, timeout: float) -> None:
        self.send(ERASE_STAGING)
        self.wait_for(
            "staging erase acknowledgement",
            exact_frame(RESPONSE_ID, ERASE_ACK),
            timeout,
        )
        print("staging region erased")

    def begin_download(self, length: int, timeout: float) -> None:
        request = bytes.fromhex("21501f01") + length.to_bytes(4, "little")
        self.send(request)
        self.wait_for(
            "download-init acknowledgement",
            exact_frame(RESPONSE_ID, bytes.fromhex("60501f0100000000")),
            timeout,
        )
        self.transfer_active = True
        print(f"download accepted: {length:,} bytes")

    def download(
        self, payload: bytes, segment_timeout: float, final_timeout: float
    ) -> None:
        segments = list(firmware_segments(payload))
        total = len(segments)
        report_interval = max(1, total // 100)
        started = time.monotonic()
        for number, segment in enumerate(segments, start=1):
            self.send(segment)
            acknowledgement = bytes([segment[0] | 0x20]) + b"\x00" * 7
            timeout = final_timeout if number == total else segment_timeout
            try:
                self.wait_for(
                    f"segment {number}/{total} acknowledgement",
                    exact_frame(RESPONSE_ID, acknowledgement),
                    timeout,
                )
            except ResponseTimeout as error:
                raise AmbiguousTransferError(
                    f"segment {number}/{total} has ambiguous delivery; it was not "
                    "retransmitted because duplicates invalidate the bootloader session"
                ) from error
            if number == 1 or number == total or number % report_interval == 0:
                percent = number * 100 / total
                print(f"download {percent:5.1f}% ({number:,}/{total:,} segments)")
        self.transfer_active = False
        elapsed = time.monotonic() - started
        print(
            f"download complete in {elapsed:.2f} seconds ({len(payload) / elapsed:,.0f} B/s)"
        )

    def verify_pending_marker(self, timeout: float, attempts: int = 3) -> None:
        for attempt in range(1, attempts + 1):
            self.send(READ_PENDING_MARKER)
            try:
                message = self.wait_for(
                    "pending-image marker",
                    lambda candidate: (
                        not candidate.is_extended_id
                        and candidate.arbitration_id == RESPONSE_ID
                        and len(candidate.data) == 8
                        and bytes(candidate.data[:4]) == PENDING_MARKER_PREFIX
                    ),
                    timeout,
                )
            except ResponseTimeout:
                if attempt == attempts:
                    raise
                continue
            marker = int.from_bytes(message.data[4:8], "little")
            if marker != PENDING_MARKER:
                raise UpdateError(
                    f"bootloader returned pending marker 0x{marker:08x}, expected "
                    f"0x{PENDING_MARKER:08x}"
                )
            print(f"pending-image marker verified on attempt {attempt}")
            return

    def cancel_transfer(self) -> None:
        for _ in range(3):
            try:
                self.send(CANCEL_TRANSFER)
            except can.CanError:
                pass
            time.sleep(0.05)
        self.transfer_active = False

    def install_and_reset(self) -> None:
        # The bootloader resets immediately after queueing its optional response.
        # Sending this command more than once risks resetting the next boot too.
        self.send(INSTALL_AND_RESET)
        print(
            "install/reset requested once; a missing reset acknowledgement is expected"
        )

    def wait_for_application(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        next_query = time.monotonic() + 0.75
        matcher = exact_frame(IDENTITY_ID, IDENTITY_RESPONSE)
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_query:
                self.send(b"", IDENTITY_ID)
                next_query = now + 0.5
            message = self.receive(min(deadline, next_query))
            if message is not None and matcher(message):
                print("installed application answered the stock identity query")
                return
        raise ResponseTimeout(
            "installed application did not answer 0x211 before the post-install timeout"
        )


def default_log_path() -> Path:
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    return Path(f"scratch/can_log_bootloader_flash-{stamp}.log")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "image", type=Path, help="26,200-byte controller flash-region bin"
    )
    parser.add_argument("--interface", default="gs_usb")
    parser.add_argument("--channel", default="0")
    parser.add_argument("--bitrate", type=int, default=BITRATE)
    parser.add_argument("--log", type=Path)
    parser.add_argument("--boot-timeout", type=float, default=6.0)
    parser.add_argument("--erase-timeout", type=float, default=5.0)
    parser.add_argument("--control-timeout", type=float, default=2.0)
    parser.add_argument("--segment-timeout", type=float, default=2.0)
    parser.add_argument("--final-timeout", type=float, default=6.0)
    parser.add_argument("--post-install-timeout", type=float, default=12.0)
    parser.add_argument(
        "--yes",
        action="store_true",
        help="confirm replacement of the installed controller application",
    )
    return parser


def print_image_summary(path: Path, image: FirmwareImage) -> None:
    print(f"image                    {path}")
    print(f"bytes                    {len(image.payload):,}")
    print(f"initial stack pointer    0x{image.initial_stack_pointer:08x}")
    print(f"reset vector             0x{image.reset_vector:08x}")
    print(f"CRC-16/XMODEM            0x{image.crc16:04x}")
    print(f"SHA-256                  {image.sha256}")
    print(f"segments                 {math.ceil(len(image.payload) / 7):,}")


def run(args: argparse.Namespace) -> int:
    image = load_firmware(args.image)
    print_image_summary(args.image, image)
    if not args.yes:
        print(
            "\nValidation passed; nothing was transmitted. Add --yes to flash this image."
        )
        return 0

    prepare_gs_usb_backend(args.interface)
    log_path = args.log or default_log_path()
    capture = Capture(log_path)
    filters = [
        {"can_id": can_id, "can_mask": 0x7FF, "extended": False}
        for can_id in (HANDSHAKE_ID, RESPONSE_ID, HEARTBEAT_ID, IDENTITY_ID)
    ]
    client: BootloaderClient | None = None
    bus: can.BusABC | None = None
    try:
        bus = can.Bus(
            interface=args.interface,
            channel=normalized_channel(args.interface, args.channel),
            bitrate=args.bitrate,
            can_filters=filters,
        )
        client = BootloaderClient(bus, capture)
        client.settle()
        client.enter_bootloader(args.boot_timeout)
        client.erase_staging(args.erase_timeout)
        client.begin_download(len(image.payload), args.control_timeout)
        client.download(image.payload, args.segment_timeout, args.final_timeout)
        client.verify_pending_marker(args.control_timeout)
        client.install_and_reset()
        client.wait_for_application(args.post_install_timeout)
        print(f"CAN log                  {log_path}")
        print("firmware update completed and the application is responding")
        return 0
    except (AmbiguousTransferError, KeyboardInterrupt) as error:
        if client is not None:
            client.cancel_transfer()
        detail = (
            "interrupted by operator"
            if isinstance(error, KeyboardInterrupt)
            else str(error)
        )
        print(f"update stopped: {detail}", file=sys.stderr)
        print(
            "The client sent the transfer-cancel command. Rerun the complete command; "
            "do not attempt to resume at the failed segment.",
            file=sys.stderr,
        )
        print(f"CAN log: {log_path}", file=sys.stderr)
        return 4
    except (UpdateError, can.CanError) as error:
        if client is not None and client.transfer_active:
            client.cancel_transfer()
        print(f"update failed: {error}", file=sys.stderr)
        print(f"CAN log: {log_path}", file=sys.stderr)
        return 3
    finally:
        if bus is not None:
            bus.shutdown()
        capture.close()


def main() -> int:
    try:
        return run(build_parser().parse_args())
    except FirmwareValidationError as error:
        print(f"invalid firmware image: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
