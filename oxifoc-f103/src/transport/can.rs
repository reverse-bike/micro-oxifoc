//! Poll-free bxCAN transport for the required stock-bike interface.

#[cfg(feature = "calibration-image")]
use core::cell::UnsafeCell;
use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::{AtomicBool, Ordering};
#[cfg(feature = "firmware")]
use core::sync::atomic::{AtomicU8, AtomicU32};

use crate::protocol::{self, Frame};
use crate::{config, hardware::peripherals};
#[cfg(feature = "firmware")]
use crate::{control::foc, safety, sensors};
use stm32f1::stm32f103::interrupt;

const RCC_APB1ENR: *mut u32 = 0x4002_101c as *mut u32;
const RCC_APB1RSTR: *mut u32 = 0x4002_1010 as *mut u32;
const RCC_APB2ENR: *mut u32 = 0x4002_1018 as *mut u32;
const AFIO_MAPR: *mut u32 = 0x4001_0004 as *mut u32;
const GPIOA_CRH: *mut u32 = 0x4001_0804 as *mut u32;
const GPIOA_BRR: *mut u32 = 0x4001_0814 as *mut u32;

const CAN_BASE: usize = 0x4000_6400;
const CAN_MCR: *mut u32 = CAN_BASE as *mut u32;
const CAN_MSR: *const u32 = (CAN_BASE + 0x04) as *const u32;
const CAN_TSR: *const u32 = (CAN_BASE + 0x08) as *const u32;
const CAN_RF0R: *mut u32 = (CAN_BASE + 0x0c) as *mut u32;
const CAN_IER: *mut u32 = (CAN_BASE + 0x14) as *mut u32;
const CAN_BTR: *mut u32 = (CAN_BASE + 0x1c) as *mut u32;
const CAN_RX0: usize = CAN_BASE + 0x1b0;
const CAN_FMR: *mut u32 = (CAN_BASE + 0x200) as *mut u32;
const CAN_FM1R: *mut u32 = (CAN_BASE + 0x204) as *mut u32;
const CAN_FS1R: *mut u32 = (CAN_BASE + 0x20c) as *mut u32;
const CAN_FFA1R: *mut u32 = (CAN_BASE + 0x214) as *mut u32;
const CAN_FA1R: *mut u32 = (CAN_BASE + 0x21c) as *mut u32;
const CAN_F0R1: *mut u32 = (CAN_BASE + 0x240) as *mut u32;
const CAN_F0R2: *mut u32 = (CAN_BASE + 0x244) as *mut u32;
#[cfg(feature = "calibration-image")]
const CAN_F1R1: *mut u32 = (CAN_BASE + 0x248) as *mut u32;
#[cfg(feature = "calibration-image")]
const CAN_F1R2: *mut u32 = (CAN_BASE + 0x24c) as *mut u32;

const CAN_INIT_TIMEOUT: u32 = 1_000_000;
const CAN_BTR_250K: u32 = (0b10 << 20) | (0b111 << 16) | 11;
const CAN_RESET_REQUEST: u32 = 1;
const CAN_RESET_ACK: u32 = 1;
const CAN_FIFO_PENDING_INTERRUPT: u32 = 1 << 1;
const CAN_FIFO_RELEASE: u32 = 1 << 5;
const CAN_FIFO_OVERRUN: u32 = 1 << 4;
#[cfg(feature = "firmware")]
const CAN_STANDARD_DATA_FILTER_MASK: u32 = (1 << 2) | (1 << 1);
#[cfg(feature = "calibration-image")]
const CALIBRATION_FILTER_BANKS: u32 = 0b11;

#[cfg(feature = "firmware")]
static RESET_REQUESTED: AtomicBool = AtomicBool::new(false);
#[cfg(feature = "firmware")]
static HEADLIGHT_COMMAND: AtomicBool = AtomicBool::new(false);
static TRANSMIT_LOCKED: AtomicBool = AtomicBool::new(false);
#[cfg(feature = "firmware")]
static TELEMETRY_SLOT: AtomicU8 = AtomicU8::new(0);
#[cfg(feature = "firmware")]
static STOCK_FAULT_PAGE: AtomicU8 = AtomicU8::new(0);
#[cfg(feature = "firmware")]
static PROJECT_TELEMETRY_PAGE: AtomicU8 = AtomicU8::new(0);
#[cfg(feature = "firmware")]
static NEXT_TELEMETRY_MS: AtomicU32 = AtomicU32::new(0);

#[cfg(feature = "calibration-image")]
struct ReceivedFrameCell(UnsafeCell<Frame>);

#[cfg(feature = "calibration-image")]
// SAFETY: the CAN receive interrupt is the only writer. Foreground copies the
// frame with interrupts masked after observing RX_PENDING.
unsafe impl Sync for ReceivedFrameCell {}

#[cfg(feature = "calibration-image")]
static RECEIVED_FRAME: ReceivedFrameCell =
    ReceivedFrameCell(UnsafeCell::new(Frame::new(0, 0, [0; 8])));
#[cfg(feature = "calibration-image")]
static RX_PENDING: AtomicBool = AtomicBool::new(false);
#[cfg(feature = "calibration-image")]
static RX_FIFO_OVERRUN_SEEN: AtomicBool = AtomicBool::new(false);
#[cfg(feature = "calibration-image")]
static COMMAND_QUEUE_DROP_SEEN: AtomicBool = AtomicBool::new(false);

pub fn initialize() -> bool {
    // SAFETY: one-time peripheral setup before CAN IRQ is unmasked.
    unsafe {
        write_volatile(RCC_APB2ENR, read_volatile(RCC_APB2ENR) | 0b1101);
        write_volatile(RCC_APB1ENR, read_volatile(RCC_APB1ENR) | (1 << 25));
        write_volatile(RCC_APB1RSTR, read_volatile(RCC_APB1RSTR) | (1 << 25));
        write_volatile(RCC_APB1RSTR, read_volatile(RCC_APB1RSTR) & !(1 << 25));

        let crh = read_volatile(GPIOA_CRH) & !((0xf << 12) | (0xf << 16) | (0xf << 20));
        write_volatile(GPIOA_BRR, 1 << 13);
        write_volatile(GPIOA_CRH, crh | (0x4 << 12) | (0xb << 16) | (0x3 << 20));
        write_volatile(
            AFIO_MAPR,
            (read_volatile(AFIO_MAPR) & !(0b111 << 24)) | (0b100 << 24),
        );

        write_volatile(CAN_MCR, CAN_RESET_REQUEST);
        if !wait_msr(CAN_RESET_ACK, true) {
            return false;
        }
        write_volatile(CAN_MCR, CAN_RESET_REQUEST | (1 << 6) | (1 << 2));
        write_volatile(CAN_BTR, CAN_BTR_250K);

        write_volatile(CAN_FMR, read_volatile(CAN_FMR) | 1);
        #[cfg(feature = "firmware")]
        {
            write_volatile(CAN_FA1R, read_volatile(CAN_FA1R) & !1);
            write_volatile(CAN_FM1R, read_volatile(CAN_FM1R) & !1);
            write_volatile(CAN_FS1R, read_volatile(CAN_FS1R) | 1);
            write_volatile(CAN_FFA1R, read_volatile(CAN_FFA1R) & !1);
            write_volatile(CAN_F0R1, 0);
            write_volatile(CAN_F0R2, CAN_STANDARD_DATA_FILTER_MASK);
            write_volatile(CAN_FA1R, read_volatile(CAN_FA1R) | 1);
        }
        #[cfg(feature = "calibration-image")]
        {
            // Two 16-bit identifier-list banks admit only calibration,
            // updater, and identity traffic. Ignoring the rest of the bike
            // bus in hardware protects the three-frame FIFO during the
            // nearly full-budget Hall and flux control passes.
            write_volatile(
                CAN_FA1R,
                read_volatile(CAN_FA1R) & !CALIBRATION_FILTER_BANKS,
            );
            write_volatile(CAN_FM1R, read_volatile(CAN_FM1R) | CALIBRATION_FILTER_BANKS);
            write_volatile(
                CAN_FS1R,
                read_volatile(CAN_FS1R) & !CALIBRATION_FILTER_BANKS,
            );
            write_volatile(
                CAN_FFA1R,
                read_volatile(CAN_FFA1R) & !CALIBRATION_FILTER_BANKS,
            );
            write_volatile(CAN_F0R1, filter16_pair(0x2f2, 0x67f));
            write_volatile(CAN_F0R2, filter16_pair(0x210, 0x211));
            write_volatile(CAN_F1R1, filter16_pair(0x212, 0x212));
            write_volatile(CAN_F1R2, filter16_pair(0x212, 0x212));
            write_volatile(CAN_FA1R, read_volatile(CAN_FA1R) | CALIBRATION_FILTER_BANKS);
        }
        write_volatile(CAN_FMR, read_volatile(CAN_FMR) & !1);

        write_volatile(CAN_IER, CAN_FIFO_PENDING_INTERRUPT);
        write_volatile(CAN_MCR, read_volatile(CAN_MCR) & !CAN_RESET_REQUEST);
        if !wait_msr(CAN_RESET_ACK, false) {
            return false;
        }
        peripherals::configure_can_interrupt_priority();
        cortex_m::peripheral::NVIC::unmask(stm32f1::stm32f103::Interrupt::USB_LP_CAN_RX0);
    }
    true
}

#[cfg(feature = "firmware")]
#[inline(never)]
#[optimize(size)]
pub fn service(
    now_ms: u32,
    vehicle_speed_tenths_kph: u16,
    distance_counter: u32,
    dc_bus_undervoltage: bool,
    effective_current_limit: u16,
    derating_reasons: u8,
    ride_stage: u8,
) {
    let next = NEXT_TELEMETRY_MS.load(Ordering::Relaxed);
    if next != now_ms && next.wrapping_sub(now_ms) < 0x8000_0000 {
        return;
    }
    NEXT_TELEMETRY_MS.store(now_ms.wrapping_add(50), Ordering::Relaxed);
    let slot = TELEMETRY_SLOT.load(Ordering::Relaxed);
    TELEMETRY_SLOT.store(protocol::next_telemetry_slot(slot), Ordering::Relaxed);
    let inputs = sensors::latest();
    let control = foc::snapshot();
    let fault_page = STOCK_FAULT_PAGE.load(Ordering::Relaxed);
    let controller_temperature = inputs.analog_valid.then(|| {
        sensors::environment::controller_temperature_deci_c(inputs.controller_temperature_adc)
    });
    let motor_temperature = inputs
        .analog_valid
        .then(|| sensors::environment::motor_temperature_deci_c(inputs.motor_temperature_adc))
        .flatten();
    let stock = protocol::StockTelemetry {
        vehicle_speed_tenths_kph,
        distance_counter,
        digital_input_flags: (u8::from(HEADLIGHT_COMMAND.load(Ordering::Relaxed)) << 1)
            | (u8::from(inputs.brake_active) << 2),
        controller_temperature_deci_c: controller_temperature,
        motor_temperature_deci_c: motor_temperature,
        fault_page,
        hardware_break: control.fault_flags & peripherals::FAULT_HARDWARE_BREAK != 0,
        dc_bus_undervoltage,
        hall_invalid: !control.hall_valid,
    };
    let mut mailboxes_used = 0;
    for frame in protocol::telemetry_slot(slot, stock).into_iter().flatten() {
        mailboxes_used += u8::from(transmit(frame));
    }
    if slot == 1 {
        STOCK_FAULT_PAGE.store(fault_page.wrapping_add(1) & 3, Ordering::Relaxed);
    }
    if mailboxes_used < 3 {
        let page = PROJECT_TELEMETRY_PAGE.load(Ordering::Relaxed);
        let frame = match page {
            0 | 1 => protocol::passive_input_telemetry(
                page,
                protocol::PassiveInputTelemetry {
                    analog_valid: inputs.analog_valid,
                    brake_active: inputs.brake_active,
                    throttle_valid: inputs.throttle.is_valid(),
                    throttle_at_rest: inputs.throttle.is_at_rest(),
                    throttle_raw: inputs.throttle.raw(),
                    throttle_demand: inputs.throttle.normalized_counts(),
                    throttle_current_limit_counts: config::RIDE_PHASE_CURRENT_LIMIT_COUNTS,
                    live_hall_state: peripherals::live_hall_state(),
                    bus_voltage_mv: if inputs.analog_valid {
                        sensors::bus_voltage_mv(inputs.bus_voltage_adc).min(u32::from(u16::MAX))
                            as u16
                    } else {
                        0
                    },
                    effective_current_limit,
                    derating_reasons,
                    controller_temperature_deci_c: controller_temperature,
                    motor_temperature_deci_c: motor_temperature,
                },
            ),
            2 => protocol::control_timing_telemetry(protocol::ControlTimingTelemetry {
                current_trips: control.phase_current_trips,
                maximum_cycles: control.control_max_cycles,
                warning_count: control.control_budget_warnings,
            }),
            3 => protocol::control_live_telemetry(protocol::ControlLiveTelemetry {
                hall_valid: control.hall_valid,
                current_valid: control.current_valid,
                output_active: control.output_active,
                voltage_limited: control.voltage_limited,
                ride_stage,
                target_q_counts: control.target_q_counts,
                measured_d_counts: control.measured_d_counts,
                measured_q_counts: control.measured_q_counts,
            }),
            4 => protocol::control_output_telemetry(protocol::ControlOutputTelemetry {
                phase_limit_counts: control.phase_current_limit_counts,
                applied_d_ticks: control.applied_d_ticks,
                applied_q_ticks: control.applied_q_ticks,
                pwm_span_ticks: control.pwm_span_ticks,
            }),
            5 => protocol::control_fault_telemetry(protocol::ControlFaultTelemetry {
                fault_flags: control.fault_flags,
                safety_events: control.safety_events.min(u32::from(u16::MAX)) as u16,
                last_safety_loss_reason: control.last_safety_loss_reason,
            }),
            6 => protocol::control_peak_telemetry(protocol::ControlPeakTelemetry {
                maximum_phase_current_abs: control.maximum_phase_current_abs,
                maximum_direct_current_abs: control.maximum_direct_current_abs,
                maximum_quadrature_error_abs: control.maximum_quadrature_error_abs,
                maximum_pwm_span_ticks: control.maximum_pwm_span_ticks,
            }),
            7..=10 => {
                let event = control.maximum_direct_event;
                protocol::control_peak_event_telemetry(
                    page - 7,
                    protocol::ControlPeakEventTelemetry {
                        generation: event.generation,
                        measured_d_counts: event.measured_d_counts,
                        measured_q_counts: event.measured_q_counts,
                        target_q_counts: event.target_q_counts,
                        hall_raw: event.hall_raw,
                        hall_angle_direction: event.hall_angle_direction,
                        edge_age_us: event.edge_age_us,
                        hall_interval_us: event.hall_interval_us,
                        angle_error_q8: event.angle_error_q8,
                        requested_d_ticks: event.requested_d_ticks,
                        requested_q_ticks: event.requested_q_ticks,
                        feedforward_d_ticks: event.feedforward_d_ticks,
                        feedforward_q_ticks: event.feedforward_q_ticks,
                        applied_d_ticks: event.applied_d_ticks,
                        applied_q_ticks: event.applied_q_ticks,
                        voltage_limited: event.voltage_limited,
                        angle_rate_limited: event.angle_rate_limited,
                    },
                )
            }
            11 => protocol::observer_status_telemetry(protocol::ObserverStatusTelemetry {
                configured: control.observer_configured,
                ready: control.observer_ready,
                active: control.observer_active,
                blend: control.observer_blend,
                confidence: control.observer_confidence,
                electrical_rpm: control.observer_electrical_rpm,
                hall_error_q16: control.observer_hall_error_q16,
            }),
            12 => protocol::observer_model_telemetry(protocol::ObserverModelTelemetry {
                flux_centi_mwb: control.observer_flux_centi_mwb,
                bemf_q_mv: control.observer_bemf_q_mv,
                phase_error_q16: control.observer_phase_error_q16,
                validity_progress: control.observer_validity_progress,
            }),
            13 => protocol::control_timing_breakdown_telemetry(
                protocol::ControlTimingBreakdownTelemetry {
                    maximum_pre_driver_cycles: control.maximum_pre_driver_cycles,
                    maximum_driver_step_cycles: control.maximum_driver_step_cycles,
                    // Valid Hall intervals are at least 100 us, so quarter-eRPM
                    // resolution covers the complete accepted speed range.
                    hall_electrical_rpm_div4: (control.electrical_rpm >> 2) as i16,
                    observer_acquisition_flags: protocol::observer_acquisition_flags(
                        control.observer_phase_error_q16,
                    ),
                },
            ),
            14 | 15 => {
                let reset = safety::boot_diagnostics_snapshot();
                if page == 14 {
                    protocol::reset_summary_telemetry(protocol::ResetSummaryTelemetry {
                        reset_flags: reset.reset_flags,
                        retained_context_valid: reset.retained_context_valid,
                        fatal_reason: reset.fatal_reason,
                        checkpoint: reset.checkpoint,
                        last_control_cycles: reset.last_control_cycles.min(u32::from(u16::MAX))
                            as u16,
                        maximum_control_cycles: reset
                            .maximum_control_cycles
                            .min(u32::from(u16::MAX))
                            as u16,
                    })
                } else {
                    protocol::crash_context_telemetry(protocol::CrashContextTelemetry {
                        detail: reset.detail,
                        control_cycle: reset.control_cycle,
                        program_counter: reset.program_counter,
                        link_register: reset.link_register,
                    })
                }
            }
            16 | 17 => {
                let failure = safety::latest_pwm_failure_snapshot();
                protocol::pwm_failure_telemetry(page + 8, failure.words())
            }
            _ => protocol::firmware_version_telemetry(),
        };
        if transmit(frame) {
            PROJECT_TELEMETRY_PAGE.store(
                protocol::next_project_telemetry_page(page),
                Ordering::Relaxed,
            );
        }
    }
}

#[cfg(feature = "firmware")]
pub fn take_reset_request() -> bool {
    RESET_REQUESTED.swap(false, Ordering::AcqRel)
}

#[cfg(feature = "calibration-image")]
pub fn take_received_frame() -> Option<Frame> {
    cortex_m::interrupt::free(|_| {
        if !RX_PENDING.swap(false, Ordering::AcqRel) {
            return None;
        }
        // SAFETY: the CAN interrupt cannot preempt this critical section, and
        // RX_PENDING prevents it from replacing an unread frame.
        Some(unsafe { *RECEIVED_FRAME.0.get() })
    })
}

#[cfg(feature = "calibration-image")]
pub fn rx_fifo_overrun_seen() -> bool {
    RX_FIFO_OVERRUN_SEEN.load(Ordering::Acquire)
}

#[cfg(feature = "calibration-image")]
pub fn command_queue_drop_seen() -> bool {
    COMMAND_QUEUE_DROP_SEEN.load(Ordering::Acquire)
}

#[cfg(feature = "calibration-image")]
fn retain_received_frame(frame: Frame) {
    let stop = frame.len >= 4
        && u32::from_le_bytes([frame.data[0], frame.data[1], frame.data[2], frame.data[3]])
            == u32::from_le_bytes(*b"STOP");
    if RX_PENDING.load(Ordering::Acquire) && !stop {
        COMMAND_QUEUE_DROP_SEEN.store(true, Ordering::Release);
        return;
    }
    // SAFETY: this function runs only in the CAN receive interrupt. The
    // release store publishes the complete copy to foreground.
    unsafe { *RECEIVED_FRAME.0.get() = frame };
    RX_PENDING.store(true, Ordering::Release);
}

pub fn transmit(frame: Frame) -> bool {
    if TRANSMIT_LOCKED
        .compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
        .is_err()
    {
        return false;
    }
    // SAFETY: mailbox ownership is claimed by setting TXRQ only after all
    // fields are written. The nonblocking software lock prevents the RX
    // interrupt and foreground telemetry from selecting the same mailbox.
    let transmitted = unsafe {
        let empty = read_volatile(CAN_TSR) >> 26;
        let mailbox = if empty & 1 != 0 {
            Some(0)
        } else if empty & 2 != 0 {
            Some(1)
        } else if empty & 4 != 0 {
            Some(2)
        } else {
            None
        };
        if let Some(mailbox) = mailbox {
            let base = CAN_BASE + 0x180 + mailbox * 0x10;
            write_volatile(base as *mut u32, u32::from(frame.id) << 21);
            write_volatile((base + 4) as *mut u32, u32::from(frame.len));
            write_volatile(
                (base + 8) as *mut u32,
                u32::from_le_bytes([frame.data[0], frame.data[1], frame.data[2], frame.data[3]]),
            );
            write_volatile(
                (base + 12) as *mut u32,
                u32::from_le_bytes([frame.data[4], frame.data[5], frame.data[6], frame.data[7]]),
            );
            write_volatile(base as *mut u32, (u32::from(frame.id) << 21) | 1);
            true
        } else {
            false
        }
    };
    TRANSMIT_LOCKED.store(false, Ordering::Release);
    transmitted
}

#[interrupt]
fn USB_LP_CAN_RX0() {
    // SAFETY: FIFO0 is exclusively drained by this interrupt.
    unsafe {
        let fifo = read_volatile(CAN_RF0R);
        if fifo & CAN_FIFO_OVERRUN != 0 {
            #[cfg(feature = "calibration-image")]
            RX_FIFO_OVERRUN_SEEN.store(true, Ordering::Release);
            write_volatile(CAN_RF0R, CAN_FIFO_OVERRUN);
        }
        while read_volatile(CAN_RF0R) & 3 != 0 {
            let identifier = read_volatile(CAN_RX0 as *const u32);
            let len = (read_volatile((CAN_RX0 + 4) as *const u32) & 0xf).min(8) as u8;
            let low = read_volatile((CAN_RX0 + 8) as *const u32).to_le_bytes();
            let high = read_volatile((CAN_RX0 + 12) as *const u32).to_le_bytes();
            if identifier & 6 == 0 {
                let frame = Frame::new(
                    (identifier >> 21) as u16,
                    len,
                    [
                        low[0], low[1], low[2], low[3], high[0], high[1], high[2], high[3],
                    ],
                );
                if frame.len == 0 {
                    if let Some(response) = protocol::identity_response(frame.id) {
                        let _ = transmit(response);
                    }
                } else if protocol::is_updater_reset(frame) {
                    #[cfg(feature = "calibration-image")]
                    {
                        // The calibration image is experimental recovery code:
                        // an exact updater request must work from every routine
                        // state. Remove all gate-drive paths before resetting,
                        // including when foreground or control state is stuck.
                        peripherals::emergency_shutdown();
                        cortex_m::peripheral::SCB::sys_reset();
                    }
                    #[cfg(feature = "firmware")]
                    RESET_REQUESTED.store(true, Ordering::Release);
                } else {
                    #[cfg(feature = "firmware")]
                    if let Some(enabled) = protocol::headlight_command(frame) {
                        HEADLIGHT_COMMAND.store(enabled, Ordering::Relaxed);
                    }
                    #[cfg(feature = "calibration-image")]
                    if frame.id == config::CAN_STOP_CALIBRATION_ID {
                        retain_received_frame(frame);
                    }
                }
            }
            write_volatile(CAN_RF0R, CAN_FIFO_RELEASE);
        }
    }
}

#[cfg(feature = "calibration-image")]
const fn filter16_pair(low_id: u16, high_id: u16) -> u32 {
    ((low_id as u32) << 5) | ((high_id as u32) << 21)
}

unsafe fn wait_msr(mask: u32, set: bool) -> bool {
    for _ in 0..CAN_INIT_TIMEOUT {
        let present = unsafe { read_volatile(CAN_MSR) } & mask != 0;
        if present == set {
            return true;
        }
    }
    false
}
