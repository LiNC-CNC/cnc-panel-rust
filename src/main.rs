//! SZGH-1080 Panel HAL Component
//! 
//! Non-realtime LinuxCNC HAL component for SZGH-1080 CNC control panel.
//! This component exposes all panel buttons and LEDs to the LinuxCNC HAL,
//! providing complete integration between the physical panel and LinuxCNC.
//!
//! Ported from szgh_panel_hal.py

use anyhow::{Context, Result};
use clap::Parser;
use log::{debug, error, info, warn};
use serialport::{DataBits, Parity, SerialPort, StopBits};
use std::{
    io::{Read, Write},
    time::Duration,
};

#[cfg(feature = "linuxcnc-hal")]
use linuxcnc_hal::{
    hal_pin::{InputPin, OutputPin},
    prelude::*,
    HalComponent, RegisterResources, Resources,
};

// ============================================================================
// Button and LED Constants
// ============================================================================

/// Button bit masks
pub mod buttons {
    pub const PROGRAM_PROTECT: u64 = 1 << 0;
    pub const ESTOP: u64 = 1 << 7;
    pub const CYCLE_START: u64 = 1 << 14;
    pub const CYCLE_STOP: u64 = 1 << 15;
    pub const RAPID: u64 = 1 << 63;
    pub const MLK: u64 = 1 << 55;
    pub const FLUSH: u64 = 1 << 47;
    pub const SPD_SP: u64 = 1 << 39;
    pub const K2: u64 = 1 << 34;
    pub const JOG_XN: u64 = 1 << 62;
    pub const BDT: u64 = 1 << 54;
    pub const SPD_CTS: u64 = 1 << 46;
    pub const SPD_CW: u64 = 1 << 38;
    pub const ARM_MAN: u64 = 1 << 30;
    pub const JOG_YP: u64 = 1 << 61;
    pub const SBK: u64 = 1 << 53;
    pub const COOLANT: u64 = 1 << 45;
    pub const MAG_CCW: u64 = 1 << 37;
    pub const POT_DN: u64 = 1 << 29;
    pub const JOG_ZP: u64 = 1 << 60;
    pub const MPG_SIM: u64 = 1 << 52;
    pub const PT_LA: u64 = 1 << 44;
    pub const MAG_CW: u64 = 1 << 36;
    pub const CHUCK: u64 = 1 << 29;
    pub const JOG_AN: u64 = 1 << 59;
    pub const JOG_AP: u64 = 1 << 51;
    pub const AIR_BLOW: u64 = 1 << 43;
    pub const SPD_ORI: u64 = 1 << 35;
    pub const CHIP_CCW: u64 = 1 << 27;
    pub const JOG_BP: u64 = 1 << 58;
    pub const JOG_ZN: u64 = 1 << 50;
    pub const WORKLAMP: u64 = 1 << 42;
    pub const K1: u64 = 1 << 34;
    pub const CHIP_SP: u64 = 1 << 26;
    pub const SAFE_DOOR: u64 = 1 << 57;
    pub const JOG_YN: u64 = 1 << 49;
    pub const DRN: u64 = 1 << 41;
    pub const CORRIDOR: u64 = 1 << 33;
    pub const CHIP_CW: u64 = 1 << 25;
    pub const K4: u64 = 1 << 17;
    pub const JOG_BN: u64 = 1 << 56;
    pub const JOG_XP: u64 = 1 << 48;
    pub const OPT: u64 = 1 << 40;
    pub const LUB: u64 = 1 << 31;
    pub const SPD_CCW: u64 = 1 << 24;
    pub const K3: u64 = 1 << 16;
}

/// LED bit masks
pub mod leds {
    pub const RAPID: u64 = 1 << 0;
    pub const MLK: u64 = 1 << 1;
    pub const FLUSH: u64 = 1 << 2;
    pub const SPD_SP: u64 = 1 << 3;
    pub const K2: u64 = 1 << 4;
    pub const C: u64 = 1 << 5;
    pub const CYCLE_OFF: u64 = 1 << 6;
    pub const JOG_XN: u64 = 1 << 8;
    pub const BDT: u64 = 1 << 9;
    pub const SPD_CTS: u64 = 1 << 10;
    pub const SPD_CW: u64 = 1 << 11;
    pub const ARM_MAN: u64 = 1 << 12;
    pub const B: u64 = 1 << 13;
    pub const CYCLE_ON: u64 = 1 << 14;
    pub const JOG_YP: u64 = 1 << 16;
    pub const SBK: u64 = 1 << 17;
    pub const COOLANT: u64 = 1 << 18;
    pub const MAG_CCW: u64 = 1 << 19;
    pub const POT_DN: u64 = 1 << 20;
    pub const A: u64 = 1 << 21;
    pub const READY: u64 = 1 << 22;
    pub const JOG_ZP: u64 = 1 << 24;
    pub const MPG_SIM: u64 = 1 << 25;
    pub const PT_LA: u64 = 1 << 26;
    pub const MAG_CW: u64 = 1 << 27;
    pub const CHUCK: u64 = 1 << 28;
    pub const Z: u64 = 1 << 29;
    pub const RUN: u64 = 1 << 30;
    pub const JOG_AN: u64 = 1 << 32;
    pub const JOG_AP: u64 = 1 << 33;
    pub const AIR_BLOW: u64 = 1 << 34;
    pub const SPD_ORI: u64 = 1 << 35;
    pub const CHIP_CCW: u64 = 1 << 36;
    pub const Y: u64 = 1 << 37;
    pub const AIRBLOW_ALARM: u64 = 1 << 38;
    pub const JOG_BP: u64 = 1 << 40;
    pub const JOG_ZN: u64 = 1 << 41;
    pub const WORKLAMP: u64 = 1 << 42;
    pub const K1: u64 = 1 << 43;
    pub const CHIP_SP: u64 = 1 << 44;
    pub const X: u64 = 1 << 45;
    pub const OIL_ALARM: u64 = 1 << 46;
    pub const SAFE_DOOR: u64 = 1 << 48;
    pub const JOG_YN: u64 = 1 << 49;
    pub const DRN: u64 = 1 << 50;
    pub const CORRIDOR: u64 = 1 << 51;
    pub const CHIP_CW: u64 = 1 << 52;
    pub const K4: u64 = 1 << 53;
    pub const GEAR_ALARM: u64 = 1 << 54;
    pub const JOG_BN: u64 = 1 << 56;
    pub const JOG_XP: u64 = 1 << 57;
    pub const OPT: u64 = 1 << 58;
    pub const LUB: u64 = 1 << 59;
    pub const SPD_CCW: u64 = 1 << 60;
    pub const K3: u64 = 1 << 61;
    pub const MC_ALARM: u64 = 1 << 62;
}

// ============================================================================
// Helper Functions
// ============================================================================

/// Calculate MODBUS CRC-16 for the given data bytes
fn modbus_crc16(data: &[u8]) -> [u8; 2] {
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        crc ^= byte as u16;
        for _ in 0..8 {
            if crc & 0x0001 != 0 {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    [(crc & 0xFF) as u8, ((crc >> 8) & 0xFF) as u8]
}

/// Convert LED flags to 10-byte array
fn led_state_to_bytes(leds: u64) -> [u8; 10] {
    let mut led_state: u64 = u64::MAX;
    led_state &= !leds;
    led_state <<= 16;

    let mut output = [0u8; 10];
    for i in 0..10 {
        let shift = (9 - i) * 8;
        // When shift >= 64, wrapping_shr returns 0, which gives us 0xFF & 0xFF = 0xFF
        // (the behavior we want for the padded bytes)
        output[i] = (led_state.wrapping_shr(shift as u32) & 0xFF) as u8;
    }
    output
}

/// Extract button state from response bytes 2-9
fn extract_button_state(response: &[u8]) -> u64 {
    if response.len() < 10 {
        return 0;
    }

    let mut button_state: u64 = 0;
    for i in 2..10 {
        button_state |= (response[i] as u64) << ((9 - i) * 8);
    }
    button_state
}

// ============================================================================
// CLI Arguments
// ============================================================================

#[derive(Parser, Debug)]
#[command(name = "szgh-panel-hal")]
#[command(about = "SZGH-1080 Panel HAL Component for LinuxCNC")]
#[command(version)]
struct Args {
    #[arg(short, long, default_value = "/dev/ttyUSB0")]
    port: String,
    #[arg(short, long, default_value_t = 115200)]
    baud: u32,
    #[arg(short, long, default_value_t = 0.05)]
    cycle_time: f64,
    #[arg(short, long)]
    verbose: bool,
    #[arg(short, long)]
    debug: bool,
}

// ============================================================================
// Panel Communication
// ============================================================================

struct PanelComm {
    port: String,
    baud: u32,
}

impl PanelComm {
    fn new(port: String, baud: u32) -> Self {
        Self { port, baud }
    }

    fn open_port(&self) -> Result<Box<dyn SerialPort>> {
        let port = serialport::new(&self.port, self.baud)
            .data_bits(DataBits::Eight)
            .parity(Parity::None)
            .stop_bits(StopBits::One)
            .timeout(Duration::from_secs(1))
            .open()
            .with_context(|| format!("Failed to open serial port {}", self.port))?;
        
        info!("Connected to serial port {}@{} baud", self.port, self.baud);
        Ok(port)
    }

    fn communicate(&self, led_bytes: &[u8; 10]) -> Result<Option<[u8; 12]>> {
        let mut port = self.open_port()?;
        
        // First write: eb 01 80 8e
        let first_command = [0xEB, 0x01, 0x80, 0x8E];
        port.write_all(&first_command)?;
        port.flush()?;

        // Read 12 bytes
        let mut response = [0u8; 12];
        let bytes_read = port.read(&mut response)?;
        
        if bytes_read != 12 {
            warn!("Incomplete response: got {} bytes, expected 12", bytes_read);
            return Ok(None);
        }

        // Check CRC
        let response_data = &response[..10];
        let response_crc = &response[10..12];
        let calculated_crc = modbus_crc16(response_data);

        if response_crc != calculated_crc && response_crc != [calculated_crc[1], calculated_crc[0]] {
            warn!("CRC check failed");
            return Ok(None);
        }

        debug!("CRC check passed");

        // Second write: eb 02 + 10 bytes + CRC16
        let second_command_prefix = [0xEB, 0x02];
        let mut crc_data = Vec::with_capacity(12);
        crc_data.extend_from_slice(&second_command_prefix);
        crc_data.extend_from_slice(led_bytes);
        let crc = modbus_crc16(&crc_data);

        let mut second_command = Vec::with_capacity(14);
        second_command.extend_from_slice(&crc_data);
        second_command.push(crc[1]);
        second_command.push(crc[0]);

        let final_command = [0xEB, 0x02, 0x81, 0xCE];

        port.write_all(&second_command)?;
        port.write_all(&final_command)?;
        port.flush()?;

        Ok(Some(response))
    }
}

// ============================================================================
// HAL Component
// ============================================================================

#[cfg(feature = "linuxcnc-hal")]
struct SZGHPanelHal {
    /// Button output pins (panel → LinuxCNC)
    program_protect: OutputPin<bool>,
    estop: OutputPin<bool>,
    cycle_start: OutputPin<bool>,
    cycle_stop: OutputPin<bool>,
    rapid: OutputPin<bool>,
    mlk: OutputPin<bool>,
    flush: OutputPin<bool>,
    spd_sp: OutputPin<bool>,
    k2: OutputPin<bool>,
    jog_xn: OutputPin<bool>,
    bdt: OutputPin<bool>,
    spd_cts: OutputPin<bool>,
    spd_cw: OutputPin<bool>,
    arm_man: OutputPin<bool>,
    jog_yp: OutputPin<bool>,
    sbk: OutputPin<bool>,
    coolant: OutputPin<bool>,
    mag_ccw: OutputPin<bool>,
    pot_dn: OutputPin<bool>,
    jog_zp: OutputPin<bool>,
    mpg_sim: OutputPin<bool>,
    pt_la: OutputPin<bool>,
    mag_cw: OutputPin<bool>,
    chuck: OutputPin<bool>,
    jog_an: OutputPin<bool>,
    jog_ap: OutputPin<bool>,
    air_blow: OutputPin<bool>,
    spd_ori: OutputPin<bool>,
    chip_ccw: OutputPin<bool>,
    jog_bp: OutputPin<bool>,
    jog_zn: OutputPin<bool>,
    worklamp: OutputPin<bool>,
    k1: OutputPin<bool>,
    chip_sp: OutputPin<bool>,
    safe_door: OutputPin<bool>,
    jog_yn: OutputPin<bool>,
    drn: OutputPin<bool>,
    corridor: OutputPin<bool>,
    chip_cw: OutputPin<bool>,
    k4: OutputPin<bool>,
    jog_bn: OutputPin<bool>,
    jog_xp: OutputPin<bool>,
    opt: OutputPin<bool>,
    lube: OutputPin<bool>,
    spd_ccw: OutputPin<bool>,
    k3: OutputPin<bool>,

    /// LED input pins (LinuxCNC → panel)
    led_rapid: InputPin<bool>,
    led_mlk: InputPin<bool>,
    led_flush: InputPin<bool>,
    led_spd_sp: InputPin<bool>,
    led_k2: InputPin<bool>,
    led_c: InputPin<bool>,
    led_cycle_off: InputPin<bool>,
    led_jog_xn: InputPin<bool>,
    led_bdt: InputPin<bool>,
    led_spd_cts: InputPin<bool>,
    led_spd_cw: InputPin<bool>,
    led_arm_man: InputPin<bool>,
    led_b: InputPin<bool>,
    led_cycle_on: InputPin<bool>,
    led_jog_yp: InputPin<bool>,
    led_sbk: InputPin<bool>,
    led_coolant: InputPin<bool>,
    led_mag_ccw: InputPin<bool>,
    led_pot_dn: InputPin<bool>,
    led_a: InputPin<bool>,
    led_ready: InputPin<bool>,
    led_jog_zp: InputPin<bool>,
    led_mpg_sim: InputPin<bool>,
    led_pt_la: InputPin<bool>,
    led_mag_cw: InputPin<bool>,
    led_chuck: InputPin<bool>,
    led_z: InputPin<bool>,
    led_run: InputPin<bool>,
    led_jog_an: InputPin<bool>,
    led_jog_ap: InputPin<bool>,
    led_air_blow: InputPin<bool>,
    led_spd_ori: InputPin<bool>,
    led_chip_ccw: InputPin<bool>,
    led_y: InputPin<bool>,
    led_airblow_alarm: InputPin<bool>,
    led_jog_bp: InputPin<bool>,
    led_jog_zn: InputPin<bool>,
    led_worklamp: InputPin<bool>,
    led_k1: InputPin<bool>,
    led_chip_sp: InputPin<bool>,
    led_x: InputPin<bool>,
    led_oil_alarm: InputPin<bool>,
    led_safe_door: InputPin<bool>,
    led_jog_yn: InputPin<bool>,
    led_drn: InputPin<bool>,
    led_corridor: InputPin<bool>,
    led_chip_cw: InputPin<bool>,
    led_k4: InputPin<bool>,
    led_gear_alarm: InputPin<bool>,
    led_jog_bn: InputPin<bool>,
    led_jog_xp: InputPin<bool>,
    led_opt: InputPin<bool>,
    led_lub: InputPin<bool>,
    led_spd_ccw: InputPin<bool>,
    led_k3: InputPin<bool>,
    led_mc_alarm: InputPin<bool>,

    /// 7-segment display pins
    sseg_magtool: InputPin<u32>,
    sseg_spdtool: InputPin<i32>,

    /// Mode switch output pins
    mode_edit: OutputPin<bool>,
    mode_auto: OutputPin<bool>,
    mode_mdi: OutputPin<bool>,
    mode_jog: OutputPin<bool>,
    mode_mpg: OutputPin<bool>,
    mode_inc: OutputPin<bool>,
    mode_home: OutputPin<bool>,
    mode_proghome: OutputPin<bool>,

    /// Home output pins
    home_all: OutputPin<bool>,
    home_x: OutputPin<bool>,
    home_y: OutputPin<bool>,
    home_z: OutputPin<bool>,

    /// Proghome output pins
    proghome_all: OutputPin<bool>,
    proghome_x: OutputPin<bool>,
    proghome_y: OutputPin<bool>,
    proghome_z: OutputPin<bool>,

    /// Override pins
    feed_override: OutputPin<f64>,
    rapid_override: OutputPin<f64>,
    spindle_override: OutputPin<f64>,

    /// Status pins
    communication_error: OutputPin<bool>,
    panel_connected: OutputPin<bool>,
}

#[cfg(feature = "linuxcnc-hal")]
impl Resources for SZGHPanelHal {
    type RegisterError = linuxcnc_hal::error::PinRegisterError;

    fn register_resources(comp: &RegisterResources) -> Result<Self, Self::RegisterError> {
        // Button output pins
        let program_protect = comp.register_pin::<OutputPin<bool>>("program_protect")?;
        let estop = comp.register_pin::<OutputPin<bool>>("estop")?;
        let cycle_start = comp.register_pin::<OutputPin<bool>>("cycle_start")?;
        let cycle_stop = comp.register_pin::<OutputPin<bool>>("cycle_stop")?;
        let rapid = comp.register_pin::<OutputPin<bool>>("rapid")?;
        let mlk = comp.register_pin::<OutputPin<bool>>("mlk")?;
        let flush = comp.register_pin::<OutputPin<bool>>("flush")?;
        let spd_sp = comp.register_pin::<OutputPin<bool>>("spd_sp")?;
        let k2 = comp.register_pin::<OutputPin<bool>>("k2")?;
        let jog_xn = comp.register_pin::<OutputPin<bool>>("jog_xn")?;
        let bdt = comp.register_pin::<OutputPin<bool>>("bdt")?;
        let spd_cts = comp.register_pin::<OutputPin<bool>>("spd_cts")?;
        let spd_cw = comp.register_pin::<OutputPin<bool>>("spd_cw")?;
        let arm_man = comp.register_pin::<OutputPin<bool>>("arm_man")?;
        let jog_yp = comp.register_pin::<OutputPin<bool>>("jog_yp")?;
        let sbk = comp.register_pin::<OutputPin<bool>>("sbk")?;
        let coolant = comp.register_pin::<OutputPin<bool>>("coolant")?;
        let mag_ccw = comp.register_pin::<OutputPin<bool>>("mag_ccw")?;
        let pot_dn = comp.register_pin::<OutputPin<bool>>("pot_dn")?;
        let jog_zp = comp.register_pin::<OutputPin<bool>>("jog_zp")?;
        let mpg_sim = comp.register_pin::<OutputPin<bool>>("mpg_sim")?;
        let pt_la = comp.register_pin::<OutputPin<bool>>("pt_la")?;
        let mag_cw = comp.register_pin::<OutputPin<bool>>("mag_cw")?;
        let chuck = comp.register_pin::<OutputPin<bool>>("chuck")?;
        let jog_an = comp.register_pin::<OutputPin<bool>>("jog_an")?;
        let jog_ap = comp.register_pin::<OutputPin<bool>>("jog_ap")?;
        let air_blow = comp.register_pin::<OutputPin<bool>>("air_blow")?;
        let spd_ori = comp.register_pin::<OutputPin<bool>>("spd_ori")?;
        let chip_ccw = comp.register_pin::<OutputPin<bool>>("chip_ccw")?;
        let jog_bp = comp.register_pin::<OutputPin<bool>>("jog_bp")?;
        let jog_zn = comp.register_pin::<OutputPin<bool>>("jog_zn")?;
        let worklamp = comp.register_pin::<OutputPin<bool>>("worklamp")?;
        let k1 = comp.register_pin::<OutputPin<bool>>("k1")?;
        let chip_sp = comp.register_pin::<OutputPin<bool>>("chip_sp")?;
        let safe_door = comp.register_pin::<OutputPin<bool>>("safe_door")?;
        let jog_yn = comp.register_pin::<OutputPin<bool>>("jog_yn")?;
        let drn = comp.register_pin::<OutputPin<bool>>("drn")?;
        let corridor = comp.register_pin::<OutputPin<bool>>("corridor")?;
        let chip_cw = comp.register_pin::<OutputPin<bool>>("chip_cw")?;
        let k4 = comp.register_pin::<OutputPin<bool>>("k4")?;
        let jog_bn = comp.register_pin::<OutputPin<bool>>("jog_bn")?;
        let jog_xp = comp.register_pin::<OutputPin<bool>>("jog_xp")?;
        let opt = comp.register_pin::<OutputPin<bool>>("opt")?;
        let lube = comp.register_pin::<OutputPin<bool>>("lube")?;
        let spd_ccw = comp.register_pin::<OutputPin<bool>>("spd_ccw")?;
        let k3 = comp.register_pin::<OutputPin<bool>>("k3")?;

        // LED input pins
        let led_rapid = comp.register_pin::<InputPin<bool>>("led_rapid")?;
        let led_mlk = comp.register_pin::<InputPin<bool>>("led_mlk")?;
        let led_flush = comp.register_pin::<InputPin<bool>>("led_flush")?;
        let led_spd_sp = comp.register_pin::<InputPin<bool>>("led_spd_sp")?;
        let led_k2 = comp.register_pin::<InputPin<bool>>("led_k2")?;
        let led_c = comp.register_pin::<InputPin<bool>>("led_c")?;
        let led_cycle_off = comp.register_pin::<InputPin<bool>>("led_cycle_off")?;
        let led_jog_xn = comp.register_pin::<InputPin<bool>>("led_jog_xn")?;
        let led_bdt = comp.register_pin::<InputPin<bool>>("led_bdt")?;
        let led_spd_cts = comp.register_pin::<InputPin<bool>>("led_spd_cts")?;
        let led_spd_cw = comp.register_pin::<InputPin<bool>>("led_spd_cw")?;
        let led_arm_man = comp.register_pin::<InputPin<bool>>("led_arm_man")?;
        let led_b = comp.register_pin::<InputPin<bool>>("led_b")?;
        let led_cycle_on = comp.register_pin::<InputPin<bool>>("led_cycle_on")?;
        let led_jog_yp = comp.register_pin::<InputPin<bool>>("led_jog_yp")?;
        let led_sbk = comp.register_pin::<InputPin<bool>>("led_sbk")?;
        let led_coolant = comp.register_pin::<InputPin<bool>>("led_coolant")?;
        let led_mag_ccw = comp.register_pin::<InputPin<bool>>("led_mag_ccw")?;
        let led_pot_dn = comp.register_pin::<InputPin<bool>>("led_pot_dn")?;
        let led_a = comp.register_pin::<InputPin<bool>>("led_a")?;
        let led_ready = comp.register_pin::<InputPin<bool>>("led_ready")?;
        let led_jog_zp = comp.register_pin::<InputPin<bool>>("led_jog_zp")?;
        let led_mpg_sim = comp.register_pin::<InputPin<bool>>("led_mpg_sim")?;
        let led_pt_la = comp.register_pin::<InputPin<bool>>("led_pt_la")?;
        let led_mag_cw = comp.register_pin::<InputPin<bool>>("led_mag_cw")?;
        let led_chuck = comp.register_pin::<InputPin<bool>>("led_chuck")?;
        let led_z = comp.register_pin::<InputPin<bool>>("led_z")?;
        let led_run = comp.register_pin::<InputPin<bool>>("led_run")?;
        let led_jog_an = comp.register_pin::<InputPin<bool>>("led_jog_an")?;
        let led_jog_ap = comp.register_pin::<InputPin<bool>>("led_jog_ap")?;
        let led_air_blow = comp.register_pin::<InputPin<bool>>("led_air_blow")?;
        let led_spd_ori = comp.register_pin::<InputPin<bool>>("led_spd_ori")?;
        let led_chip_ccw = comp.register_pin::<InputPin<bool>>("led_chip_ccw")?;
        let led_y = comp.register_pin::<InputPin<bool>>("led_y")?;
        let led_airblow_alarm = comp.register_pin::<InputPin<bool>>("led_airblow_alarm")?;
        let led_jog_bp = comp.register_pin::<InputPin<bool>>("led_jog_bp")?;
        let led_jog_zn = comp.register_pin::<InputPin<bool>>("led_jog_zn")?;
        let led_worklamp = comp.register_pin::<InputPin<bool>>("led_worklamp")?;
        let led_k1 = comp.register_pin::<InputPin<bool>>("led_k1")?;
        let led_chip_sp = comp.register_pin::<InputPin<bool>>("led_chip_sp")?;
        let led_x = comp.register_pin::<InputPin<bool>>("led_x")?;
        let led_oil_alarm = comp.register_pin::<InputPin<bool>>("led_oil_alarm")?;
        let led_safe_door = comp.register_pin::<InputPin<bool>>("led_safe_door")?;
        let led_jog_yn = comp.register_pin::<InputPin<bool>>("led_jog_yn")?;
        let led_drn = comp.register_pin::<InputPin<bool>>("led_drn")?;
        let led_corridor = comp.register_pin::<InputPin<bool>>("led_corridor")?;
        let led_chip_cw = comp.register_pin::<InputPin<bool>>("led_chip_cw")?;
        let led_k4 = comp.register_pin::<InputPin<bool>>("led_k4")?;
        let led_gear_alarm = comp.register_pin::<InputPin<bool>>("led_gear_alarm")?;
        let led_jog_bn = comp.register_pin::<InputPin<bool>>("led_jog_bn")?;
        let led_jog_xp = comp.register_pin::<InputPin<bool>>("led_jog_xp")?;
        let led_opt = comp.register_pin::<InputPin<bool>>("led_opt")?;
        let led_lub = comp.register_pin::<InputPin<bool>>("led_lub")?;
        let led_spd_ccw = comp.register_pin::<InputPin<bool>>("led_spd_ccw")?;
        let led_k3 = comp.register_pin::<InputPin<bool>>("led_k3")?;
        let led_mc_alarm = comp.register_pin::<InputPin<bool>>("led_mc_alarm")?;

        // 7-segment display pins
        let sseg_magtool = comp.register_pin::<InputPin<u32>>("sseg_magtool")?;
        let sseg_spdtool = comp.register_pin::<InputPin<i32>>("sseg_spdtool")?;

        // Mode switch output pins
        let mode_edit = comp.register_pin::<OutputPin<bool>>("mode_edit")?;
        let mode_auto = comp.register_pin::<OutputPin<bool>>("mode_auto")?;
        let mode_mdi = comp.register_pin::<OutputPin<bool>>("mode_mdi")?;
        let mode_jog = comp.register_pin::<OutputPin<bool>>("mode_jog")?;
        let mode_mpg = comp.register_pin::<OutputPin<bool>>("mode_mpg")?;
        let mode_inc = comp.register_pin::<OutputPin<bool>>("mode_inc")?;
        let mode_home = comp.register_pin::<OutputPin<bool>>("mode_home")?;
        let mode_proghome = comp.register_pin::<OutputPin<bool>>("mode_proghome")?;

        // Home output pins
        let home_all = comp.register_pin::<OutputPin<bool>>("home-all")?;
        let home_x = comp.register_pin::<OutputPin<bool>>("home-x")?;
        let home_y = comp.register_pin::<OutputPin<bool>>("home-y")?;
        let home_z = comp.register_pin::<OutputPin<bool>>("home-z")?;

        // Proghome output pins
        let proghome_all = comp.register_pin::<OutputPin<bool>>("proghome-all")?;
        let proghome_x = comp.register_pin::<OutputPin<bool>>("proghome-x")?;
        let proghome_y = comp.register_pin::<OutputPin<bool>>("proghome-y")?;
        let proghome_z = comp.register_pin::<OutputPin<bool>>("proghome-z")?;

        // Override pins
        let feed_override = comp.register_pin::<OutputPin<f64>>("feed_override")?;
        let rapid_override = comp.register_pin::<OutputPin<f64>>("rapid_override")?;
        let spindle_override = comp.register_pin::<OutputPin<f64>>("spindle_override")?;

        // Status pins
        let communication_error = comp.register_pin::<OutputPin<bool>>("communication_error")?;
        let panel_connected = comp.register_pin::<OutputPin<bool>>("panel_connected")?;

        Ok(Self {
            program_protect,
            estop,
            cycle_start,
            cycle_stop,
            rapid,
            mlk,
            flush,
            spd_sp,
            k2,
            jog_xn,
            bdt,
            spd_cts,
            spd_cw,
            arm_man,
            jog_yp,
            sbk,
            coolant,
            mag_ccw,
            pot_dn,
            jog_zp,
            mpg_sim,
            pt_la,
            mag_cw,
            chuck,
            jog_an,
            jog_ap,
            air_blow,
            spd_ori,
            chip_ccw,
            jog_bp,
            jog_zn,
            worklamp,
            k1,
            chip_sp,
            safe_door,
            jog_yn,
            drn,
            corridor,
            chip_cw,
            k4,
            jog_bn,
            jog_xp,
            opt,
            lube,
            spd_ccw,
            k3,
            led_rapid,
            led_mlk,
            led_flush,
            led_spd_sp,
            led_k2,
            led_c,
            led_cycle_off,
            led_jog_xn,
            led_bdt,
            led_spd_cts,
            led_spd_cw,
            led_arm_man,
            led_b,
            led_cycle_on,
            led_jog_yp,
            led_sbk,
            led_coolant,
            led_mag_ccw,
            led_pot_dn,
            led_a,
            led_ready,
            led_jog_zp,
            led_mpg_sim,
            led_pt_la,
            led_mag_cw,
            led_chuck,
            led_z,
            led_run,
            led_jog_an,
            led_jog_ap,
            led_air_blow,
            led_spd_ori,
            led_chip_ccw,
            led_y,
            led_airblow_alarm,
            led_jog_bp,
            led_jog_zn,
            led_worklamp,
            led_k1,
            led_chip_sp,
            led_x,
            led_oil_alarm,
            led_safe_door,
            led_jog_yn,
            led_drn,
            led_corridor,
            led_chip_cw,
            led_k4,
            led_gear_alarm,
            led_jog_bn,
            led_jog_xp,
            led_opt,
            led_lub,
            led_spd_ccw,
            led_k3,
            led_mc_alarm,
            sseg_magtool,
            sseg_spdtool,
            mode_edit,
            mode_auto,
            mode_mdi,
            mode_jog,
            mode_mpg,
            mode_inc,
            mode_home,
            mode_proghome,
            home_all,
            home_x,
            home_y,
            home_z,
            proghome_all,
            proghome_x,
            proghome_y,
            proghome_z,
            feed_override,
            rapid_override,
            spindle_override,
            communication_error,
            panel_connected,
        })
    }
}

#[cfg(feature = "linuxcnc-hal")]
impl SZGHPanelHal {
    /// Update all button HAL pins from button state
    fn update_button_pins(&self, button_state: u64) {
        // Update mode pins based on mode switch (bits 18-20)
        let current_mode = ((button_state >> 18) & 0b111) as usize;
        
        let mode_pins: [&OutputPin<bool>; 8] = [
            &self.mode_edit,
            &self.mode_auto,
            &self.mode_mdi,
            &self.mode_jog,
            &self.mode_mpg,
            &self.mode_inc,
            &self.mode_home,
            &self.mode_proghome,
        ];

        for (i, pin) in mode_pins.iter().enumerate() {
            let _ = pin.set_value(i == current_mode);
        }

        let mode_home_active = current_mode == 6;
        let mode_proghome_active = current_mode == 7;
        let home_or_proghome_active = mode_home_active || mode_proghome_active;

        // Helper function to set button pin with home mode suppression
        let set_button = |pin: &OutputPin<bool>, mask: u64| {
            let pressed = (!button_state & mask) != 0;
            let _ = pin.set_value(pressed);
        };

        // Update button pins (not affected by home mode)
        set_button(&self.program_protect, buttons::PROGRAM_PROTECT);
        set_button(&self.estop, buttons::ESTOP);
        set_button(&self.cycle_start, buttons::CYCLE_START);
        set_button(&self.cycle_stop, buttons::CYCLE_STOP);
        set_button(&self.mlk, buttons::MLK);
        set_button(&self.flush, buttons::FLUSH);
        set_button(&self.spd_sp, buttons::SPD_SP);
        set_button(&self.k2, buttons::K2);
        set_button(&self.bdt, buttons::BDT);
        set_button(&self.spd_cts, buttons::SPD_CTS);
        set_button(&self.spd_cw, buttons::SPD_CW);
        set_button(&self.arm_man, buttons::ARM_MAN);
        set_button(&self.jog_yp, buttons::JOG_YP);
        set_button(&self.sbk, buttons::SBK);
        set_button(&self.coolant, buttons::COOLANT);
        set_button(&self.mag_ccw, buttons::MAG_CCW);
        set_button(&self.pot_dn, buttons::POT_DN);
        set_button(&self.jog_zp, buttons::JOG_ZP);
        set_button(&self.mpg_sim, buttons::MPG_SIM);
        set_button(&self.pt_la, buttons::PT_LA);
        set_button(&self.mag_cw, buttons::MAG_CW);
        set_button(&self.chuck, buttons::CHUCK);
        set_button(&self.jog_an, buttons::JOG_AN);
        set_button(&self.jog_ap, buttons::JOG_AP);
        set_button(&self.air_blow, buttons::AIR_BLOW);
        set_button(&self.spd_ori, buttons::SPD_ORI);
        set_button(&self.chip_ccw, buttons::CHIP_CCW);
        set_button(&self.jog_bp, buttons::JOG_BP);
        set_button(&self.jog_zn, buttons::JOG_ZN);
        set_button(&self.worklamp, buttons::WORKLAMP);
        set_button(&self.k1, buttons::K1);
        set_button(&self.chip_sp, buttons::CHIP_SP);
        set_button(&self.safe_door, buttons::SAFE_DOOR);
        set_button(&self.jog_yn, buttons::JOG_YN);
        set_button(&self.drn, buttons::DRN);
        set_button(&self.corridor, buttons::CORRIDOR);
        set_button(&self.chip_cw, buttons::CHIP_CW);
        set_button(&self.k4, buttons::K4);
        set_button(&self.opt, buttons::OPT);
        set_button(&self.lube, buttons::LUB);
        set_button(&self.spd_ccw, buttons::SPD_CCW);
        set_button(&self.k3, buttons::K3);

        // Jog buttons suppressed in home/proghome mode
        let set_jog_button = |pin: &OutputPin<bool>, mask: u64| {
            let pressed = if home_or_proghome_active {
                false
            } else {
                (!button_state & mask) != 0
            };
            let _ = pin.set_value(pressed);
        };

        set_jog_button(&self.rapid, buttons::RAPID);
        set_jog_button(&self.jog_xn, buttons::JOG_XN);
        set_jog_button(&self.jog_yp, buttons::JOG_YP);
        set_jog_button(&self.jog_zp, buttons::JOG_ZP);
        set_jog_button(&self.jog_an, buttons::JOG_AN);
        set_jog_button(&self.jog_ap, buttons::JOG_AP);
        set_jog_button(&self.jog_bp, buttons::JOG_BP);
        set_jog_button(&self.jog_zn, buttons::JOG_ZN);
        set_jog_button(&self.jog_yn, buttons::JOG_YN);
        set_jog_button(&self.jog_bn, buttons::JOG_BN);
        set_jog_button(&self.jog_xp, buttons::JOG_XP);

        // Read button states for home/proghome logic
        let rapid_pressed = (!button_state & buttons::RAPID) != 0;
        let jog_xp_pressed = (!button_state & buttons::JOG_XP) != 0;
        let jog_xn_pressed = (!button_state & buttons::JOG_XN) != 0;
        let jog_yp_pressed = (!button_state & buttons::JOG_YP) != 0;
        let jog_yn_pressed = (!button_state & buttons::JOG_YN) != 0;
        let jog_zp_pressed = (!button_state & buttons::JOG_ZP) != 0;
        let jog_zn_pressed = (!button_state & buttons::JOG_ZN) != 0;

        // Set home outputs
        let _ = self.home_all.set_value(mode_home_active && rapid_pressed);
        let _ = self.home_x.set_value(mode_home_active && (jog_xp_pressed || jog_xn_pressed));
        let _ = self.home_y.set_value(mode_home_active && (jog_yp_pressed || jog_yn_pressed));
        let _ = self.home_z.set_value(mode_home_active && (jog_zp_pressed || jog_zn_pressed));

        // Set proghome outputs
        let _ = self.proghome_all.set_value(mode_proghome_active && rapid_pressed);
        let _ = self.proghome_x.set_value(mode_proghome_active && (jog_xp_pressed || jog_xn_pressed));
        let _ = self.proghome_y.set_value(mode_proghome_active && (jog_yp_pressed || jog_yn_pressed));
        let _ = self.proghome_z.set_value(mode_proghome_active && (jog_zp_pressed || jog_zn_pressed));

        // Override switches
        let feed_sw = (button_state >> 8) & 0xF;
        let rapid_sw = (button_state >> 12) & 0b11;
        let spindle_sw = (button_state >> 21) & 0b111;

        let _ = self.feed_override.set_value(feed_sw as f64 / 10.0);

        let rapid_value = match rapid_sw {
            0 => 0.0,
            1 => 0.25,
            2 => 0.5,
            3 => 1.0,
            _ => 1.0,
        };
        let _ = self.rapid_override.set_value(rapid_value);

        let spindle_value = match spindle_sw {
            0 => 0.5,
            1 => 0.6,
            2 => 0.7,
            3 => 0.8,
            4 => 0.9,
            5 => 1.0,
            6 => 1.1,
            7 => 1.2,
            _ => 1.0,
        };
        let _ = self.spindle_override.set_value(spindle_value);
    }

    /// Get LED state from HAL pins
    fn update_led_state(&self, mode_home_active: bool, mode_proghome_active: bool) -> u64 {
        let mut led_state = 0u64;

        // Helper macro to check LED pins
        macro_rules! check_led {
            ($pin:expr, $mask:expr) => {
                if let Ok(&value) = $pin.value() {
                    if value {
                        led_state |= $mask;
                    }
                }
            };
        }

        check_led!(self.led_rapid, leds::RAPID);
        check_led!(self.led_mlk, leds::MLK);
        check_led!(self.led_flush, leds::FLUSH);
        check_led!(self.led_spd_sp, leds::SPD_SP);
        check_led!(self.led_k2, leds::K2);
        check_led!(self.led_c, leds::C);
        check_led!(self.led_cycle_off, leds::CYCLE_OFF);
        check_led!(self.led_jog_xn, leds::JOG_XN);
        check_led!(self.led_bdt, leds::BDT);
        check_led!(self.led_spd_cts, leds::SPD_CTS);
        check_led!(self.led_spd_cw, leds::SPD_CW);
        check_led!(self.led_arm_man, leds::ARM_MAN);
        check_led!(self.led_b, leds::B);
        check_led!(self.led_cycle_on, leds::CYCLE_ON);
        check_led!(self.led_jog_yp, leds::JOG_YP);
        check_led!(self.led_sbk, leds::SBK);
        check_led!(self.led_coolant, leds::COOLANT);
        check_led!(self.led_mag_ccw, leds::MAG_CCW);
        check_led!(self.led_pot_dn, leds::POT_DN);
        check_led!(self.led_a, leds::A);
        check_led!(self.led_ready, leds::READY);
        check_led!(self.led_jog_zp, leds::JOG_ZP);
        check_led!(self.led_mpg_sim, leds::MPG_SIM);
        check_led!(self.led_pt_la, leds::PT_LA);
        check_led!(self.led_mag_cw, leds::MAG_CW);
        check_led!(self.led_chuck, leds::CHUCK);
        check_led!(self.led_z, leds::Z);
        check_led!(self.led_run, leds::RUN);
        check_led!(self.led_jog_an, leds::JOG_AN);
        check_led!(self.led_jog_ap, leds::JOG_AP);
        check_led!(self.led_air_blow, leds::AIR_BLOW);
        check_led!(self.led_spd_ori, leds::SPD_ORI);
        check_led!(self.led_chip_ccw, leds::CHIP_CCW);
        check_led!(self.led_y, leds::Y);
        check_led!(self.led_airblow_alarm, leds::AIRBLOW_ALARM);
        check_led!(self.led_jog_bp, leds::JOG_BP);
        check_led!(self.led_jog_zn, leds::JOG_ZN);
        check_led!(self.led_worklamp, leds::WORKLAMP);
        check_led!(self.led_k1, leds::K1);
        check_led!(self.led_chip_sp, leds::CHIP_SP);
        check_led!(self.led_x, leds::X);
        check_led!(self.led_oil_alarm, leds::OIL_ALARM);
        check_led!(self.led_safe_door, leds::SAFE_DOOR);
        check_led!(self.led_jog_yn, leds::JOG_YN);
        check_led!(self.led_drn, leds::DRN);
        check_led!(self.led_corridor, leds::CORRIDOR);
        check_led!(self.led_chip_cw, leds::CHIP_CW);
        check_led!(self.led_k4, leds::K4);
        check_led!(self.led_gear_alarm, leds::GEAR_ALARM);
        check_led!(self.led_jog_bn, leds::JOG_BN);
        check_led!(self.led_jog_xp, leds::JOG_XP);
        check_led!(self.led_opt, leds::OPT);
        check_led!(self.led_lub, leds::LUB);
        check_led!(self.led_spd_ccw, leds::SPD_CCW);
        check_led!(self.led_k3, leds::K3);
        check_led!(self.led_mc_alarm, leds::MC_ALARM);

        // Add home mode LEDs by default when in home or proghome mode
        if mode_home_active || mode_proghome_active {
            led_state |= leds::RAPID;
            led_state |= leds::JOG_XP;
            led_state |= leds::JOG_XN;
            led_state |= leds::JOG_YP;
            led_state |= leds::JOG_YN;
            led_state |= leds::JOG_ZP;
            led_state |= leds::JOG_ZN;
        }

        led_state
    }
}

// ============================================================================
// Main Function
// ============================================================================

fn main() -> Result<()> {
    let args = Args::parse();

    // Initialize logging
    if args.debug {
        env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("debug")).init();
    } else {
        env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    }

    info!("SZGH Panel HAL component starting...");
    info!("Serial port: {}@{} baud", args.port, args.baud);
    info!("Cycle time: {}s", args.cycle_time);

    // Create panel communication
    let panel_comm = PanelComm::new(args.port.clone(), args.baud);

    #[cfg(feature = "linuxcnc-hal")]
    {
        // Create HAL component
        let comp: HalComponent<SZGHPanelHal> = HalComponent::new("szgh_panel")?;
        
        info!("HAL component created and ready");

        // Get a reference to the pins
        let resources = comp.resources();
        
        // Track mode state for LED updates
        let mut current_mode: usize = 0;

        // Main communication loop
        while !comp.should_exit() {
            // Calculate mode state from last button state
            let mode_home_active = current_mode == 6;
            let mode_proghome_active = current_mode == 7;
            
            // Update LED state from HAL pins
            let led_state = resources.update_led_state(mode_home_active, mode_proghome_active);
            
            // Get 7-segment display values
            let sseg_magtool = resources.sseg_magtool.value().unwrap_or(&0) & 0xFF;
            let sseg_spdtool = (*resources.sseg_spdtool.value().unwrap_or(&0) as u32) & 0xFF;

            // Create LED bytes for panel
            let mut led_bytes = led_state_to_bytes(led_state);
            led_bytes[8] = sseg_magtool as u8;
            led_bytes[9] = sseg_spdtool as u8;

            // Communicate with panel
            match panel_comm.communicate(&led_bytes) {
                Ok(Some(response)) => {
                    // Extract button state from response
                    let button_state = extract_button_state(&response);
                    current_mode = ((button_state >> 18) & 0b111) as usize;

                    // Update HAL button pins (works through interior mutability)
                    resources.update_button_pins(button_state);
                    
                    // Update status pins
                    let _ = resources.communication_error.set_value(false);
                    let _ = resources.panel_connected.set_value(true);
                    
                    debug!(
                        "Panel communication successful - buttons: 0x{:016X}, LEDs: 0x{:016X}",
                        button_state, led_state
                    );
                }
                Ok(None) => {
                    // Communication error
                    let _ = resources.communication_error.set_value(true);
                    let _ = resources.panel_connected.set_value(false);
                    warn!("Panel communication failed - no response");
                }
                Err(e) => {
                    error!("Error in communication loop: {}", e);
                    let _ = resources.communication_error.set_value(true);
                    let _ = resources.panel_connected.set_value(false);
                }
            }

            // Sleep for cycle time
            std::thread::sleep(Duration::from_secs_f64(args.cycle_time));
        }

        info!("Shutting down SZGH Panel HAL component...");
        info!("SZGH Panel HAL component stopped");
    }

    #[cfg(not(feature = "linuxcnc-hal"))]
    {
        info!("Note: Full implementation requires LinuxCNC HAL (compile with --features linuxcnc-hal)");
        
        // Demo mode - just test serial communication
        info!("Running in demo mode...");
        let led_bytes = [0xFF; 10];
        
        for i in 0..10 {
            match panel_comm.communicate(&led_bytes) {
                Ok(Some(response)) => {
                    let button_state = extract_button_state(&response);
                    info!("Iteration {}: buttons = 0x{:016X}", i, button_state);
                }
                Ok(None) => {
                    warn!("Communication failed - no response");
                }
                Err(e) => {
                    error!("Communication error: {}", e);
                    break;
                }
            }
            std::thread::sleep(Duration::from_secs(1));
        }
    }

    Ok(())
}
