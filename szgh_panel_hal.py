#!/usr/bin/env python3
"""
SZGH-1080 Panel HAL Component
Non-realtime LinuxCNC HAL component for SZGH-1080 CNC control panel

This component exposes all panel buttons and LEDs to the LinuxCNC HAL,
providing complete integration between the physical panel and LinuxCNC.

Based on the proven szgh-1080-panel.py communication protocol.
"""

import hal
import time
import serial
import struct
import sys
import os
import signal
from threading import Thread, Event
import logging

# Set up logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("szgh_panel_hal")


# MODBUS CRC-16 function (from original code)
def modbus_crc16(data):
    """Calculate MODBUS CRC-16 for the given data bytes"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc.to_bytes(2, byteorder="little")  # MODBUS uses little-endian


# Button and LED mappings (from original C code)
class Buttons:
    PROGRAM_PROTECT = 1 << 0
    ESTOP = 1 << 7
    CYCLE_START = 1 << 14
    CYCLE_STOP = 1 << 15
    RAPID = 1 << 63
    MLK = 1 << 55
    FLUSH = 1 << 47
    SPD_SP = 1 << 39
    K2 = 1 << 34
    JOG_XN = 1 << 62
    BDT = 1 << 54
    SPD_CTS = 1 << 46
    SPD_CW = 1 << 38
    ARM_MAN = 1 << 30
    JOG_YP = 1 << 61
    SBK = 1 << 53
    COOLANT = 1 << 45
    MAG_CCW = 1 << 37
    POT_DN = 1 << 29
    JOG_ZP = 1 << 60
    MPG_SIM = 1 << 52
    PT_LA = 1 << 44
    MAG_CW = 1 << 36
    CHUCK = 1 << 29
    JOG_AN = 1 << 59
    JOG_AP = 1 << 51
    AIR_BLOW = 1 << 43
    SPD_ORI = 1 << 35
    CHIP_CCW = 1 << 27
    JOG_BP = 1 << 58
    JOG_ZN = 1 << 50
    WORKLAMP = 1 << 42
    K1 = 1 << 34
    CHIP_SP = 1 << 26
    SAFE_DOOR = 1 << 57
    JOG_YN = 1 << 49
    DRN = 1 << 41
    CORRIDOR = 1 << 33
    CHIP_CW = 1 << 25
    K4 = 1 << 17
    JOG_BN = 1 << 56
    JOG_XP = 1 << 48
    OPT = 1 << 40
    LUB = 1 << 31
    SPD_CCW = 1 << 24
    K3 = 1 << 16


class Leds:
    RAPID = 1 << 0
    MLK = 1 << 1
    FLUSH = 1 << 2
    SPD_SP = 1 << 3
    K2 = 1 << 4
    C = 1 << 5
    CYCLE_OFF = 1 << 6
    JOG_XN = 1 << 8
    BDT = 1 << 9
    SPD_CTS = 1 << 10
    SPD_CW = 1 << 11
    ARM_MAN = 1 << 12
    B = 1 << 13
    CYCLE_ON = 1 << 14
    JOG_YP = 1 << 16
    SBK = 1 << 17
    COOLANT = 1 << 18
    MAG_CCW = 1 << 19
    POT_DN = 1 << 20
    A = 1 << 21
    READY = 1 << 22
    JOG_ZP = 1 << 24
    MPG_SIM = 1 << 25
    PT_LA = 1 << 26
    MAG_CW = 1 << 27
    CHUCK = 1 << 28
    Z = 1 << 29
    RUN = 1 << 30
    JOG_AN = 1 << 32
    JOG_AP = 1 << 33
    AIR_BLOW = 1 << 34
    SPD_ORI = 1 << 35
    CHIP_CCW = 1 << 36
    Y = 1 << 37
    AIRBLOW_ALARM = 1 << 38
    JOG_BP = 1 << 40
    JOG_ZN = 1 << 41
    WORKLAMP = 1 << 42
    K1 = 1 << 43
    CHIP_SP = 1 << 44
    X = 1 << 45
    OIL_ALARM = 1 << 46
    SAFE_DOOR = 1 << 48
    JOG_YN = 1 << 49
    DRN = 1 << 50
    CORRIDOR = 1 << 51
    CHIP_CW = 1 << 52
    K4 = 1 << 53
    GEAR_ALARM = 1 << 54
    JOG_BN = 1 << 56
    JOG_XP = 1 << 57
    OPT = 1 << 58
    LUB = 1 << 59
    SPD_CCW = 1 << 60
    K3 = 1 << 61
    MC_ALARM = 1 << 62


class SZGHPanelHAL:
    def __init__(
        self, port="/dev/ttyUSB0", baud=115200, cycle_time=0.05, verbose=False
    ):
        self.port = port
        self.baud = baud
        self.cycle_time = cycle_time
        self.verbose = verbose
        self.ser = None
        self.running = True
        self.shutdown_event = Event()

        # Current states
        self.button_state = 0
        self.led_state = 0
        self.sseg_magtool = 0
        self.sseg_spdtool = 0
        self.communication_error = False
        self.panel_connected = False
        self.mode_home_active = False
        self.mode_proghome_active = False

        # Mode switch state tracking for pulse generation
        self.mode_switch_state = 0
        self.mode_pin_names = [
            "mode_edit",
            "mode_auto",
            "mode_mdi",
            "mode_jog",
            "mode_mpg",
            "mode_inc",
            "mode_home",
            "mode_proghome",
        ]

        # HAL component
        self.h = None

    def log(self, message, level=logging.INFO):
        if self.verbose or level >= logging.WARNING:
            logger.log(level, message)

    def connect_serial(self):
        """Connect to the serial port"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1,
            )
            self.log(f"Connected to serial port {self.port}@{self.baud}")
            return True
        except serial.SerialException as e:
            self.log(
                f"Failed to connect to serial port {self.port}: {e}", logging.ERROR
            )
            return False

    def disconnect_serial(self):
        """Disconnect from serial port"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.log("Disconnected from serial port")

    def modbus_crc16(self, data):
        """Calculate MODBUS CRC-16 for the given data bytes"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc.to_bytes(2, byteorder="little")

    @staticmethod
    def dec_to_bcd_hex(value):
        """Convert decimal number to BCD-like hex representation.
        
        Treats decimal digits as hex digits:
        - 42 decimal -> 0x42 (tens=4, ones=2 -> 4*16 + 2 = 66)
        - 99 decimal -> 0x99 (tens=9, ones=9 -> 9*16 + 9 = 153)
        
        Args:
            value: Integer 0-99
            
        Returns:
            Integer with BCD-like hex encoding (0x00-0x99)
        """
        # Clamp to valid range
        value = max(0, min(99, int(value)))
        
        # Extract decimal digits
        tens = value // 10
        ones = value % 10
        
        # Combine as if they were hex digits
        return (tens << 4) | ones

    def led_state_to_bytes(self, leds):
        """Convert LED flags to 10-byte array (like Python)"""
        led_state = (1 << 64) - 1  # UINT64_MAX equivalent
        led_state = led_state & ~leds
        led_state <<= 16  # Shift left by 16 bits like Python

        output = bytearray(10)
        for i in range(10):
            output[i] = (led_state >> ((9 - i) * 8)) & 0xFF
        return output

    def panel_state(self, led_state_bytes):
        """Communicate with panel - send LED state, read button state"""
        self.connect_serial()

        try:
            # First write: 'eb 01 80 8e'
            first_command = bytes([0xEB, 0x01, 0x80, 0x8E])
            self.ser.write(first_command)
            self.ser.flush()

            # Read 12 bytes
            response = self.ser.read(12)
            if len(response) != 12:
                self.log(
                    f"Incomplete response: got {len(response)} bytes, expected 12",
                    logging.WARNING,
                )
                return None

            # Check CRC of the first response (keyboard state)
            # The response format is: [header bytes] + [data bytes] + [CRC bytes]
            # CRC should be calculated over the first 10 bytes, last 2 bytes are CRC
            if len(response) == 12:
                response_data = response[:10]  # First 10 bytes (data)
                response_crc = response[10:12]  # Last 2 bytes (CRC)
                calculated_crc = self.modbus_crc16(response_data)

                # Compare CRCs (note: response CRC might be in different endianness)
                if (
                    response_crc != calculated_crc
                    and response_crc != calculated_crc[::-1]
                ):
                    self.log(
                        f"CRC check failed for keyboard state response. "
                        f"Expected: {calculated_crc.hex()} or {calculated_crc[::-1].hex()}, "
                        f"Got: {response_crc.hex()}",
                        logging.WARNING,
                    )
                    return None

                self.log(f"CRC check passed for keyboard state response", logging.DEBUG)

            # Second write: 'eb 02' + 10 variable bytes + CRC16
            second_command_prefix = bytes([0xEB, 0x02])

            # Calculate CRC-16 of the 12 bytes (prefix + data)
            crc_data = second_command_prefix + led_state_bytes
            crc = self.modbus_crc16(crc_data)

            # Build complete second command
            second_command = crc_data + crc[::-1]  # Reverse CRC bytes

            # Final write: 'eb 02 81 ce'
            final_command = bytes([0xEB, 0x02, 0x81, 0xCE])

            # Send combined command
            self.ser.write(second_command + final_command)
            self.ser.flush()
            self.disconnect_serial()
            return response

        except serial.SerialException as e:
            self.log(f"Serial communication error: {e}", logging.ERROR)
            return None

    def extract_button_state(self, response):
        """Extract button state from response bytes 2-9"""
        if not response or len(response) < 10:
            return 0

        button_state = 0
        for i in range(2, 10):
            button_state |= (response[i]) << ((9 - i) * 8)
        return button_state

    def create_hal_pins(self):
        """Create all HAL pins for the panel"""
        # Create HAL component
        self.h = hal.component("szgh_panel")

        # Button output pins (panel → LinuxCNC)
        button_pins = [
            "program_protect",
            "estop",
            "cycle_start",
            "cycle_stop",
            "rapid",
            "mlk",
            "flush",
            "spd_sp",
            "k2",
            "jog_xn",
            "bdt",
            "spd_cts",
            "spd_cw",
            "arm_man",
            "jog_yp",
            "sbk",
            "coolant",
            "mag_ccw",
            "pot_dn",
            "jog_zp",
            "mpg_sim",
            "pt_la",
            "mag_cw",
            "chuck",
            "jog_an",
            "jog_ap",
            "air_blow",
            "spd_ori",
            "chip_ccw",
            "jog_bp",
            "jog_zn",
            "worklamp",
            "k1",
            "chip_sp",
            "safe_door",
            "jog_yn",
            "drn",
            "corridor",
            "chip_cw",
            "k4",
            "jog_bn",
            "jog_xp",
            "opt",
            "lube",
            "spd_ccw",
            "k3",
        ]

        for pin_name in button_pins:
            self.h.newpin(pin_name, hal.HAL_BIT, hal.HAL_OUT)

        # LED input pins (LinuxCNC → panel)
        led_pins = [
            "led_rapid",
            "led_mlk",
            "led_flush",
            "led_spd_sp",
            "led_k2",
            "led_c",
            "led_cycle_off",
            "led_jog_xn",
            "led_bdt",
            "led_spd_cts",
            "led_spd_cw",
            "led_arm_man",
            "led_b",
            "led_cycle_on",
            "led_jog_yp",
            "led_sbk",
            "led_coolant",
            "led_mag_ccw",
            "led_pot_dn",
            "led_a",
            "led_ready",
            "led_jog_zp",
            "led_mpg_sim",
            "led_pt_la",
            "led_mag_cw",
            "led_chuck",
            "led_z",
            "led_run",
            "led_jog_an",
            "led_jog_ap",
            "led_air_blow",
            "led_spd_ori",
            "led_chip_ccw",
            "led_y",
            "led_airblow_alarm",
            "led_jog_bp",
            "led_jog_zn",
            "led_worklamp",
            "led_k1",
            "led_chip_sp",
            "led_x",
            "led_oil_alarm",
            "led_safe_door",
            "led_jog_yn",
            "led_drn",
            "led_corridor",
            "led_chip_cw",
            "led_k4",
            "led_gear_alarm",
            "led_jog_bn",
            "led_jog_xp",
            "led_opt",
            "led_lub",
            "led_spd_ccw",
            "led_k3",
            "led_mc_alarm",
        ]

        for pin_name in led_pins:
            self.h.newpin(pin_name, hal.HAL_BIT, hal.HAL_IN)

        # 7-segment display pins (u32 instead of u8 for HAL compatibility)
        self.h.newpin("sseg_magtool", hal.HAL_S32, hal.HAL_IN)
        self.h.newpin("sseg_spdtool", hal.HAL_S32, hal.HAL_IN)

        # Mode switch output pins (8 modes, each outputs 0.5s pulse when selected)
        for mode_name in self.mode_pin_names:
            self.h.newpin(mode_name, hal.HAL_BIT, hal.HAL_OUT)

        # Home output pins (active when mode_home and corresponding jog/rapid button pressed)
        self.h.newpin("home-all", hal.HAL_BIT, hal.HAL_OUT)
        self.h.newpin("home-x", hal.HAL_BIT, hal.HAL_OUT)
        self.h.newpin("home-y", hal.HAL_BIT, hal.HAL_OUT)
        self.h.newpin("home-z", hal.HAL_BIT, hal.HAL_OUT)

        # Proghome output pins (active when mode_proghome and corresponding jog/rapid button pressed)
        self.h.newpin("proghome-all", hal.HAL_BIT, hal.HAL_OUT)
        self.h.newpin("proghome-x", hal.HAL_BIT, hal.HAL_OUT)
        self.h.newpin("proghome-y", hal.HAL_BIT, hal.HAL_OUT)
        self.h.newpin("proghome-z", hal.HAL_BIT, hal.HAL_OUT)

        self.h.newpin("feed_override", hal.HAL_FLOAT, hal.HAL_OUT)
        self.h.newpin("rapid_override", hal.HAL_FLOAT, hal.HAL_OUT)
        self.h.newpin("spindle_override", hal.HAL_FLOAT, hal.HAL_OUT)

        # Velocity input pins (LinuxCNC → panel)
        self.h.newpin("vel_x", hal.HAL_FLOAT, hal.HAL_IN)
        self.h.newpin("vel_y", hal.HAL_FLOAT, hal.HAL_IN)
        self.h.newpin("vel_z", hal.HAL_FLOAT, hal.HAL_IN)

        # Status pins
        self.h.newpin("communication_error", hal.HAL_BIT, hal.HAL_OUT)
        self.h.newpin("panel_connected", hal.HAL_BIT, hal.HAL_OUT)

        # Ready the component
        self.h.ready()
        self.log("HAL component created and ready")

    def update_button_pins(self, button_state):
        """Update all button HAL pins from button state"""
        if not self.h:
            return

        # Update each button pin based on button state
        button_map = {
            "program_protect": Buttons.PROGRAM_PROTECT,
            "estop": Buttons.ESTOP,
            "cycle_start": Buttons.CYCLE_START,
            "cycle_stop": Buttons.CYCLE_STOP,
            "rapid": Buttons.RAPID,
            "mlk": Buttons.MLK,
            "flush": Buttons.FLUSH,
            "spd_sp": Buttons.SPD_SP,
            "k2": Buttons.K2,
            "jog_xn": Buttons.JOG_XN,
            "bdt": Buttons.BDT,
            "spd_cts": Buttons.SPD_CTS,
            "spd_cw": Buttons.SPD_CW,
            "arm_man": Buttons.ARM_MAN,
            "jog_yp": Buttons.JOG_YP,
            "sbk": Buttons.SBK,
            "coolant": Buttons.COOLANT,
            "mag_ccw": Buttons.MAG_CCW,
            "pot_dn": Buttons.POT_DN,
            "jog_zp": Buttons.JOG_ZP,
            "mpg_sim": Buttons.MPG_SIM,
            "pt_la": Buttons.PT_LA,
            "mag_cw": Buttons.MAG_CW,
            "chuck": Buttons.CHUCK,
            "jog_an": Buttons.JOG_AN,
            "jog_ap": Buttons.JOG_AP,
            "air_blow": Buttons.AIR_BLOW,
            "spd_ori": Buttons.SPD_ORI,
            "chip_ccw": Buttons.CHIP_CCW,
            "jog_bp": Buttons.JOG_BP,
            "jog_zn": Buttons.JOG_ZN,
            "worklamp": Buttons.WORKLAMP,
            "k1": Buttons.K1,
            "chip_sp": Buttons.CHIP_SP,
            "safe_door": Buttons.SAFE_DOOR,
            "jog_yn": Buttons.JOG_YN,
            "drn": Buttons.DRN,
            "corridor": Buttons.CORRIDOR,
            "chip_cw": Buttons.CHIP_CW,
            "k4": Buttons.K4,
            "jog_bn": Buttons.JOG_BN,
            "jog_xp": Buttons.JOG_XP,
            "opt": Buttons.OPT,
            "lube": Buttons.LUB,
            "spd_ccw": Buttons.SPD_CCW,
            "k3": Buttons.K3,
        }

        # update switches first to know current mode
        current_mode = (button_state >> 18) & 0b111
        self.mode_switch_state = current_mode

        found_active = False
        for i, mode_name in enumerate(self.mode_pin_names):
            # make sure only one mode is active at any time
            active = i == current_mode and not found_active
            if active:
                found_active = True
            self.h[mode_name] = active

        # Home mode logic: when mode_home is active, jog buttons control home outputs
        self.mode_home_active = (current_mode == 6)
        self.mode_proghome_active = (current_mode == 7)
        home_or_proghome_active = self.mode_home_active or self.mode_proghome_active

        # Jog buttons that should not output in home or proghome mode
        jog_pins = {"jog_xn", "jog_yp", "jog_zp", "jog_an", "jog_ap",
                    "jog_bp", "jog_zn", "jog_yn", "jog_bn", "jog_xp"}

        for pin_name, button_mask in button_map.items():
            if pin_name in jog_pins and home_or_proghome_active:
                self.h[pin_name] = False
            else:
                self.h[pin_name] = bool(~button_state & button_mask)

        # Read button states
        rapid_pressed = bool(~button_state & Buttons.RAPID)
        jog_xp_pressed = bool(~button_state & Buttons.JOG_XP)
        jog_xn_pressed = bool(~button_state & Buttons.JOG_XN)
        jog_yp_pressed = bool(~button_state & Buttons.JOG_YP)
        jog_yn_pressed = bool(~button_state & Buttons.JOG_YN)
        jog_zp_pressed = bool(~button_state & Buttons.JOG_ZP)
        jog_zn_pressed = bool(~button_state & Buttons.JOG_ZN)

        # Set home outputs based on mode_home and button presses
        self.h["home-all"] = self.mode_home_active and rapid_pressed
        self.h["home-x"] = self.mode_home_active and (jog_xp_pressed or jog_xn_pressed)
        self.h["home-y"] = self.mode_home_active and (jog_yp_pressed or jog_yn_pressed)
        self.h["home-z"] = self.mode_home_active and (jog_zp_pressed or jog_zn_pressed)

        # Set proghome outputs based on mode_proghome and button presses
        self.h["proghome-all"] = self.mode_proghome_active and rapid_pressed
        self.h["proghome-x"] = self.mode_proghome_active and (jog_xp_pressed or jog_xn_pressed)
        self.h["proghome-y"] = self.mode_proghome_active and (jog_yp_pressed or jog_yn_pressed)
        self.h["proghome-z"] = self.mode_proghome_active and (jog_zp_pressed or jog_zn_pressed)

        feed_sw = (button_state >> 8) & 0xF
        rapid_sw = (button_state >> 12) & 0b11
        spindle_sw = (button_state >> 21) & 0b111

        # Update override float values for qtpyvcp
        self.h["feed_override"] = feed_sw / 10.0
        rapid_values = {0: 0.0, 1: 0.25, 2: 0.5, 3: 1.0}
        self.h["rapid_override"] = rapid_values.get(rapid_sw, 1.0)
        spindle_values = {
            0: 0.5,
            1: 0.6,
            2: 0.7,
            3: 0.8,
            4: 0.9,
            5: 1.0,
            6: 1.1,
            7: 1.2,
        }
        self.h["spindle_override"] = spindle_values.get(spindle_sw, 1.0)

    def update_led_state(self):
        """Get LED state from HAL pins"""
        led_map = {
            "led_rapid": Leds.RAPID,
            "led_mlk": Leds.MLK,
            "led_flush": Leds.FLUSH,
            "led_spd_sp": Leds.SPD_SP,
            "led_k2": Leds.K2,
            "led_c": Leds.C,
            "led_cycle_off": Leds.CYCLE_OFF,
            "led_jog_xn": Leds.JOG_XN,
            "led_bdt": Leds.BDT,
            "led_spd_cts": Leds.SPD_CTS,
            "led_spd_cw": Leds.SPD_CW,
            "led_arm_man": Leds.ARM_MAN,
            "led_b": Leds.B,
            "led_cycle_on": Leds.CYCLE_ON,
            "led_jog_yp": Leds.JOG_YP,
            "led_sbk": Leds.SBK,
            "led_coolant": Leds.COOLANT,
            "led_mag_ccw": Leds.MAG_CCW,
            "led_pot_dn": Leds.POT_DN,
            "led_a": Leds.A,
            "led_ready": Leds.READY,
            "led_jog_zp": Leds.JOG_ZP,
            "led_mpg_sim": Leds.MPG_SIM,
            "led_pt_la": Leds.PT_LA,
            "led_mag_cw": Leds.MAG_CW,
            "led_chuck": Leds.CHUCK,
            "led_z": Leds.Z,
            "led_run": Leds.RUN,
            "led_jog_an": Leds.JOG_AN,
            "led_jog_ap": Leds.JOG_AP,
            "led_air_blow": Leds.AIR_BLOW,
            "led_spd_ori": Leds.SPD_ORI,
            "led_chip_ccw": Leds.CHIP_CCW,
            "led_y": Leds.Y,
            "led_airblow_alarm": Leds.AIRBLOW_ALARM,
            "led_jog_bp": Leds.JOG_BP,
            "led_jog_zn": Leds.JOG_ZN,
            "led_worklamp": Leds.WORKLAMP,
            "led_k1": Leds.K1,
            "led_chip_sp": Leds.CHIP_SP,
            "led_x": Leds.X,
            "led_oil_alarm": Leds.OIL_ALARM,
            "led_safe_door": Leds.SAFE_DOOR,
            "led_jog_yn": Leds.JOG_YN,
            "led_drn": Leds.DRN,
            "led_corridor": Leds.CORRIDOR,
            "led_chip_cw": Leds.CHIP_CW,
            "led_k4": Leds.K4,
            "led_gear_alarm": Leds.GEAR_ALARM,
            "led_jog_bn": Leds.JOG_BN,
            "led_jog_xp": Leds.JOG_XP,
            "led_opt": Leds.OPT,
            "led_lub": Leds.LUB,
            "led_spd_ccw": Leds.SPD_CCW,
            "led_k3": Leds.K3,
            "led_mc_alarm": Leds.MC_ALARM,
        }

        # LEDs that are on by default in home mode (ignore HAL input pins)
        home_mode_leds = {
            "led_rapid": Leds.RAPID,
            "led_jog_xp": Leds.JOG_XP,
            "led_jog_xn": Leds.JOG_XN,
            "led_jog_yp": Leds.JOG_YP,
            "led_jog_yn": Leds.JOG_YN,
            "led_jog_zp": Leds.JOG_ZP,
            "led_jog_zn": Leds.JOG_ZN,
        }

        # Start with LEDs from HAL pins
        led_state = 0
        for pin_name, led_mask in led_map.items():
            if self.h and self.h[pin_name]:
                led_state |= led_mask

        # Add home mode LEDs by default when in home or proghome mode (ignore HAL pins)
        if self.mode_home_active or self.mode_proghome_active:
            for led_mask in home_mode_leds.values():
                led_state |= led_mask

        # Add velocity-based LEDs only in mode_auto (mode == 1)
        if self.mode_switch_state == 1:
            if self.h:
                vel_x = self.h["vel_x"]
                vel_y = self.h["vel_y"]
                vel_z = self.h["vel_z"]
                
                # X axis: negative velocity -> JOG_XN, positive -> JOG_XP
                if vel_x < 0:
                    led_state |= Leds.JOG_XN
                elif vel_x > 0:
                    led_state |= Leds.JOG_XP
                
                # Y axis: negative velocity -> JOG_YN, positive -> JOG_YP
                if vel_y < 0:
                    led_state |= Leds.JOG_YN
                elif vel_y > 0:
                    led_state |= Leds.JOG_YP
                
                # Z axis: negative velocity -> JOG_ZN, positive -> JOG_ZP
                if vel_z < 0:
                    led_state |= Leds.JOG_ZN
                elif vel_z > 0:
                    led_state |= Leds.JOG_ZP

        return led_state

    def update_7segment_displays(self):
        """Update 7-segment display values from HAL pins"""
        # Mask to 8 bits for 7-segment display
        if self.h:
            self.sseg_magtool = self.dec_to_bcd_hex(self.h["sseg_magtool"] & 0xFF)
            self.sseg_spdtool = self.dec_to_bcd_hex(self.h["sseg_spdtool"] & 0xFF)

    def communication_loop(self):
        """Main communication loop with the panel"""
        while self.running and not self.shutdown_event.is_set():
            try:
                # Update LED state from HAL pins
                self.led_state = self.update_led_state()
                self.update_7segment_displays()

                # Create LED bytes for panel
                led_bytes = self.led_state_to_bytes(self.led_state)
                led_bytes = bytearray(led_bytes)
                led_bytes[8] = self.sseg_magtool
                led_bytes[9] = self.sseg_spdtool

                # Communicate with panel
                response = self.panel_state(led_bytes)

                if response:
                    # Extract button state from response
                    self.button_state = self.extract_button_state(response)

                    # Update HAL button pins
                    self.update_button_pins(self.button_state)

                    # Update status pins
                    if self.h:
                        self.h["communication_error"] = False
                        self.h["panel_connected"] = True

                    if self.verbose:
                        self.log(
                            f"Panel communication successful - buttons: 0x{self.button_state:016X}, LEDs: 0x{self.led_state:016X}"
                        )

                else:
                    # Communication error
                    if self.h:
                        self.h["communication_error"] = True
                        self.h["panel_connected"] = False
                    self.log("Panel communication failed", logging.WARNING)

                # Sleep for cycle time
                time.sleep(self.cycle_time)

            except KeyboardInterrupt:
                self.log("Received interrupt signal, shutting down...")
                break
            except Exception as e:
                self.log(f"Error in communication loop: {e}", logging.ERROR)
                if self.h:
                    self.h["communication_error"] = True
                    self.h["panel_connected"] = False
                time.sleep(self.cycle_time)

    def signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        self.log(f"Received signal {signum}, shutting down...")
        self.running = False
        self.shutdown_event.set()

    def run(self):
        """Main run method"""
        # Set up signal handlers
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

        # Connect to serial port
        if not self.connect_serial():
            self.log("Failed to connect to serial port, exiting", logging.ERROR)
            return 1

        try:
            # Create HAL pins
            self.create_hal_pins()

            self.log("SZGH Panel HAL component started successfully")
            self.log(f"Serial port: {self.port}@{self.baud}")
            self.log(f"Cycle time: {self.cycle_time}s")

            # Start communication loop
            self.communication_loop()

        except Exception as e:
            self.log(f"Fatal error: {e}", logging.ERROR)
            return 1

        finally:
            # Cleanup
            self.log("Shutting down SZGH Panel HAL component...")
            self.disconnect_serial()
            if self.h:
                self.h.exit()
            self.log("SZGH Panel HAL component stopped")

        return 0


def main():
    """Main entry point"""
    import argparse

    parser = argparse.ArgumentParser(description="SZGH-1080 Panel HAL Component")
    parser.add_argument(
        "-p",
        "--port",
        default="/dev/ttyUSB0",
        help="Serial port device (default: /dev/ttyUSB0)",
    )
    parser.add_argument(
        "-b", "--baud", type=int, default=115200, help="Baud rate (default: 115200)"
    )
    parser.add_argument(
        "-c",
        "--cycle-time",
        type=float,
        default=0.05,
        help="Cycle time in seconds (default: 0.05)",
    )
    parser.add_argument(
        "-v", "--verbose", action="store_true", help="Enable verbose output"
    )
    parser.add_argument(
        "-d", "--debug", action="store_true", help="Enable debug logging"
    )

    args = parser.parse_args()

    if args.debug:
        logger.setLevel(logging.DEBUG)

    # Create and run the component
    panel = SZGHPanelHAL(
        port=args.port, baud=args.baud, cycle_time=args.cycle_time, verbose=args.verbose
    )

    return panel.run()


if __name__ == "__main__":
    sys.exit(main())
