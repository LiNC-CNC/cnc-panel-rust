# SZGH-1080 Panel HAL for LinuxCNC

A non-realtime LinuxCNC HAL component that makes an SZGH-1080 CNC control panel
a first-class part of the machine: every button, every LED and both 7-segment
displays become HAL pins, carried over the panel's serial link.

The repository has two implementations of the same protocol:

- **`src/main.rs`** — the Rust component built here. It registers the HAL
  component `szgh_panel` and is loaded with `loadusr`.
- **`cnc_panel_hal.py`** — the original Python implementation, kept as a
  reference for the protocol and for quick hardware checks.

The component is not realtime and does not replace LinuxCNC's motion or I/O
logic. It reports what the operator pressed and lights what LinuxCNC asks for.

## How it works

The panel is polled over a serial line at 115200 baud, 8 data bits, no parity,
one stop bit. Each cycle:

1. The component sends `EB 01 80 8E` and reads the panel's 12-byte status frame
   (10 data bytes plus a MODBUS CRC-16).
2. It builds the LED image from the HAL `led_*` inputs, adds the two BCD
   7-segment values, and sends `EB 02` + 10 bytes + CRC-16, followed by
   `EB 02 81 CE`.
3. The button word from the status frame is written to the button output pins.

The default cycle time is 50 ms. `panel_connected` and `communication_error`
report the link state; a failed CRC or a short read leaves the previous button
state alone and raises the error pin.

### Pins

| Group | Direction | Names |
|---|---|---|
| Panel buttons | HAL output | `estop`, `program_protect`, `cycle_start`, `cycle_stop`, `rapid`, `jog_xp`/`jog_xn`/`jog_yp`/`jog_yn`/`jog_zp`/`jog_zn`, `jog_ap`/`jog_an`/`jog_bp`/`jog_bn`, `spd_cw`/`spd_ccw`/`spd_sp`/`spd_cts`/`spd_ori`, `coolant`, `lube`, `chuck`, `mag_cw`/`mag_ccw`, `pot_dn`, `chip_cw`/`chip_ccw`/`chip_sp`, `worklamp`, `air_blow`, `safe_door`, `drn`, `corridor`, `flush`, `sbk`, `step`, `opt`, `pt_la`, `mpg_sim`, `arm_man`, `bdt`, `mlk`, `k1`–`k4` |
| Panel LEDs | HAL input | `led_*` for the same legend, plus `led_x`/`led_y`/`led_z`/`led_a`/`led_b`/`led_c`, `led_ready`, `led_run`, `led_cycle_on`/`led_cycle_off`, `led_airblow_alarm`/`led_oil_alarm`/`led_gear_alarm`/`led_mc_alarm` |
| 7-segment displays | HAL input | `sseg_magtool`, `sseg_spdtool` (decimal values, converted to BCD) |
| Link status | HAL output | `panel_connected`, `communication_error` |

The full list is whatever `halcmd show pin szgh_panel` reports, and the bit
tables in `src/main.rs` (`buttons` and `leds`) are the authoritative mapping.

Some LEDs are composed from machine state rather than copied one-to-one: in
HOME and PROG.HOME the jog LEDs mark the axes, and in AUTO the velocity pins
light the direction the axes are moving.

## Requirements

- A LinuxCNC installation with its development files, matching the
  `linuxcnc-hal` crate — the default build links against the HAL library.
- Stable Rust and Cargo (edition 2021).
- The panel's serial adapter, normally `/dev/ttyUSB0`. The user running
  LinuxCNC needs read/write access; add it to `dialout` or install a udev rule
  for the adapter.
- Python 3 with `pyserial` and LinuxCNC's `hal` module, if you want to run the
  reference implementation.

## Build

> **Local fork note.** `Cargo.toml` carries a `[patch.crates-io]` entry that
> points at `patches/linuxcnc-hal-rs`, a checkout of the HAL bindings that is
> not part of this repository. Remove that block to build against the published
> `linuxcnc-hal` 0.3 crate, or restore the fork next to `Cargo.toml`.

```sh
cargo build --release
```

The binary is `target/release/szgh-panel-hal`.

A build without the HAL feature is useful for checking the serial link and the
button mapping on a bench machine:

```sh
cargo run --release --no-default-features -- --port /dev/ttyUSB0 --debug
```

That variant opens the port, polls the panel once a second and logs the button
word; it creates no HAL component and no pins.

## Run under LinuxCNC

```sh
loadusr -Wn szgh_panel ./target/release/szgh-panel-hal --port /dev/ttyUSB0
```

Options:

| Option | Default | Meaning |
|---|---|---|
| `-p`, `--port` | `/dev/ttyUSB0` | Serial device |
| `-b`, `--baud` | `115200` | Baud rate |
| `-c`, `--cycle-time` | `0.05` | Poll interval in seconds |
| `-v`, `--verbose` | off | Info-level logging |
| `-d`, `--debug` | off | Debug-level logging, including frame contents |

`-Wn` makes `loadusr` wait until the `szgh_panel` component exists, so pins can
be wired on the next line. Example:

```hal
loadusr -Wn szgh_panel ./target/release/szgh-panel-hal -p /dev/ttyUSB0

# Panel buttons into LinuxCNC. Wire the physical E-stop through the
# machine's safety circuit as well; a HAL pin is not a safety device.
net panel-estop        szgh_panel.estop        => ...
net panel-cycle-start  szgh_panel.cycle_start  => ...
net panel-cycle-stop   szgh_panel.cycle_stop   => ...
net panel-program-protect szgh_panel.program_protect => ...

# LinuxCNC state onto the panel lamps.
net panel-led-ready    ... => szgh_panel.led_ready
net panel-led-run      ... => szgh_panel.led_run
net panel-magtool      ... => szgh_panel.sseg_magtool
```

The component exits when LinuxCNC unloads it.

## Python reference

```sh
python3 cnc_panel_hal.py --help
python3 cnc_panel_hal.py --port /dev/ttyUSB0 --debug
```

It exposes the same pin names and speaks the same frames. It is slower to start
and needs the LinuxCNC Python environment, so the Rust component is the one to
load on a machine; the Python script is there to compare behaviour when the
panel or a wiring change misbehaves.

## Safety

This is a panel interface, not a safety-rated control. The emergency stop,
door interlock and any other safety function must stay in the machine's
hardware safety chain. A serial link, a HAL pin or a host process can fail;
none of them may be the only path that stops the machine.

## License

GPL-2.0-or-later. See [`LICENSE`](LICENSE). The software comes with no warranty;
it controls machinery and is used at your own risk.
