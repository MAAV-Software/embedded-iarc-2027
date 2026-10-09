# MAAV PX4 Acceptance-Test Prototype

A small, read-only MAVLink health-check tool for a Pixhawk running PX4. It is
the starting point for MAAV's broader physical-drone acceptance-test system.

`full` currently checks, without changing vehicle state:

- USB/MAVLink heartbeat and the responding system/component IDs
- the `MAV_SYS_ID` parameter
- `AUTOPILOT_VERSION`
- battery, GPS, microSD visibility/free space, and logger status
- onboard accelerometer, gyroscope, barometer, and IMU-status topics
- magnetometer, RC/ELRS, ESC telemetry, and DShot-driver availability
- an optional complete PX4 parameter comparison against a golden JSON snapshot

It **does not arm the vehicle, command motors, alter parameters, or emit
actuator outputs**. The only vehicle-directed traffic is MAVLink requests for
existing information and telemetry intervals.

## Project layout

```
.
├── README.md
├── requirements.txt
└── src/
    └── maav_test/
        ├── __init__.py
        ├── cli.py          # command-line parsing and report formatting
        ├── connection.py   # serial discovery and MAVLink transport
        ├── models.py       # small result/report types
        ├── shell.py        # read-only PX4 NSH-over-MAVLink transport
        └── tests.py        # individual PX4 acceptance checks
```

Each future acceptance test belongs in `PX4Tests`, while MAVLink connection
and request mechanics stay in `PX4Connection`.

## Install

Python 3.10+ is recommended. From this directory:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e .
```

## Find the Pixhawk USB device on macOS

Connect the Pixhawk directly over USB, then run either:

```bash
maav-test --list-ports
```

or:

```bash
ls /dev/tty.usbmodem* /dev/cu.usbmodem* 2>/dev/null
```

The device is commonly `/dev/tty.usbmodemXXXX`. Prefer the `/dev/tty.*` name
for this first serial prototype. If no device appears, confirm that the Pixhawk
is powered and try another data-capable USB cable/port.

## Run

With the virtual environment activated:

```bash
maav-test --device /dev/tty.usbmodemXXXX full
```

Before opening the serial device or reading any MAVLink messages, every test
suite asks whether props are removed or the drone is securely strapped down.
The response is retained for that run. Answering `n` permits read-only checks,
but prevents any later motor-output suite from running.

The default serial rate is `115200`; change it only when your MAVLink USB link
has been configured differently:

```bash
maav-test --device /dev/tty.usbmodemXXXX --baud 57600 full
```

If exactly one likely USB serial device is present, `--device` can be omitted:

```bash
maav-test full
```

Useful options:

```bash
maav-test --list-ports
maav-test --device /dev/tty.usbmodemXXXX --timeout 8 full
maav-test --device /dev/tty.usbmodemXXXX --show-coordinates full
maav-test --device /dev/tty.usbmodemXXXX --require-peripherals full
```

By default, absent optional hardware (battery, GPS, external magnetometer, RC,
or ESC telemetry) is reported as `SKIP`. Use `--require-peripherals` on a
fully assembled aircraft to turn those missing signals into `FAIL` results.

## Golden parameter configurations

Export a snapshot from a reviewed, known-good aircraft. Do not treat an
unreviewed live export as a golden configuration.

```bash
maav-test --device /dev/tty.usbmodemXXXX --export-config configs/drone-03.json full
maav-test --device /dev/tty.usbmodemXXXX --golden-config configs/drone-03.json full
```

Snapshots are JSON files with numeric parameter values. The comparison reports
missing and changed golden parameters; additional vehicle parameters are allowed
so a firmware update can be reviewed deliberately rather than failing merely
because PX4 added a parameter.

## Read-only PX4 shell commands

PX4's MAVLink Shell is an NSH terminal carried in MAVLink `SERIAL_CONTROL`
messages. The prototype exposes it separately so PX4-internal diagnostics do
not become a requirement for every structured-MAVLink test.

Close QGroundControl first, then run an allowed observational command:

```bash
maav-test --device /dev/tty.usbmodemXXXX shell ver all
maav-test --device /dev/tty.usbmodemXXXX shell 'param show -q MAV_SYS_ID'
maav-test --device /dev/tty.usbmodemXXXX shell 'listener battery_status -n 5'
maav-test --device /dev/tty.usbmodemXXXX shell 'logger status'
maav-test --device /dev/tty.usbmodemXXXX shell 'dshot status'
```

The shell command has a deliberately narrow read-only allowlist: `ver`, `param
show`, `listener`, `logger status`, `dshot status`, `ls`, `df`, `free`, `top`,
and `mavlink status`. It rejects shell operators and all other commands. It does
not implement `actuator_test`, parameter writes, reboot, arming, or motor
commands.

## Motor-output test

This is the only command that can intentionally drive a motor output. It is
not part of `full`, cannot run accidentally, is capped at 15% output for five
seconds per motor, and requires an explicit terminal confirmation:

```bash
maav-test --device /dev/tty.usbmodemXXXX \
  --spin-motors \
  --motor 1 --motor-power 0.10 --motor-duration 2 motors
```

The suite asks: `Are the props removed or is the drone securely strapped down?
[y/N]`. `y`/`yes` enables motor tests later in the same run; any other response
limits the run to read-only checks. `--confirm-props-removed` remains available
only for a deliberately configured non-interactive fixture or CI job.

To test each configured motor sequentially, replace `--motor 1` with
`--all-motors`. The test uses PX4's `actuator_test`, then checks `esc_status`
and captures `vehicle_imu_status` snapshots. It cannot automatically verify
motor rotation direction without a camera, directional RPM sensor, or human
inspection; it also reports a warning rather than a false PASS if RPM telemetry
is absent.

On Linux, pass the Linux device explicitly (for example,
`--device /dev/ttyACM0`). Serial-port discovery is deliberately isolated in
`connection.py`, so adding udev naming or a different discovery policy does not
affect the tests.

## MAVLink concepts used here

### Heartbeat and targets

Vehicles periodically broadcast `HEARTBEAT`. Receiving one proves that MAVLink
frames are being decoded and gives us the vehicle's source system and component
IDs. Subsequent directed requests use those values as `target_system` and
`target_component`; they should not be assumed to be `1`.

### Parameters

`PARAM_REQUEST_READ` asks for one named parameter. PX4 answers with
`PARAM_VALUE`. `MAV_SYS_ID` is a PX4 parameter and is normally the same system
ID reported by the heartbeat, but reporting both makes a configuration mistake
visible.

### Version and telemetry

`AUTOPILOT_VERSION` is requested using the standard `MAV_CMD_REQUEST_MESSAGE`
command. Battery and GPS are ordinarily streamed telemetry messages. The tool
first asks PX4 to stream each one at 1 Hz using the standard
`MAV_CMD_SET_MESSAGE_INTERVAL`, then waits for the corresponding message.

PX4 can reject or ignore an interval request depending on firmware, component,
or message availability. Therefore a missing battery/GPS result means the tool
did not receive telemetry before its timeout; it does **not** by itself prove a
disconnected battery, failed sensor, or bad GPS.

`BATTERY_STATUS.voltages` is an array in millivolts. `current_battery` is in
10 mA units, and `battery_remaining` is a percentage when known. `GPS_RAW_INT`
reports fix quality, satellite count, and horizontal/vertical accuracy in
centimeters. Coordinates are suppressed unless `--show-coordinates` is used.

## Extending safely

Add an individual method such as `test_sd_card()` to `PX4Tests` and append it
to `run_full()`. Make it return a `TestResult` rather than printing directly;
this preserves one readable report format and makes later JSON/JUnit output
straightforward.

Motor-output, ELRS-loss/recovery, vibration-under-spin, power-cycle, and soak
tests are intentionally not implemented in `full`. They need separate commands
with a props-removed confirmation, physical safety interlock, a test fixture,
and an explicit expected result/range for each aircraft.

## RadioMaster arm test

The arm test is separate from `full`. It sends the reviewed `Command.ARM`
global-variable arm value of `1024` through the RadioMaster USB serial
interface, then waits for PX4 to report the armed flag in `HEARTBEAT`. It then
restores the ARM gvar to `-1024` (disarm), regardless of whether PX4 accepted the request. It
does not command any motor output, but an armed aircraft can respond to later
RC input, so remove props or securely restrain the vehicle before continuing.

```bash
maav-test --device /dev/tty.usbmodemXXXX \
  --controller-device /dev/cu.usbmodem00000000001B1 \
  --arm arm
```
