# MAAV PX4 Test Tool

A compact command-line tool for checking a PX4 flight controller and its RC
link. Read-only checks are the default. Motor and arm checks are separate
commands that always require an interactive physical-safety confirmation.

## Install

Python 3.10 or newer is required.

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e .
```

## Find the Pixhawk

```bash
maav-test --list-ports
```

On macOS, use the Pixhawk's `/dev/tty.usbmodem...` device. Close
QGroundControl or any other program using that port before running a check.

## Everyday commands

Run the normal read-only check:

```bash
maav-test --device /dev/tty.usbmodemXXXX full
```

If only one plausible Pixhawk device is connected, omit `--device`:

```bash
maav-test full
```

Read one PX4 topic through the MAVLink shell:

```bash
maav-test --device /dev/tty.usbmodemXXXX shell 'listener input_rc -n 1'
```

The shell accepts only observational commands such as `listener`, `ver`,
`logger status`, `dshot status`, `param show`, `df`, and `mavlink status`.

## Arm-switch test

With props removed or the vehicle restrained, run:

```bash
maav-test --device /dev/tty.usbmodemXXXX arm /dev/cu.usbmodem00000000001B1
```

The command prompts for safety confirmation, sends the RadioMaster ARM gvar
at `1024`, checks PX4's armed flag, then always restores the gvar to `-1024`
(disarm). It does not command motor outputs. A failed arm test can be a correct
PX4 response when preflight checks are failing.

## Motor test

Use only with props removed or the vehicle securely restrained:

```bash
maav-test --device /dev/tty.usbmodemXXXX motors --motor 1
```

To test every configured motor:

```bash
maav-test --device /dev/tty.usbmodemXXXX motors --all
```

Motor output is limited to 10% for two seconds by default. Adjust it only when
needed, within the enforced limits:

```bash
maav-test --device /dev/tty.usbmodemXXXX motors --motor 1 --power 0.10 --duration 2
```

## Less-common options

These options are for diagnosing unusual connections or reviewing a known-good
configuration:

```bash
maav-test --baud 57600 full
maav-test --timeout 8 full
maav-test full --golden-config configs/drone-03.json
maav-test full --export-config configs/drone-03.json
maav-test full --require-peripherals
```

Use `maav-test --help` or `maav-test COMMAND --help` for the complete help for
one command.
