import serial
from enum import Enum

SERIAL_PORT = 'COM7'


class Command(Enum):
    ARM = 0,
    ROLL = 1,
    PITCH = 2,
    THROTTLE = 3,
    YAW = 4,
    MODE = 5,
    KILL = 6,
    


def send_message(gvar_idx, val, port=SERIAL_PORT, baudrate=115200):
    message = (gvar_idx.value[0] << 12) | (val + 1024)
    message = message.to_bytes(2, byteorder="little")
    
    with serial.Serial(port, baudrate, timeout=1) as ser:
        ser.write(message)
        ser.flush()


def send_percentage(gvar_idx, percentage):
    send_message(gvar_idx, int((2048 * percentage) - 1024))
