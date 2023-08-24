import threading
import time

import serial
from dataclasses import dataclass
from threading import Thread, Lock
import copy

STATUSOK = B'\xA1'
MOTORBYTE = b'\xA2'

TIMEOUT = b'\xF1'

RIGHTFWD = b'\x01'
RIGHTRWD = b'\x02'
LEFTFWD = b'\x03'
LEFTRWD = b'\x04'
FORWARD = b'\x06'
BACKWARDS = b'\x07'
STOP = b'\x08'


class direction:
    forward = FORWARD
    backward = BACKWARDS
    left = LEFTFWD
    right = RIGHTFWD
    stop = STOP


class speed:
    full = 255
    medium_full = 200
    half = 190
    medium_slow = 120
    slow = 80


@dataclass
class command:
    dir: bytes
    spd: int


class _MotorControl:
    lock = Lock()

    block = command(dir=b'\x00', spd=0)
    flag = False

    def __init__(self):
        # Configure the serial port
        self.serial_port = serial.Serial('/dev/cu.usbserial-1430', 115200, timeout=1)

    def write_to_controller(self):
        print("write")

        while True:
            self.lock.acquire()
            if self.flag:
                tmpspd = copy.deepcopy(self.block.spd)
                tmpdir = copy.deepcopy(self.block.dir)
                self.flag = False
                self.lock.release()
                print("Speed: ", tmpspd)
                print("Direction: ", tmpdir)

                value = tmpspd.to_bytes(1, 'little')

                # Initialise connection to the board
                while True:
                    # Read byte from the serial port
                    if self.serial_port.in_waiting:
                        received_byte = self.serial_port.read(1)
                        # Process the received byte as needed
                        print(f"Received byte: {received_byte}")
                        if received_byte == STATUSOK:
                            print("Connection Initialised")
                            break

                    # Send a byte to the serial port
                    to_send_byte = b'\xA0'
                    x = self.serial_port.write(to_send_byte)
                    time.sleep(0.01)

                # time.sleep(1) # this may need to be removed

                # Send a byte to the serial port
                print("Sent initial")
                x = self.serial_port.write(MOTORBYTE)

                received_byte = self.serial_port.read(1)

                print(f"Received byte: {received_byte}")
                if received_byte == STATUSOK:
                    print("sent control Message")
                    # Transmit the message
                    self.serial_port.write(tmpdir)  # , value]
                    self.serial_port.write(value)
                    # serial_port.write([command, value])

                    # serial_port.write(message_bytes)
                    # serial_port.flush()
                    # if serial_port.in_waiting >= 2:
                    command_byte = self.serial_port.read(1)
                    if command_byte == TIMEOUT:
                        print(f"Received error byte: {command_byte}")
                    else:
                        print(f"Received control byte: {command_byte}")
                        value_byte = self.serial_port.read(1)
                        print(f"Received value byte: {value_byte}")
                        status_byte = self.serial_port.read(1)
                        print(f"Received status byte: {status_byte}")

                        # check values sent are same as received
                        if command_byte == tmpdir and value_byte == value and status_byte == STATUSOK:
                            print("all checks out!")
                        else:
                            print("checks failure...")

            else:
                self.lock.release()
            time.sleep(0.001)


class motorcontroller(_MotorControl):
    previous_message = command(dir="", spd="")

    def __init__(self):
        # thread = super().__init__(target=self.write_to_controller())
        # thread.join()

        super().__init__()

        self.thread = threading.Thread(target=self.write_to_controller, daemon=True)

        print("Motor Controller initialised")

    def move(self, dir2: bytes, spd1: int):
        tmp_message = command(dir2, spd1)

        # Check whether the new message is the same as the last, do nothing if it is. the motor controller cannot
        # handle high speed polling
        if tmp_message.dir != self.previous_message.dir and tmp_message.spd != self.previous_message.spd:
            # print("\nCommand - Dir: ", dir2, "Speed: ", spd1)

            self.lock.acquire()
            if self.flag == False:
                self.flag = True
                self.block.spd = spd1
                self.block.dir = dir2
            self.lock.release()

        self.previous_message = command(dir=dir2, spd=spd1)
