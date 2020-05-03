#! /usr/bin/env python3
import sys
import keyboard
import smbus
from time import sleep


bus = smbus.SMBus(1)
slave = 0x48

def forward():
    print("forward")
    bus.write_i2c_block_data(slave, 6, [1, 0x18])

def backward():
    print("backward")
    bus.write_i2c_block_data(slave, 6, [1, 0x24])

def left():
    print("left")
    bus.write_i2c_block_data(slave, 6, [1, 0x14])

def right():
    print("right")
    bus.write_i2c_block_data(slave, 6, [1, 0x28])

def stop():
    print("stop")
    bus.write_i2c_block_data(slave, 6, [1, 0x00])


if __name__ == "__main__":
    keyboard.add_hotkey("up arrow", forward);
    keyboard.add_hotkey("down arrow", backward);
    keyboard.add_hotkey("left arrow", left);
    keyboard.add_hotkey("right arrow", right);
    keyboard.add_hotkey("space", stop);
    while 1:
        data = bus.read_i2c_block_data(slave, 0, 6)
        sys.stdout.write("\rbatt: {}, I0: {}, I1: {}         ".format(
            (data[1] << 8) + data[0],
            (data[3] << 8) + data[2],
            (data[5] << 8) + data[4]))
        sleep(0.3)
