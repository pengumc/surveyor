#! /usr/bin/env python3
import sys
import keyboard
import smbus
from time import sleep


bus = smbus.SMBus(1)
slave = 0x48
speed_R = 0
speed_L = 0

def forward():
    print("forward")
    global speed_L, speed_R
    speed_R = 0x40
    speed_L = 0x40
    bus.write_i2c_block_data(slave, 6, [3, 0x00, speed_R, speed_L])

def backward():
    print("backward")
    global speed_L, speed_R
    speed_R = 0x40
    speed_L = 0x40
    bus.write_i2c_block_data(slave, 6, [3, 0x03, speed_R, speed_L])

def left():
    print("left")
    global speed_L, speed_R
    speed_L = 0x40
    speed_R = 0x40
    bus.write_i2c_block_data(slave, 6, [3, 0x02, speed_R, speed_L])

def right():
    print("right")
    global speed_L, speed_R
    speed_L = 0x40
    speed_R = 0x40
    bus.write_i2c_block_data(slave, 6, [3, 0x01, speed_R, speed_L])

def stop():
    print("stop")
    global speed_L, speed_R
    speed_L = 0x00
    speed_R = 0x00
    bus.write_i2c_block_data(slave, 6, [3, 0x00, speed_R, speed_L])

def left_speed_down():
    global speed_L 
    speed_L = max(0x00, speed_L-16)
    print("left slower ", hex(speed_L))
    bus.write_i2c_block_data(slave, 8, [1, speed_L])

def left_speed_up():
    global speed_L 
    speed_L = min(0x40, speed_L+16)
    print("left faster ", hex(speed_L))
    bus.write_i2c_block_data(slave, 8, [1, speed_L])

def right_speed_down():
    global speed_R 
    speed_R = max(0x00, speed_R-16)
    print("right slower ", hex(speed_R))
    bus.write_i2c_block_data(slave, 7, [1, speed_R])

def right_speed_up():
    global speed_R 
    speed_R = min(0x40, speed_R+16)
    print("right faster ", hex(speed_R))
    bus.write_i2c_block_data(slave, 7, [1, speed_R])

if __name__ == "__main__":
    keyboard.add_hotkey("up arrow", forward);
    keyboard.add_hotkey("down arrow", backward);
    keyboard.add_hotkey("left arrow", left);
    keyboard.add_hotkey("right arrow", right);
    keyboard.add_hotkey("space", stop);
    keyboard.add_hotkey("z", left_speed_down);
    keyboard.add_hotkey("q", left_speed_up);
    keyboard.add_hotkey("c", right_speed_down);
    keyboard.add_hotkey("e", right_speed_up);
    while 1:
        data = bus.read_i2c_block_data(slave, 0, 6)
        sys.stdout.write("\rbatt: {}, I0: {}, I1: {}         ".format(
            (data[1] << 8) + data[0],
            (data[3] << 8) + data[2],
            (data[5] << 8) + data[4]))
        sleep(0.3)
