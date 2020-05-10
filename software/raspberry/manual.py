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
    bus.write_i2c_block_data(slave, 9, [3, 0x00, speed_R, speed_L])

def backward():
    print("backward")
    global speed_L, speed_R
    speed_R = 0x40
    speed_L = 0x40
    bus.write_i2c_block_data(slave, 9, [3, 0x03, speed_R, speed_L])

def left():
    print("left")
    global speed_L, speed_R
    speed_L = 0x40
    speed_R = 0x40
    bus.write_i2c_block_data(slave, 9, [3, 0x02, speed_R, speed_L])

def right():
    print("right")
    global speed_L, speed_R
    speed_L = 0x40
    speed_R = 0x40
    bus.write_i2c_block_data(slave, 9, [3, 0x01, speed_R, speed_L])

def stop():
    print("stop")
    global speed_L, speed_R
    speed_L = 0x00
    speed_R = 0x00
    bus.write_i2c_block_data(slave, 9, [3, 0x00, speed_R, speed_L])

def left_speed_down():
    global speed_L 
    speed_L = max(0x00, speed_L-16)
    print("left slower ", hex(speed_L))
    bus.write_i2c_block_data(slave, 11, [1, speed_L])

def left_speed_up():
    global speed_L 
    speed_L = min(0x40, speed_L+16)
    print("left faster ", hex(speed_L))
    bus.write_i2c_block_data(slave, 11, [1, speed_L])

def right_speed_down():
    global speed_R 
    speed_R = max(0x00, speed_R-16)
    print("right slower ", hex(speed_R))
    bus.write_i2c_block_data(slave, 10, [1, speed_R])

def right_speed_up():
    global speed_R 
    speed_R = min(0x40, speed_R+16)
    print("right faster ", hex(speed_R))
    bus.write_i2c_block_data(slave, 10, [1, speed_R])

def map_hotkeys():
    keyboard.add_hotkey("up arrow", forward)
    keyboard.add_hotkey("down arrow", backward)
    keyboard.add_hotkey("left arrow", left)
    keyboard.add_hotkey("right arrow", right)
    keyboard.add_hotkey("space", stop)
    keyboard.add_hotkey("z", left_speed_down)
    keyboard.add_hotkey("q", left_speed_up)
    keyboard.add_hotkey("c", right_speed_down)
    keyboard.add_hotkey("e", right_speed_up)


if __name__ == "__main__":
    map_hotkeys()
    while 1:
        data = bus.read_i2c_block_data(slave, 0, 9)
        check = (data[8] == (0xFF +sum(data[0:8])) & 0xFF)
        sys.stdout.write("\rbatt: {:.3f}, I0: {:.2f}, I1: {:.3f}, T: {:.1f}, ({})        ".format(
            ((data[1] << 8) + data[0])/64,
            ((data[3] << 8) + data[2])/64,
            ((data[5] << 8) + data[4])/64,
            ((data[7] << 8) + data[6])/64,
            check))
        if not check:
            print("\rbatt: {:.3f}, I0: {:.2f}, I1: {:.3f}, T: {:.1f}, ({})        ".format(
                ((data[1] << 8) + data[0])/64,
                ((data[3] << 8) + data[2])/64,
                ((data[5] << 8) + data[4])/64,
                ((data[7] << 8) + data[6])/64,
                check))


        sleep(0.2)
