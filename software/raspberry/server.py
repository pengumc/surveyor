#! /usr/bin/env python3
import sys
import keyboard
import smbus
from time import sleep
import socketserver
import struct
import manual

class MyTCPHandler(socketserver.BaseRequestHandler):

    def handle(self):
        while 1:
            self.request.recv(1)
            try:
                i2c_data = bus.read_i2c_block_data(slave, 0, 9)
            except:
                self.request.sendall(b'\x00'*9)
                continue
            self.request.sendall(struct.pack("9B", *i2c_data))
            sys.stdout.write("\rbatt: {:.3f}, I0: {:.2f}, I1: {:.3f}, T: {:.1f}, ({})        ".format(
                ((i2c_data[1] << 8) + i2c_data[0])/64,
                ((i2c_data[3] << 8) + i2c_data[2])/64,
                ((i2c_data[5] << 8) + i2c_data[4])/64,
                ((i2c_data[7] << 8) + i2c_data[6])/64,
                i2c_data[8] == (0xFF +sum(i2c_data[0:8])) & 0xFF))


bus = smbus.SMBus(1)
slave = 0x48
speed_R = 0
speed_L = 0

if __name__ == "__main__":
    manual.map_hotkeys()
    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.TCPServer(("0.0.0.0", 12345), MyTCPHandler) as server:
        server.serve_forever()

    # while 1:
    #     data = bus.read_i2c_block_data(slave, 0, 6)
    #     sys.stdout.write("\rbatt: {:.3f}, I0: {:.2f}, I1: {:.3f}         ".format(
    #         ((data[1] << 8) + data[0])/64,
    #         ((data[3] << 8) + data[2])/64,
    #         ((data[5] << 8) + data[4])/64))
    #     sleep(0.3)
