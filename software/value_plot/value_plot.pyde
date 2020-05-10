
import socket
import struct

s = socket.create_connection(("192.168.1.247", 12345))

y = [[0 for i in range(1000)] for k in range(3)]

def line_it(n, r, g, b):
    s = createShape(PShape.PATH)
    s.beginShape()
    s.noFill()
    s.stroke(r, g, b)
    for i in range(1000):
        y0 = 8+512-(y[n][i]) 
        s.vertex(i, y0)
    s.endShape()
    shape(s, 0, 0)

def setup():
    size(1000, 512+16)
    frameRate(30)

def draw():
    background(255)
    s.sendall(b'a')
    d = s.recv(1024)
    if d:
        vals = struct.unpack("<4H", d[0:8])
        y[0][:-1] = y[0][1:]
        y[0][-1] = vals[0]/128
        y[1][:-1] = y[1][1:]
        y[1][-1] = vals[1]/128
        y[2][:-1] = y[2][1:]
        y[2][-1] = vals[2]/128
    line_it(0, 255, 0, 0)
    line_it(1, 0, 255, 0)
    line_it(2, 0, 0, 255)
