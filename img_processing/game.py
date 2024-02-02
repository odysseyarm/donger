#!/usr/bin/env python3

# Example file showing a basic pygame "game loop"
from typing import Any
import pygame, sys, serial
import numpy as np
import numpy.typing as npt

def shiftl(d: bytes, n: int, out: npt.NDArray[np.byte]):
    """Shift the data to the left n bits"""
    out[0] = (d[0] << n) & 0xff
    for i in range(1, len(d)):
        if i < len(out):
            out[i-1] |= d[i]>>(8-n)
            out[i] = (d[i] << n) & 0xff

def shiftr(d: bytes, n: int, out: npt.NDArray[Any]):
    """Shift the data to the right n bits (n<8)"""
    # r = [d[0] >> n, (d[0] << (8-n)) & 0xff]
    out[0] = d[0] >> n
    out[1] = (d[0] << (8-n)) & 0xff
    for i in range(1, len(d)):
        if i+1 < len(out):
            out[i] |= d[i]>>n
            out[i+1] = (d[i] << (8-n)) & 0xff

serial_file = sys.argv[1] if len(sys.argv) >= 2 else "/dev/ttyACM1"
ser = serial.Serial(serial_file, timeout=1)

# pygame setup
pygame.init()
screen = pygame.display.set_mode((686*2 + 5, 700))
clock = pygame.time.Clock()
running = True

surfaces = [pygame.image.frombytes(bytes([0] * (98*98)), (98, 98), "P") for _ in [0,1]]
for s in surfaces:
    s.set_palette([(x, x, x) for x in range(256)])
array = np.array([[0]*98]*98, dtype=np.byte)

screen.fill("purple")
while running:
    # poll for events
    # pygame.QUIT event means the user clicked X to close your window
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    ser.write(b"a")
    id, len0, len1 = ser.read(3)
    length = len0 | (len1<<8)
    print(id, length)
    data = ser.read(9898)
    if length == 9898:
        data = shiftl(data, 4, array.reshape(98*98))
    elif length == 9897:
        data = shiftr(data, 3, array.reshape(98*98))
    else:
        print(f"weird length {length}")
        continue

    pygame.surfarray.blit_array(surfaces[id], array)
    # fill the screen with a color to wipe away anything from last frame

    # RENDER YOUR GAME HERE
    scaled = pygame.transform.scale(surfaces[id], (686, 686))
    screen.blit(scaled, (700*id, 0))

    # flip() the display to put your work on screen
    pygame.display.flip()
    print(clock.get_fps(), "FPS")

    clock.tick(60)  # limits FPS to 60

pygame.quit()
