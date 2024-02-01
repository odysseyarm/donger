#!/usr/bin/env python3

# Example file showing a basic pygame "game loop"
import pygame, sys, serial

def shiftl(d: bytes, n: int) -> bytes:
    """Shift the data to the left n bits"""
    r = [(d[0] << n) & 0xff]
    for i in range(1, len(d)):
        r[-1] |= d[i]>>(8-n)
        r.append((d[i] << n) & 0xff)
    return bytes(r)

def shiftr(d: bytes, n: int) -> bytes:
    """Shift the data to the right n bits (n<8)"""
    r = [d[0] >> n, (d[0] << (8-n)) & 0xff]
    for i in range(1, len(d)):
        r[-1] |= d[i]>>n
        r.append((d[i] << (8-n)) & 0xff)
    return bytes(r)

serial_file = sys.argv[1] if len(sys.argv) >= 2 else "/dev/ttyACM1"
ser = serial.Serial(serial_file, timeout=1)

# pygame setup
pygame.init()
screen = pygame.display.set_mode((700, 700))
clock = pygame.time.Clock()
running = True

while running:
    # poll for events
    # pygame.QUIT event means the user clicked X to close your window
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    ser.write(b"a")
    length_bytes = ser.read(2)
    length = length_bytes[0] | (length_bytes[1]<<8)
    data = ser.read(length)
    if length == 9898:
        data = shiftl(data, 4)
    elif length == 9897:
        data = shiftr(data, 3)
    else:
        print(f"weird length {length}")
        continue

    # fill the screen with a color to wipe away anything from last frame
    screen.fill("purple")

    # RENDER YOUR GAME HERE
    surface = pygame.image.frombytes(data[:98*98], (98, 98), "P")
    surface.set_palette([(x, x, x) for x in range(256)])
    scaled = pygame.transform.scale(surface, (686, 686))
    screen.blit(scaled, (0, 0))

    # flip() the display to put your work on screen
    pygame.display.flip()
    print(clock.get_fps(), "FPS")

    clock.tick(60)  # limits FPS to 60

pygame.quit()
