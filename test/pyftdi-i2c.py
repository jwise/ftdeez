from pyftdi import FtdiLogger
import logging

logging.getLogger().setLevel(logging.INFO)
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)
ch.setFormatter(logging.Formatter("[%(asctime)s] %(name)s: %(levelname)s: %(message)s"))
logging.getLogger().addHandler(ch)

FtdiLogger.set_level(logging.DEBUG)
#FtdiLogger.log.addHandler(logging.StreamHandler)

import pyftdi.i2c

i2c = pyftdi.i2c.I2cController()
i2c.configure('ftdi://ftdi:2232:123456/1', frequency=50000)

STATUS = 0x00
STATUS_HW_ID = 0x01
STATUS_VERSION = 0x02
GPIO = 0x01
GPIO_DIRCLR = 0x03
GPIO_SET = 0x05
GPIO_PULLENSET = 0x0B
NEOPIXEL = 0x0E
NEOPIXEL_PIN = 0x01
NEOPIXEL_BUF_LENGTH = 0x03
NEOPIXEL_BUF = 0x04
NEOPIXEL_SHOW = 0x05

def seesaw_read(port, base, fn, n):
    port.write(bytes([base, fn]))
    return port.read(n)

def seesaw_write(port, base, fn, bs):
    port.write(bytes([base, fn]) + bs)

port_30 = i2c.get_port(0x30)
print(seesaw_read(port_30, STATUS, STATUS_HW_ID, 0x01))
vers = seesaw_read(port_30, STATUS, STATUS_VERSION, 0x04)

seesaw_write(port_30, GPIO, GPIO_DIRCLR, b'\x00\x00\x00\xF0')
seesaw_write(port_30, GPIO, GPIO_SET, b'\x00\x00\x00\xF0')
seesaw_write(port_30, GPIO, GPIO_PULLENSET, b'\x00\x00\x00\xF0')

def wr_npxl(bs):
    seesaw_write(port_30, NEOPIXEL, NEOPIXEL_PIN, b'\x03')
    seesaw_write(port_30, NEOPIXEL, NEOPIXEL_BUF_LENGTH, bytes([0, 12]))
    seesaw_write(port_30, NEOPIXEL, NEOPIXEL_BUF, b'\x00\x00' + bs)
    seesaw_write(port_30, NEOPIXEL, NEOPIXEL_SHOW, b'')

import time
ar = [[0, 1, 16], [1, 16, 16], [16, 16, 16], [0, 16, 16]]
arofs = 0
while True:
    pxs = []
    for n in range(4):
        pxs += ar[(n + arofs) % 4]
    arofs += 1
    wr_npxl(bytes(pxs))
    time.sleep(0.3) 

dcode = (vers[2] << 8) | vers[3]
print(f"year {dcode & 0x7F}, month {(dcode >> 7) & 0xF}, day {(dcode >> 11) & 0x1F}")

#print(port_30.write(b''))
#print(port_34.write(b''))
