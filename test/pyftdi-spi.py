from pyftdi import FtdiLogger
import logging

logging.getLogger().setLevel(logging.INFO)
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)
ch.setFormatter(logging.Formatter("[%(asctime)s] %(name)s: %(levelname)s: %(message)s"))
logging.getLogger().addHandler(ch)

FtdiLogger.set_level(logging.DEBUG)
#FtdiLogger.log.addHandler(logging.StreamHandler)

from bitarray import bitarray

import pyftdi.spi

ctrl = pyftdi.spi.SpiController()
ctrl.configure('ftdi://ftdi:2232:123456/1')

spi = ctrl.get_port(cs = 0, freq = 9600, mode = 0)

b = bitarray('1111')

# This is obviously extremely hokey: there are (completely legal, by SPI
# protocol!) bit time gaps between each written 'byte', which do not align
# to UART byte boundaries.
#
# But with a slow enough baud rate, it *does* work sometimes...  which is
# good enough for this.

for bb in b"Hello, world! Hello, world! Hello, world!":
    b += bitarray('0')
    r = bitarray(bytes([bb]))
    r.reverse()
    b += r
    b += bitarray('111')

print(b.tobytes())

spi.write(b.tobytes())
