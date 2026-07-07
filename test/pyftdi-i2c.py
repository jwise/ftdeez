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
port = i2c.get_port(0x30)
port.write(b'')
port.write(b'')
