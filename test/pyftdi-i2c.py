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
port_30 = i2c.get_port(0x30)
port_34 = i2c.get_port(0x34)
print(port_30.write(b'abcd'))
print(port_34.write(b'abcd'))
