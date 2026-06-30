import logging
import asyncio

import fakeusb1
import pyusbip
import ftdeez

from glasgow.hardware.assembly import HardwareAssembly
from glasgow.applet.interface.uart import UARTInterface

_logger = logging.getLogger("ftdeez_glasgow")

class GlasgowD2xxChannel(ftdeez.BaseD2xxChannel):
    # supports only UART mode for now, and only barely that
    def __init__(self, assembly, tx, rx):
        super().__init__()
        self._logger = logging.getLogger(f"ftdeez_glasgow.GlasgowD2xxChannel.{id(self)}")
        self.interface = UARTInterface(
            logging.getLogger(f"ftdeez_glasgow.GlasgowD2xxChannel.{id(self)}.UARTInterface"),
            assembly, tx=tx, rx=rx)
        self._logger = logging.getLogger(f"ftdeez_glasgow.GlasgowD2xxChannel.{id(self)}")
        self.flush_queued = False
    
    async def task(self):
        await self.interface.set_baud(115200)
        
        while True:
            buf = await self.interface.read_all()
            await self.put_infifo(buf)
            # handle latency timer flush character!
        
        self.put_infifo(buf)
    
    def set_baud_rate(self, divisor):
        DIVISOR_FRAC_LUT = {
            0: 0.0,
            1: 0.5,
            2: 0.25,
            3: 0.125,
            4: 0.375,
            5: 0.675,
            6: 0.75,
            7: 0.875
        }
        divisor_mode = (divisor >> 17) & 1
        divisor_int = (divisor & 0x3FFF)
        divisor_frac = (divisor >> 14) & 7
        
        divisor_calc = divisor_int + DIVISOR_FRAC_LUT[divisor_frac]
        if divisor_mode:
            baud = 120000000 / divisor_calc / 10
        else:
            baud = 48000000 / divisor_calc / 16

        self._logger.info(f"setting baud to {int(baud)}")
        asyncio.create_task(self.interface.set_baud(int(baud)))
    
    async def bulk_out(self, buf):
        await self.interface.write(buf)

        async def do_flush():
            await asyncio.sleep(0.01)
            self.flush_queued = False
            await self.interface.flush()

        if not self.flush_queued:
            self.flush_queued = True
            asyncio.create_task(do_flush())

        return len(buf)
    

async def main():
    assembly = await HardwareAssembly.find_device()
    assembly.use_voltage({"A": 3.3, "B": 3.3})
    
    channels = [
        GlasgowD2xxChannel(assembly, "A0", "A1"),
        GlasgowD2xxChannel(assembly, "B0", "B1"),
    ]

    dev = ftdeez.Ft2232Device(channels=channels)
    usbctx = fakeusb1.FakeUSBContext(devices=[dev])
    
    async with assembly:
        for c in channels:
            asyncio.create_task(c.task())
            
        server = await pyusbip.serve_context(usbctx, host='0.0.0.0')
        _logger.info('Serving on {}'.format(server.sockets[0].getsockname()))
        
        await server.serve_forever()

if __name__ == "__main__":
    ftdeez.setup_logging()
    asyncio.run(main())
    asyncio.get_event_loop().run_forever()
