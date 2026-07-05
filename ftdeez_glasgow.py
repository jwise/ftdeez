import logging
import asyncio

import fakeusb1
import pyusbip
import ftdeez

import argparse

from glasgow.hardware.assembly import HardwareAssembly
from glasgow.applet.interface.uart import UARTInterface

from amaranth import *
from amaranth.lib.wiring import In, Out, flipped
from amaranth.lib import enum, wiring, io, stream
from amaranth.lib.cdc import FFSynchronizer

_logger = logging.getLogger("ftdeez_glasgow")

class UartDataBits(enum.Enum, shape=1):
    Bits7 = 0
    Bits8 = 1

class UartStopBits(enum.Enum, shape=1):
    Bits1 = 0
    Bits2 = 1

class UartParity(enum.Enum, shape=3):
    Off   = 0
    Odd   = 1
    Even  = 2
    Mark  = 3
    Space = 4

class UartRx(wiring.Component):
    rx: In(1) # must be synchronized into cclk!

    rx_stream: Out(stream.Signature(8))
    bit_cyc: In(24)
    data_bits: In(UartDataBits)
    stop_bits: In(UartStopBits) # really, enum: 1 or 2
    parity: In(UartParity) # really, enum: none, odd, even
    
    ferr: Out(1)
    perr: Out(1)
    ovf:  Out(1)
    
    def elaborate(self, platform):
        m = Module()
        
        start      = Signal()
        timer      = Signal(24)
        bit_strobe = Signal()
        shreg      = Signal(8)
        bitno      = Signal(4)
        
        with m.If(start):
            m.d.sync += timer.eq(self.bit_cyc >> 1)
        with m.Elif(timer == 0):
            m.d.sync += timer.eq(self.bit_cyc - 1)
        with m.Else():
            m.d.sync += timer.eq(timer - 1)
        m.d.comb += bit_strobe.eq(timer == 0)
        
        parity_expect = Signal()
        with m.Switch(self.parity):
            with m.Case(UartParity.Odd):
                m.d.comb += parity_expect.eq(~shreg.xor())
            with m.Case(UartParity.Even):
                m.d.comb += parity_expect.eq(shreg.xor())
            with m.Case(UartParity.Mark):
                m.d.comb += parity_expect.eq(1)
            with m.Case(UartParity.Space):
                m.d.comb += parity_expect.eq(0)

        with m.FSM():
            with m.State("IDLE"):
                with m.If(~self.rx):
                    m.d.comb += start.eq(1)
                    m.d.sync += bitno.eq(0)
                    m.next = "START"
            with m.State("START"):
                with m.If(bit_strobe):
                    m.d.sync += shreg.eq(0) # reset to 0, for the case of 7 bit data and still calculating parity
                    m.next = "DATA"
            with m.State("DATA"):
                with m.If(bit_strobe):
                    m.d.sync += [
                        shreg.eq(Cat(shreg[1:], self.rx)),
                        bitno.eq(bitno + 1),
                    ]
                    with m.If(bitno == Mux(self.data_bits == UartDataBits.Bits7, 6, 7)):
                        with m.If(self.parity == UartParity.Off):
                            m.next = "STOP"
                        with m.Else():
                            m.next = "PARITY"
            with m.State("PARITY"):
                with m.If(bit_strobe):
                    with m.If(self.rx == parity_expect):
                        m.next = "STOP"
                    with m.Else():
                        m.d.comb += self.perr.eq(1)
                        m.next = "IDLE"

            with m.State("STOP"):
                # XXX: support two stop bits
                with m.If(bit_strobe):
                    with m.If(~self.rx):
                        m.d.comb += self.ferr.eq(1)
                        m.next = "IDLE"
                    with m.Else():
                        m.d.comb += self.rx_stream.payload.eq(shreg)
                        m.d.comb += self.rx_stream.valid.eq(1)
                        with m.If(self.rx_stream.ready):
                            m.next = "IDLE"
                        with m.Else():
                            m.next = "READY"

            with m.State("READY"):
                m.d.comb += self.rx_stream.payload.eq(shreg)
                m.d.comb += self.rx_stream.valid.eq(1)
                with m.If(self.rx_stream.ready):
                    m.next = "IDLE"
                with m.Elif(~self.rx):
                    m.d.comb += self.ovf.eq(1)
                    m.next = "IDLE"
        
        return m

class UartTx(wiring.Component):
    tx: Out(1)

    tx_stream: In(stream.Signature(8))
    bit_cyc: In(24)
    data_bits: In(UartDataBits)
    stop_bits: In(UartStopBits) # really, enum: 1 or 2
    parity: In(UartParity) # really, enum: none, odd, even

    def elaborate(self, platform):
        m = Module()
        
        start      = Signal()
        timer      = Signal(24)
        bit_strobe = Signal()
        shreg      = Signal(8)
        bitno      = Signal(4)
        
        with m.If(start | (timer == 0)):
            m.d.sync += timer.eq(self.bit_cyc - 1)
        with m.Else():
            m.d.sync += timer.eq(timer - 1)
        m.d.comb += bit_strobe.eq(timer == 0)
        
        parity_gen = Signal()

        with m.FSM():
            with m.State("IDLE"):
                m.d.comb += self.tx_stream.ready.eq(1)
                with m.If(self.tx_stream.valid):
                    m.d.sync += self.tx.eq(0)
                    m.d.sync += bitno.eq(0)
                    m.d.comb += start.eq(1)
                    m.d.sync += shreg.eq(self.tx_stream.payload)
                    with m.Switch(self.parity):
                        with m.Case(UartParity.Odd):
                            m.d.sync += parity_gen.eq(~self.tx_stream.payload.xor())
                        with m.Case(UartParity.Even):
                            m.d.sync += parity_gen.eq(self.tx_stream.payload.xor())
                        with m.Case(UartParity.Mark):
                            m.d.sync += parity_gen.eq(1)
                        with m.Case(UartParity.Space):
                            m.d.sync += parity_gen.eq(0)
                    m.next = "START"
                with m.Else():
                    m.d.sync += self.tx.eq(1)
            with m.State("START"):
                with m.If(bit_strobe):
                    m.d.sync += self.tx.eq(shreg[0])
                    m.d.sync += shreg.eq(Cat(shreg[1:], C(0,1)))
                    m.next = "DATA"
            with m.State("DATA"):
                with m.If(bit_strobe):
                    with m.If(bitno != Mux(self.data_bits == UartDataBits.Bits7, 6, 7)):
                        m.d.sync += [
                            self.tx.eq(shreg[0]),
                            shreg.eq(Cat(shreg[1:], C(0,1))),
                            bitno.eq(bitno + 1),
                        ]
                    with m.Else():
                        with m.If(self.parity == UartParity.Off):
                            m.d.sync += self.tx.eq(1)
                            m.next = "STOP"
                        with m.Else():
                            m.d.sync += self.tx.eq(parity_gen)
                            m.next = "PARITY"
            with m.State("PARITY"):
                with m.If(bit_strobe):
                    m.d.sync += self.tx.eq(1)
                    m.next = "STOP"

            with m.State("STOP"):
                # XXX: support two stop bits
                with m.If(bit_strobe):
                    m.next = "IDLE"
        
        return m



class GlasgowD2xxComponent(wiring.Component):
    rx_stream: Out(stream.Signature(8))
    tx_stream: In(stream.Signature(8))
    bit_cyc: In(24)
    
    def __init__(self, ports):
        self.ports = ports # ideally, this would be a portgroup
        
        super().__init__()
    
    def elaborate(self, platform):
        m = Module()
        
        m.submodules.uart_rx = uart_rx = UartRx()
        wiring.connect(m, uart_rx.rx_stream, flipped(self.rx_stream))
        m.d.comb += uart_rx.bit_cyc.eq(self.bit_cyc)
        m.d.comb += uart_rx.data_bits.eq(UartDataBits.Bits8)
        m.d.comb += uart_rx.stop_bits.eq(UartStopBits.Bits1)
        m.d.comb += uart_rx.parity.eq(UartParity.Off)

        m.submodules.uart_tx = uart_tx = UartTx()
        wiring.connect(m, uart_tx.tx_stream, flipped(self.tx_stream))
        m.d.comb += uart_tx.bit_cyc.eq(self.bit_cyc)
        m.d.comb += uart_tx.data_bits.eq(UartDataBits.Bits8)
        m.d.comb += uart_tx.stop_bits.eq(UartStopBits.Bits1)
        m.d.comb += uart_tx.parity.eq(UartParity.Off)
        
        # XXX LATER: make this be dbus / cbus, and i/o buffers
        m.submodules.rx_buffer = rx_buffer = io.Buffer("i", self.ports.rx)
        m.submodules += FFSynchronizer(rx_buffer.i, uart_rx.rx, init=1)
        m.submodules.tx_buffer = tx_buffer = io.Buffer("o", self.ports.tx)
        m.d.comb += tx_buffer.o.eq(uart_tx.tx)
        
        return m

class GlasgowD2xxChannel(ftdeez.BaseD2xxChannel):
    # supports only UART mode for now, and only barely that
    def __init__(self, assembly, pins):
        super().__init__()
        self._logger = logging.getLogger(f"ftdeez_glasgow.GlasgowD2xxChannel.{id(self)}")
        
        ports = assembly.add_port_group(tx=pins[0], rx=pins[1])
        component = assembly.add_submodule(GlasgowD2xxComponent(ports))
        self._pipe = assembly.add_inout_pipe(component.rx_stream, component.tx_stream)
        self._bit_cyc = assembly.add_rw_register(component.bit_cyc)
        self._sys_clk_period = assembly.sys_clk_period
        
        self._logger = logging.getLogger(f"ftdeez_glasgow.GlasgowD2xxChannel.{id(self)}")
        self.flush_queued = False
    
    async def _set_baud(self, baud):
        cyc = round(1 / (baud * self._sys_clk_period))
        if cyc < 2:
            raise GlasgowAppletError(f"baud rate {baud} is too high")
        await self._bit_cyc.set(cyc)
    
    async def task(self):
        await self._set_baud(115200)
        
        while True:
            if not self._pipe.readable:
                buf = await self._pipe.recv(1)
            else:
                buf = await self._pipe.recv(self._pipe.readable)
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
        asyncio.create_task(self._set_baud(int(baud)))
    
    async def bulk_out(self, buf):
        await self._pipe.send(buf)

        async def do_flush():
            await asyncio.sleep(0.01)
            self.flush_queued = False
            await self._pipe.flush()

        if not self.flush_queued:
            self.flush_queued = True
            asyncio.create_task(do_flush())

        return len(buf)
    

async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--channel', action='append', default=[], required=True, help="add a channel to ftdeez, with pins in a comma-delimited list.  can be specified multiple times to have a ftdeez with multiple channels")
    parser.add_argument('--voltage-a', action='store', type=float, default=3.3)
    parser.add_argument('--voltage-b', action='store', type=float, default=3.3)
    args = parser.parse_args()
    
    assembly = await HardwareAssembly.find_device()
    assembly.use_voltage({"A": args.voltage_a, "B": args.voltage_b})
    
    channels = []
    for channel_pins in args.channel:
        pins = channel_pins.split(',')
        if len(pins) > 16:
            parser.error('channel had too many pins (a real D2xx has 8 pins of DBUS and 8 pins of CBUS)')
        
        for pin in pins:
            if pin not in ['A0', 'A1', 'A2', 'A3', 'A4', 'A5', 'A6', 'A7', 'B0', 'B1', 'B2', 'B3', 'B4', 'B5', 'B6', 'B7', '']:
                parser.error(f"invalid pin assignment {pin}")
        
        pins = [pin if pin != '' else None for pin in pins]
        if len(pins) < 16:
            pins.append([None] * (16 - len(pins)))
        
        channels.append(GlasgowD2xxChannel(assembly, pins))

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
