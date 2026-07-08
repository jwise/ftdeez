import logging
import asyncio

import fakeusb1
import pyusbip
import ftdeez
from mpsse import MPSSE

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
    modem_line_status: Out(16)
    ack_status: In(16)
    modem_ctrl: In(8)
    bit_mode: In(16)
    
    def __init__(self, portgroup, pins):
        self.portgroup = portgroup
        self.pins = pins
        
        super().__init__()
    
    def elaborate(self, platform):
        m = Module()

        # make I/O buffers for each port
        ports_i = Signal(16)
        ports_o = Signal(16)
        ports_oe = Signal(16)
        
        buffers = {}
        for pin,_ in self.pins:
            if not pin or pin in buffers:
                continue
            buffers[pin] = io.Buffer("io", self.portgroup[pin])
            m.submodules += buffers[pin]
            m.d.comb += buffers[pin].oe.eq(0) # will be overridden later
        
        for p in range(16):
            pin, params = self.pins[p]
            if not pin:
                ports_i[p].eq(1)
                continue
            
            is_input_only = False
            has_ff = True
            for param in params:
                match param:
                    case 'i':
                        is_input_only = True
                    case 'noff':
                        has_ff = False 
            
            if not is_input_only:
                m.d.comb += buffers[pin].o.eq(ports_o[p])
                m.d.comb += buffers[pin].oe.eq(ports_oe[p])
            if has_ff: # XXX: maybe the nonsynchronized version should go to MPSSE, synchronized version to UART?
                m.submodules += FFSynchronizer(buffers[pin].i, ports_i[p], init=1)
            else:
                m.d.comb += ports_i[p].eq(buffers[pin].i)

        ### UART SPECIFIC BITS ###
        # UART TODO: support BREAK mode
        # UART TODO: support hardware flow control
        # UART TODO: support xon/xoff flow control
        # UART TODO: support "alert" character
        # UART TODO: inject incorrect framing character on framing error
        # UART TODO: hook up data bits, stop bits, parity
        m.submodules.uart_rx = uart_rx = UartRx()
        m.d.comb += uart_rx.bit_cyc.eq(self.bit_cyc)
        m.d.comb += uart_rx.data_bits.eq(UartDataBits.Bits8)
        m.d.comb += uart_rx.stop_bits.eq(UartStopBits.Bits1)
        m.d.comb += uart_rx.parity.eq(UartParity.Off)

        m.submodules.uart_tx = uart_tx = UartTx()
        m.d.comb += uart_tx.bit_cyc.eq(self.bit_cyc)
        m.d.comb += uart_tx.data_bits.eq(UartDataBits.Bits8)
        m.d.comb += uart_tx.stop_bits.eq(UartStopBits.Bits1)
        m.d.comb += uart_tx.parity.eq(UartParity.Off)

        with m.If(self.bit_mode[8:16] == 0x00): # base mode
            wiring.connect(m, uart_rx.rx_stream, flipped(self.rx_stream))
            wiring.connect(m, uart_tx.tx_stream, flipped(self.tx_stream))

            m.d.comb += ports_oe[0].eq(1)
            m.d.comb += ports_o[0].eq(uart_tx.tx)

            m.d.comb += uart_rx.rx.eq(ports_i[1])

            m.d.comb += ports_oe[2].eq(1) # RTSn
            m.d.comb += ports_o[2].eq(~self.modem_ctrl[1]) # RTSn

            m.d.comb += ports_oe[4].eq(1) # DTRn
            m.d.comb += ports_o[4].eq(~self.modem_ctrl[0]) # DTRn
        
        # XXX LATER: make this be muxed
        ovf = Signal()
        ovf_set = uart_rx.ovf & (self.bit_mode[8:16] == 0x00)
        ovf_clr = self.ack_status[1]
        
        perr = Signal()
        perr_set = uart_rx.perr & (self.bit_mode[8:16] == 0x00)
        perr_clr = self.ack_status[2]
        
        ferr = Signal()
        ferr_set = uart_rx.ferr & (self.bit_mode[8:16] == 0x00)
        ferr_clr = self.ack_status[3]
        
        m.d.sync +=  ovf.eq(( ovf |  ovf_set) & ~ ovf_clr)
        m.d.sync += perr.eq((perr | perr_set) & ~perr_clr)
        m.d.sync += ferr.eq((ferr | ferr_set) & ~ferr_clr)

        ### MPSSE SPECIFIC BITS ###
        m.submodules.mpsse = mpsse = MPSSE()
        with m.If(self.bit_mode[8:16] == 0x02): # MPSSE mode
            wiring.connect(m, mpsse.in_stream, flipped(self.tx_stream))
            wiring.connect(m, mpsse.out_stream, flipped(self.rx_stream))
            
            m.d.comb += ports_oe.eq(mpsse.pads_oe)
            m.d.comb += ports_o.eq(mpsse.pads_o)
            m.d.comb += mpsse.pads_i.eq(ports_i)
        
        # XXX: also FIFO reset opcode from host!
        m.d.comb += mpsse.reset.eq(self.bit_mode[8:16] != 0x02)

        # PORTS
        m.d.comb += self.modem_line_status[8].eq(1)
        m.d.comb += self.modem_line_status[12].eq(~ports_i[3]) # CTSn
        m.d.comb += self.modem_line_status[13].eq(~ports_i[5]) # DSRn
        m.d.comb += self.modem_line_status[14].eq(~ports_i[7]) # RIn
        m.d.comb += self.modem_line_status[15].eq(~ports_i[6]) # DCDn
        
        m.d.comb += self.modem_line_status[0].eq(self.rx_stream.ready == 0) # "data ready"?
        m.d.comb += self.modem_line_status[1].eq(ovf)
        m.d.comb += self.modem_line_status[2].eq(perr)
        m.d.comb += self.modem_line_status[3].eq(ferr)
        m.d.comb += self.modem_line_status[4].eq(0) # XXX: break interrupt
        m.d.comb += self.modem_line_status[5].eq(0) # 'TX holding register'?
        m.d.comb += self.modem_line_status[6].eq(self.tx_stream.valid == 0) # TX FIFO is empty
        m.d.comb += self.modem_line_status[7].eq(0) # 'FIFO error'?
        
        return m

class GlasgowD2xxChannel(ftdeez.BaseD2xxChannel):
    # supports only UART mode for now, and only barely that
    def __init__(self, assembly, pins):
        super().__init__()

        pulls = {}        
        grp = {}
        for n,(pin,params) in enumerate(pins):
            if not pin:
                continue
                
            grp[f"{pin}"] = pin
            for param in params:
                match param:
                    case 'pu':
                        pulls[pin] = 'high'
                    case 'pd':
                        pulls[pin] = 'low'

        assembly.use_pulls(pulls)
        portgroup = assembly.add_port_group(**grp)
        
        component = assembly.add_submodule(GlasgowD2xxComponent(portgroup, pins))

        self._pipe = assembly.add_inout_pipe(component.rx_stream, component.tx_stream)
        self._bit_cyc = assembly.add_rw_register(component.bit_cyc)
        self._modem_line_status = assembly.add_ro_register(component.modem_line_status)
        self._ack_status = assembly.add_rw_register(component.ack_status)
        self._modem_ctrl = assembly.add_rw_register(component.modem_ctrl)
        self._bit_mode = assembly.add_rw_register(component.bit_mode)
        
        self._sys_clk_period = assembly.sys_clk_period
        print(f"sys_clk_period = {assembly.sys_clk_period}")
        
        self._logger = logging.getLogger(f"ftdeez_glasgow.GlasgowD2xxChannel.{id(self)}")
        self.flush_queued = False
    
    async def get_modem_status(self):
        modem_status = await self._modem_line_status.get()
        if len(self._in_buf) > 0:
            modem_status |= 0x0001

        # XXX HACK: we can lose stuff here, this needs to be a one-shot strobe...
        if modem_status & 0xE:
            await self._ack_status.set(modem_status)
            await self._ack_status.set(0)

        return modem_status
    
    async def set_modem_ctrl(self, wvalue):
        modem_ctrl = await self._modem_ctrl.get()
        if wvalue & 0x0100:
            modem_ctrl &= ~0x01
            modem_ctrl |= wvalue & 0x01
        if wvalue & 0x0200:
            modem_ctrl &= ~0x02
            modem_ctrl |= wvalue & 0x02
        
        await self._modem_ctrl.set(modem_ctrl)
    
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
    
    async def set_bitmode(self, wValue):
        self._logger.info('setting bit mode up')
        await self._bit_mode.set(wValue)
    
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
        
        out_pins = []
        for pin in pins:
            if pin == '':
                out_pins.append((None,[]))
                continue
            
            params = []
            if '=' in pin:
                pin,params = pin.split('=')
                params = params.split('+')
            
            if pin not in ['A0', 'A1', 'A2', 'A3', 'A4', 'A5', 'A6', 'A7', 'B0', 'B1', 'B2', 'B3', 'B4', 'B5', 'B6', 'B7', '']:
                parser.error(f"invalid pin assignment {pin}")

            for param in params:
                match param:
                    case 'i':
                        pass
                    case 'noff':
                        pass
                    case 'pu':
                        pass
                    case 'pd':
                        pass
                    case _:
                        parser.error(f"unknown pin parameter {param} on pin {pin}")

            out_pins.append((pin, params))
        
        if len(out_pins) < 16:
            out_pins += [(None,[])] * (16 - len(out_pins))
        
        channels.append(GlasgowD2xxChannel(assembly, out_pins))

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
