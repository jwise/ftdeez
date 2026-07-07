# MPSSE reference:
# http://www.ftdichip.com/Support/Documents/AppNotes/AN_135_MPSSE_Basics.pdf
# http://www.ftdichip.com/Support/Documents/AppNotes/ AN_108_Command_Processor_for_MPSSE_and_MCU_Host_Bus_Emulation_Modes.pdf
#
# originally by Catherine <whitequark@whitequark.org> in 2018 or earlier,
# removed from Glasgow in 2023, mercilessly mutilated by Joshua to work with
# a more modern Amaranth

from contextlib import contextmanager

from amaranth import *
from amaranth.lib.wiring import In, Out, flipped
from amaranth.lib import enum, wiring, io, stream, data
from amaranth.lib.cdc import FFSynchronizer


__all__ = ['MPSSE']

class MPSSEClockGen(wiring.Component):
    clken: In(1)
    clkpos: Out(1)
    clkneg: Out(1)
    
    tcken: In(1)
    tckpos: Out(1)
    tckneg: Out(1)
    
    tck: Out(1)
    
    divisor: In(16)
    legacy_divisor_en: In(1) 
    
    tck_override: In(1)
    tck_override_value: In(1)
    
    SYSCLK_MULTIPLIER = 4

    # XXX: in MPSSEClockGen, this appears to be 'divisor + 5'.  but
    # according to https://github.com/vjardin/ftdi/blob/main/mpsse.md , 0x8B
    # ("legacy_divisor_en == 1") should result in 12MHz base clock
    # (divide-by-5), and 0x8A should result in 60MHz base clock.
    #
    # also, for that matter, this demands a 12MHz base clock.  according ot
    # that doc, the TCK frequency should be base / ((1 + Divisor) * 2).  so
    # a divisor of 0x0000 in 60MHz mode should result in 30MHz TCK; a
    # divisor of 0x0000 in 12MHz mode shoudl result in 6MHz TCK.
    #
    # this divisor does not at all take into account sysclk!  on Glasgow
    # right now, sysclk is 48MHz.
    
    def elaborate(self, platform):
        m = Module()
        
        clk2x   = Signal(init=1)
        counter = Signal(24)
        
        divisor_computed = Signal(24)
        m.d.sync += divisor_computed.eq(
            Mux(self.legacy_divisor_en,
                (self.divisor + 1) * self.SYSCLK_MULTIPLIER - 1,
                self.divisor # should be * SYSCLK_MULTIPLIER / 5!  but *you* divide by 5 in digital logic.
            ))
        
        with m.If(self.clken):
            with m.If(counter == 0):
                m.d.sync += counter.eq(divisor_computed)
                m.d.sync += clk2x.eq(~clk2x)
            with m.Else():
                m.d.sync += counter.eq(counter - 1)
        with m.Else():
            m.d.sync += counter.eq(divisor_computed)
            m.d.sync += clk2x.eq(clk2x.init)
        # XXX: I think this causes CLK2X to actually be at 1x of the
        # programmed divisor time?

        # XXX: I do not understand the relationship between CLKPOS, CLKNEG
        # and TCKPOS, TCKNEG.  it seems like they are the same except when
        # they get out of sync because TCKEN is disabled.  why does anyone care about CLKPOS or CLKEN then?
        
        clkreg = Signal()
        m.d.sync += clkreg.eq(self.clken & clk2x)
        m.d.comb += [
            self.clkpos.eq(self.clken & (~clkreg &  clk2x)),
            self.clkneg.eq(self.clken & ( clkreg & ~clk2x))
        ]

        tckstb = Signal()
        with m.If(self.tck_override):
            m.d.sync += self.tck.eq(self.tck_override_value)
        with m.Else():
            m.d.sync += self.tck.eq(self.tck ^ tckstb)
        m.d.comb += [
            tckstb.eq((self.clkpos | self.clkneg) & self.tcken),
            self.tckpos.eq(tckstb & ~self.tck),
            self.tckneg.eq(tckstb &  self.tck)
        ]
        
        return m


class MPSSE(wiring.Component):
    in_stream:  In(stream.Signature(8))
    out_stream: Out(stream.Signature(8))
    
    pads_i: In(16)
    pads_o: Out(16)
    pads_oe: Out(16)
    
    def elaborate(self, platform):
        m = Module()
        
        divisor = Signal(16, init=0)
        legacy_divisor_en = Signal(init=1)

        position = Signal(data.FlexibleLayout(19, {
            "bit":    data.Field(unsigned(3), 0),
            "lobyte": data.Field(unsigned(8), 3),
            "hibyte": data.Field(unsigned(8), 11),
        }))
        
        # expose for TB
        self.divisor = divisor
        self.position = position
        self.legacy_divisor_en = legacy_divisor_en

        # Clock generator

        m.submodules.clkgen = clkgen = MPSSEClockGen()
        m.d.comb += clkgen.divisor.eq(divisor)
        m.d.comb += clkgen.legacy_divisor_en.eq(legacy_divisor_en)

        pad_o_tck = self.pads_o[0]
        pad_o_tdi = self.pads_o[1]
        pad_o_tms = self.pads_o[3]
        
        loopback = Signal()
        pad_i_tdo = Mux(loopback, pad_o_tdi, self.pads_i[2])

        m.d.comb += pad_o_tck.eq(clkgen.tck)

        # Command decoder

        pend_cmd   = Signal(8)
        curr_cmd   = Signal(8)

        is_shift   = Signal()
        shift_cmd = Signal(data.FlexibleLayout(7, {
            "wneg": data.Field(unsigned(1), 0),
            "bits": data.Field(unsigned(1), 1),
            "rneg": data.Field(unsigned(1), 2),
            "le"  : data.Field(unsigned(1), 3),
            "tdi" : data.Field(unsigned(1), 4),
            "tdo" : data.Field(unsigned(1), 5),
            "tms" : data.Field(unsigned(1), 6),
        }))
        
        with m.If(curr_cmd[7:] == 0b0):
            m.d.comb += is_shift.eq(1)
            m.d.comb += shift_cmd.eq(curr_cmd[:7])
        with m.Elif(curr_cmd == 0x8E):
            m.d.comb += shift_cmd.eq(0x02)
        with m.Elif(curr_cmd == 0x8F):
            m.d.comb += shift_cmd.eq(0x00)

        is_gpio      = curr_cmd[2:] == 0b100000
        gpio_cmd_rd  = curr_cmd[0]
        gpio_cmd_adr = curr_cmd[1]

        # Command processor
        
        @contextmanager
        def _consume_input():
            m.d.comb += self.in_stream.ready.eq(1)
            with m.If(self.in_stream.valid):
                yield

        @contextmanager
        def _produce_output(out):
            m.d.comb += self.out_stream.valid.eq(1)
            m.d.comb += self.out_stream.payload.eq(out)
            with m.If(self.out_stream.ready):
                yield

        with m.FSM() as fsm:
            self.fsm = fsm
            m.d.comb += curr_cmd.eq(pend_cmd) # overridden in IDLE only
            
            with m.State("IDLE"), _consume_input():
                m.d.comb += curr_cmd.eq(self.in_stream.payload)
                m.d.sync += pend_cmd.eq(self.in_stream.payload)
                
                with m.If(is_shift):
                    m.d.sync += position.eq(0)
                    with m.If(shift_cmd.tms):
                        with m.If(shift_cmd.bits & shift_cmd.le & ~shift_cmd.tdi):
                            m.next = "SHIFT-LENGTH-BITS"
                        with m.Else():
                            m.next = "ERROR"
                    with m.Elif(shift_cmd.tdi | shift_cmd.tdo):
                        with m.If((shift_cmd.wneg & ~shift_cmd.tdi) |
                                  (shift_cmd.rneg & ~shift_cmd.tdo)):
                            m.next = "ERROR"
                        with m.Elif(shift_cmd.bits):
                            m.next = "SHIFT-LENGTH-BITS"
                        with m.Else():
                            m.next = "SHIFT-LENGTH-LOBYTE"
                    with m.Else():
                        m.next = "ERROR"
                with m.Elif(is_gpio):
                    with m.If(gpio_cmd_rd):
                        m.next = "GPIO-READ-I"
                    with m.Else():
                        m.next = "GPIO-WRITE-O"
                with m.Elif(curr_cmd == 0x8E):
                    m.next = "SHIFT-LENGTH-BITS"
                with m.Elif(curr_cmd == 0x8F):
                    m.next = "SHIFT-LENGTH-LOBYTE"
                with m.Elif(curr_cmd == 0x86):
                    m.next = "DIVISOR-LOBYTE"
                with m.Elif(curr_cmd == 0x84):
                    m.d.sync += loopback.eq(1)
                with m.Elif(curr_cmd == 0x85):
                    m.d.sync += loopback.eq(0)
                with m.Elif(curr_cmd == 0x8A):
                    m.d.sync += legacy_divisor_en.eq(0)
                with m.Elif(curr_cmd == 0x8B):
                    m.d.sync += legacy_divisor_en.eq(1)
                with m.Else():
                    m.next = "ERROR"
        

            # Shift subcommand, length handling

            def begin_shifting():
                with m.If(shift_cmd.tdi ^ shift_cmd.tms):
                    m.next = "SHIFT-LOAD"
                with m.Else():
                    m.next = "SHIFT-SETUP"

            with m.State("SHIFT-LENGTH-LOBYTE"), _consume_input():
                m.d.sync += position.bit.eq(7),
                m.d.sync += position.lobyte.eq(self.in_stream.payload)
                m.next = "SHIFT-LENGTH-HIBYTE"

            with m.State("SHIFT-LENGTH-HIBYTE"), _consume_input():
                m.d.sync += Cat(position.lobyte, position.hibyte).eq(Cat(position.lobyte, self.in_stream.payload) - 1)
                begin_shifting()

            with m.State("SHIFT-LENGTH-BITS"), _consume_input():
                m.d.sync += position.bit.eq(self.in_stream.payload)
                begin_shifting()

            # Shift subcommand, actual shifting

            rx_data_be = Mux(~shift_cmd.le, self.in_stream.payload, Cat([self.in_stream.payload[7 - i] for i in range(8)]))

            bits_in = Signal(8)
            bits_out = Signal(8)

            output_setup = Signal()
            output_hold  = Signal()
            input_setup = Signal()
            input_hold  = Signal()
            m.d.comb += [
                output_setup.eq(clkgen.tckpos & ~shift_cmd.wneg |
                             clkgen.tckneg &  shift_cmd.wneg),
                output_hold .eq(clkgen.tckpos &  shift_cmd.wneg |
                             clkgen.tckneg & ~shift_cmd.wneg),
                input_setup.eq(clkgen.tckpos & ~shift_cmd.rneg |
                             clkgen.tckneg &  shift_cmd.rneg),
                input_hold .eq(clkgen.tckpos &  shift_cmd.rneg |
                             clkgen.tckneg & ~shift_cmd.rneg),
            ]

            with m.State("SHIFT-LOAD"), _consume_input():
                m.d.sync += bits_in.eq(rx_data_be << 1),
                with m.If(shift_cmd.tdi):
                    m.d.sync += pad_o_tdi.eq(rx_data_be[7])
                with m.Else():
                    m.d.sync += pad_o_tms.eq(rx_data_be[7])
                m.next = "SHIFT-SETUP"

            with m.State("SHIFT-SETUP"):
                m.d.comb += clkgen.clken.eq(1)
                m.d.comb += clkgen.tcken.eq(clkgen.clkneg)
                with m.If(clkgen.clkneg):
                    m.next = "SHIFT-CLOCK"

            with m.State("SHIFT-CLOCK"):
                m.d.comb += clkgen.clken.eq(1)
                m.d.comb += clkgen.tcken.eq(1)
                with m.If(output_setup):
                    m.d.sync += bits_in.eq(bits_in << 1)
                    with m.If(shift_cmd.tdi):
                        m.d.sync += pad_o_tdi.eq(bits_in[7])
                    with m.Elif(shift_cmd.tms):
                        m.d.sync += pad_o_tms.eq(bits_in[7])
                with m.If(input_setup):
                    m.d.sync += bits_out.eq(bits_out << 1),
                    with m.If(shift_cmd.tdo):
                        m.d.sync += bits_out[0].eq(pad_i_tdo)
                with m.If(clkgen.clkpos):
                    m.d.sync += position.as_value().eq(position.as_value() - 1)
                    with m.If(position.bit == 0):
                        with m.If(shift_cmd.tdo):
                            m.next = "SHIFT-REPORT"
                        with m.Elif((position.lobyte != 0x0) | (position.hibyte != 0x0)):
                            begin_shifting()
                        with m.Else():
                            m.next = "IDLE"

            # XXX: it woudl be nice to also be able to grab the input data on the next clock
            with m.State("SHIFT-REPORT"):
                with _produce_output(bits_out):
                    with m.If((position.lobyte != 0xFF) & (position.hibyte != 0xFF)):
                        begin_shifting()
                    with m.Else():
                        m.next = "IDLE"

            # GPIO commands

            with m.State("GPIO-READ-I"):
                with _produce_output(Mux(gpio_cmd_adr == 0, self.pads_i[0:8], self.pads_i[8:16])):
                    m.next = "IDLE"

            with m.State("GPIO-WRITE-O"), _consume_input():
                with m.If(gpio_cmd_adr == 0):
                    m.d.comb += clkgen.tck_override.eq(1)
                    m.d.comb += clkgen.tck_override_value.eq(self.in_stream.payload[0])
                    m.d.sync += self.pads_o[1:8].eq(self.in_stream.payload[1:8])
                with m.Else():
                    m.d.sync += self.pads_o[8:16].eq(self.in_stream.payload)
                m.next = "GPIO-WRITE-OE"

            with m.State("GPIO-WRITE-OE"), _consume_input():
                with m.If(gpio_cmd_adr == 0):
                    m.d.sync += self.pads_oe[0:8].eq(self.in_stream.payload)
                with m.Else():
                    m.d.sync += self.pads_oe[8:16].eq(self.in_stream.payload)
                m.next = "IDLE"

            # Divisor subcommand

            with m.State("DIVISOR-LOBYTE"), _consume_input():
                m.d.sync += divisor[0:8].eq(self.in_stream.payload)
                m.next = "DIVISOR-HIBYTE"

            with m.State("DIVISOR-HIBYTE"), _consume_input():
                m.d.sync += divisor[8:16].eq(self.in_stream.payload),
                m.next = "IDLE"

            # Error "subcommand"
            with m.State("ERROR"):
                with _produce_output(0xFA):
                    m.next = "ERROR-DESC"

            with m.State("ERROR-DESC"):
                with _produce_output(pend_cmd):
                    m.next = "IDLE"
        
        return m

# -------------------------------------------------------------------------------------------------

import unittest

from glasgow.gateware import simulation_test
from amaranth.sim import Tick, Simulator

# XXX: this has not been updated, obviously
class MPSSETestbench(Elaboratable):
    def __init__(self):
        self.dut = MPSSE()
        self.tck = self.dut.pads_o[0]
        self.tdi = self.dut.pads_o[1]
        self.tdo = self.dut.pads_i[2]
        self.tms = self.dut.pads_o[3]
        
    def elaborate(self, platform):
        m = Module()
        
        m.submodules.dut = self.dut

        self.clkdiv = 5
        
        return m

    def do_finalize(self):
        self.states = {v: k for k, v in self.dut.fsm.encoding.items()}

    async def dut_state(self, ctx):
        state = {v: k for k, v in self.dut.fsm.encoding.items()}
        return state[ctx.get(self.dut.fsm.state)]

    async def write(self, ctx, byte):
        ctx.set(self.dut.in_stream.payload, byte)
        ctx.set(self.dut.in_stream.valid, 1)
        for _ in range(32 * self.clkdiv):
            if ctx.get(self.dut.in_stream.ready) == 1:
                await ctx.tick()
                ctx.set(self.dut.in_stream.valid, 0)
                return
            await ctx.tick()
        raise Exception("DUT stuck while writing")

    async def read(self, ctx):
        ctx.set(self.dut.out_stream.ready, 1)
        for _ in range(32 * self.clkdiv):
            if ctx.get(self.dut.out_stream.valid) == 1:
                byte = ctx.get(self.dut.out_stream.payload)
                await ctx.tick()
                ctx.set(self.dut.out_stream.ready, 0)
                return byte
            await ctx.tick()
        raise Exception("DUT stuck while reading")

    async def _wait_for_tck(self, ctx, at_setup=None):
        setup = None
        for _ in range(64 * self.clkdiv):
            if at_setup:
                setup = at_setup()
            tckold = ctx.get(self.tck)
            await ctx.tick()
            tcknew = ctx.get(self.tck)
            if tckold != tcknew:
                break
        if tckold == tcknew:
            raise Exception("DUT ceased driving TCK")
        return setup

    async def recv_tdi(self, ctx, nbits, pos):
        bits = 0
        for n in range(nbits * 2):
            await self._wait_for_tck(ctx)
            if ctx.get(self.tck) == pos:
                bits = (bits << 1) | ctx.get(self.tdi)
        return bits

    async def recv_tms(self, ctx, nbits, pos):
        bits = 0
        for n in range(nbits * 2):
            await self._wait_for_tck(ctx)
            if ctx.get(self.tck) == pos:
                bits = (bits << 1) | ctx.get(self.tms)
        return bits

    async def xfer(self, ctx, nbits, out_bits, in_bits, out_pos, in_pos):
        for n in range(nbits * 2):
            tdiold = await self._wait_for_tck(ctx, at_setup=lambda: ctx.get(self.tdi))
            tcknew = ctx.get(self.tck)
            
            print(f'tck {tcknew}, tdiold {tdiold}, tdi {ctx.get(self.tdi)}')

            if in_pos == tcknew:
                if ctx.get(self.tdi) != tdiold:
                    await ctx.tick().repeat(4)
                    raise Exception("DUT violated setup/hold timings")

                in_bit  = in_bits  & (1 << (nbits - n // 2) - 1)
                print(f"{in_bits:0{nbits}b} {in_bit:0{nbits}b} ")
                if ctx.get(self.tdi) != (1 if in_bit else 0):
                    badbit = ctx.get(self.tdi)
                    await ctx.tick().repeat(8)
                    raise Exception("DUT clocked out bit {} as {} (expected {})"
                                    .format(n // 2, badbit, 1 if in_bit else 0))
            if out_pos == tcknew:
                out_bit = out_bits & (1 << (nbits - n // 2) - 1)
                ctx.set(self.tdo, out_bit)

        for _ in range(16 * self.clkdiv):
            tckold = ctx.get(self.tck)
            await ctx.tick()
            tcknew = ctx.get(self.tck)
            if tckold != tcknew and in_pos == tcknew:
                raise Exception("DUT spuriously drives TCK")

        return True

    async def out_xfer(self, ctx, nbits, bits, pos):
        return await self.xfer(ctx, nbits, bits, 0, pos, False)

    async def in_xfer(self, ctx, nbits, bits, pos):
        return await self.xfer(ctx, nbits, 0, bits, False, pos)

import functools
def simulation_test_v2(case=None, **kwargs):
    def configure_wrapper(case):
        @functools.wraps(case)
        def wrapper(self):
            async def setup_wrapper(ctx):
                if hasattr(self, "simulationSetUp"):
                    await self.simulationSetUp(ctx, self.tb)
                if hasattr(self, "configure"):
                    await self.configure(ctx, self.tb, **kwargs)
                await case(self, ctx, self.tb)
            if isinstance(self.tb, Elaboratable):
                sim = Simulator(self.tb)
                with sim.write_vcd("test.vcd"):
                    sim.add_clock(1e-8)
                    sim.add_testbench(setup_wrapper)
                    sim.run()
        return wrapper
        
    if case is None:
        return configure_wrapper
    else:
        return configure_wrapper(case)

class MPSSETestCase(unittest.TestCase):
    def setUp(self):
        self.tb = MPSSETestbench()

    async def configure(self, ctx, tb):
        # speed up tests
        ctx.set(tb.dut.legacy_divisor_en, 0)

    @simulation_test_v2
    async def test_error(self, ctx, tb):
        await tb.write(ctx, 0xFF)
        self.assertEqual(await tb.read(ctx), 0xFA)
        self.assertEqual(await tb.read(ctx), 0xFF)
        self.assertEqual(await tb.dut_state(ctx), "IDLE")

    @simulation_test_v2
    async def test_gpio_read(self, ctx, tb):
        ctx.set(tb.dut.pads_i, 0xAA55)
        await ctx.tick()
        await tb.write(ctx, 0x81)
        self.assertEqual(await tb.read(ctx), 0x55)
        await tb.write(ctx, 0x83)
        self.assertEqual(await tb.read(ctx), 0xAA)
        self.assertEqual(await tb.dut_state(ctx), "IDLE")

    @simulation_test_v2
    async def test_gpio_write(self, ctx, tb):
        await tb.write(ctx, 0x80)
        await tb.write(ctx, 0xA1)
        await tb.write(ctx, 0x52)
        self.assertEqual(ctx.get(tb.dut.pads_o),  0x00A1)
        self.assertEqual(ctx.get(tb.dut.pads_oe), 0x0052)
        await tb.write(ctx, 0x82)
        await tb.write(ctx, 0x7E)
        await tb.write(ctx, 0x81)
        self.assertEqual(ctx.get(tb.dut.pads_o),  0x7EA1)
        self.assertEqual(ctx.get(tb.dut.pads_oe), 0x8152)
        self.assertEqual(await tb.dut_state(ctx), "IDLE")

    @simulation_test_v2
    async def test_bits_write(self, ctx, tb):
        await tb.write(ctx, 0x12)
        await tb.write(ctx, 5)
        self.assertEqual(ctx.get(tb.dut.position.bit), 5)
        await tb.write(ctx, 0x55)
        self.assertEqual(await tb.recv_tdi(ctx, 5, pos=True), 0x0A)

    @simulation_test_v2
    async def test_bits_write_div4(self, ctx, tb):
        await tb.write(ctx, 0x86) # set clock divisor
        await tb.write(ctx, 0x03) # divL = 0x03
        await tb.write(ctx, 0x00) # divH = 0x00
        await tb.write(ctx, 0x12) # MPSSE_DO_WRITE | MPSSE_BITMODE
        await tb.write(ctx, 5)
        self.assertEqual(ctx.get(tb.dut.position.bit), 5)
        await tb.write(ctx, 0x55)
        self.assertEqual(await tb.recv_tdi(ctx, 5, pos=True), 0x0A)

    @simulation_test_v2
    async def test_bytes_write_div2(self, ctx, tb):
        await tb.write(ctx, 0x86) # set clock divisor
        await tb.write(ctx, 0x01) # divL = 0x01
        await tb.write(ctx, 0x00) # divH = 0x00
        await tb.write(ctx, 0x10) # MPSSE_DO_WRITE
        await tb.write(ctx, 3)
        await tb.write(ctx, 0)
        self.assertEqual(ctx.get(tb.dut.position.lobyte), 2)
        self.assertEqual(ctx.get(tb.dut.position.hibyte), 0)
        self.assertEqual(ctx.get(tb.dut.position.bit), 7)
        await tb.write(ctx, 0xA0)
        self.assertEqual(await tb.recv_tdi(ctx, 8, pos=True), 0xA0)
        await tb.write(ctx, 0x50)
        self.assertEqual(await tb.recv_tdi(ctx, 8, pos=True), 0x50)
        await tb.write(ctx, 0x0F)
        self.assertEqual(await tb.recv_tdi(ctx, 8, pos=True), 0x0F)

    @simulation_test_v2
    async def test_clk_bits(self, ctx, tb):
        await tb.write(ctx, 0x8E)
        await tb.write(ctx, 5)
        self.assertEqual(ctx.get(tb.dut.position.bit), 5)
        self.assertEqual(await tb.recv_tdi(ctx, 6, pos=True), 0x00)
        self.assertEqual(ctx.get(tb.tck), 0)
        await ctx.tick()
        self.assertEqual(await tb.dut_state(ctx), "IDLE")

    @simulation_test_v2
    async def test_clk_bytes(self, ctx, tb):
        await tb.write(ctx, 0x8F)
        await tb.write(ctx, 5)
        await tb.write(ctx, 0)
        self.assertEqual(ctx.get(tb.dut.position.lobyte), 4)
        self.assertEqual(ctx.get(tb.dut.position.hibyte), 0)
        self.assertEqual(ctx.get(tb.dut.position.bit), 7)
        self.assertEqual(await tb.recv_tdi(ctx, 40, pos=True), 0x00)
        self.assertEqual(ctx.get(tb.tck), 0)
        self.assertEqual(await tb.dut_state(ctx), "IDLE")

    @simulation_test_v2
    async def test_bits_read(self, ctx, tb):
        await tb.write(ctx, 0x22)
        await tb.write(ctx, 5)
        self.assertEqual(ctx.get(tb.dut.position.bit), 5)
        self.assertEqual(await tb.read(ctx), 0x00)

    @simulation_test_v2
    async def test_legacy_divisor(self, ctx, tb):
        # restore pristine MPSSE state
        ctx.set(tb.dut.legacy_divisor_en, 0)
        self.tb.clkdiv = 5

        # works
        await tb.write(ctx, 0x22)
        await tb.write(ctx, 5)
        self.assertEqual(ctx.get(tb.dut.position.bit), 5)
        self.assertEqual(await tb.read(ctx), 0x00)

        # works
        await tb.write(ctx, 0x8A)
        self.tb.clkdiv = 1
        await tb.write(ctx, 0x22)
        await tb.write(ctx, 5)
        self.assertEqual(ctx.get(tb.dut.position.bit), 5)
        self.assertEqual(await tb.read(ctx), 0x00)

        # fails - timeout
        await tb.write(ctx, 0x8B)
        self.tb.clkdiv = 1
        await tb.write(ctx, 0x22)
        await tb.write(ctx, 5)
        self.assertEqual(ctx.get(tb.dut.position.bit), 5)
        with self.assertRaises(Exception):
            await tb.read(ctx)

    @simulation_test_v2
    async def test_bits_read_write(self, ctx, tb):
        await tb.write(ctx, 0x84)
        await tb.write(ctx, 0x33)
        await tb.write(ctx, 5)
        self.assertEqual(ctx.get(tb.dut.position.bit), 5)
        await tb.write(ctx, 0x55)
        self.assertEqual(await tb.recv_tdi(ctx, 5, pos=True), 0x0A)
        self.assertEqual(await tb.read(ctx), 0x15) # non-negative read clock

    @simulation_test_v2
    async def test_tms_write(self, ctx, tb):
        await tb.write(ctx, 0x4A)
        await tb.write(ctx, 5)
        self.assertEqual(ctx.get(tb.dut.position.bit), 5)
        await tb.write(ctx, 0x55)
        self.assertEqual(await tb.recv_tms(ctx, 5, pos=True), 0x15)

    @simulation_test_v2
    async def test_invalid_tms_commands(self, ctx, tb):
        await tb.write(ctx, 0x5A)
        self.assertEqual(await tb.dut_state(ctx), "ERROR")
        await tb.read(ctx)
        await tb.read(ctx)
        self.assertEqual(await tb.dut_state(ctx), "IDLE")
        await tb.write(ctx, 0x42)
        self.assertEqual(await tb.dut_state(ctx), "ERROR")
        await tb.read(ctx)
        await tb.read(ctx)
        await tb.write(ctx, 0x68)
        self.assertEqual(await tb.dut_state(ctx), "ERROR")
        await tb.read(ctx)
        await tb.read(ctx)

    @simulation_test_v2
    async def test_hibyte_lobyte_write(self, ctx, tb):
        await tb.write(ctx, 0x10)
        await tb.write(ctx, 0x05)
        self.assertEqual(ctx.get(tb.dut.position.lobyte), 0x05)
        await tb.write(ctx, 0x11)
        self.assertEqual(ctx.get(tb.dut.position.hibyte), 0x11)

    @simulation_test_v2
    async def test_divisor_write(self, ctx, tb):
        await tb.write(ctx, 0x86)
        await tb.write(ctx, 0x34)
        await tb.write(ctx, 0x12)
        self.assertEqual(ctx.get(tb.dut.divisor), 0x1234)
        self.assertEqual(await tb.dut_state(ctx), "IDLE")

    async def write_single_byte(self, ctx, tb, pos):
        await tb.write(ctx, 0x80)
        if pos:
            await tb.write(ctx, 0b0001)
        else:
            await tb.write(ctx, 0b0000)
        await tb.write(ctx, 0b1101)

        if pos:
            await tb.write(ctx, 0x10)
        else:
            await tb.write(ctx, 0x11)
        await tb.write(ctx, 0x00)
        await tb.write(ctx, 0x00)
        await tb.write(ctx, 0xA5)
        self.assertTrue(await tb.in_xfer(ctx, 8, 0b10100101, not pos))

    @simulation_test_v2
    async def test_write_single_byte_clkpos(self, ctx, tb):
        ctx.set(tb.dut.divisor, 1)
        await self.write_single_byte(ctx, tb, pos=True)

    @simulation_test_v2
    async def test_write_single_byte_clkneg(self, ctx, tb):
        ctx.set(tb.dut.divisor, 1)
        await self.write_single_byte(ctx, tb, pos=False)

    @simulation_test_v2
    async def test_write_single_byte_clkneg_fast(self, ctx, tb):
        await self.write_single_byte(ctx, tb, pos=False)

    @simulation_test_v2
    async def test_write_single_byte_clkwrong(self, ctx, tb):
        await tb.write(ctx, 0x10) # +ve, but we start from tck=0
        await tb.write(ctx, 0x00)
        await tb.write(ctx, 0x00)
        await tb.write(ctx, 0xA5)
        self.assertEqual(await tb.recv_tdi(ctx, 8, pos=False), 0xA5)
        await ctx.tick()
        self.assertEqual(ctx.get(tb.tck), 0)
