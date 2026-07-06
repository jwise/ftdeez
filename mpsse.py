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

        # XXX: how does real FTDI deal with a GPIO set request for TCK?
        pad_o_tck = self.pads_o[0]
        pad_o_tdi = self.pads_o[1]
        pad_i_tdo = self.pads_i[2]
        pad_o_tms = self.pads_o[3]
        
        loopback = Signal()

        m.submodules.clkgen = clkgen = MPSSEClockGen()
        m.d.comb += clkgen.divisor.eq(divisor)
        m.d.comb += clkgen.legacy_divisor_en.eq(legacy_divisor_en)
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
                m.d.sync += position.hibyte.eq(self.in_stream.payload)
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
                    with m.If(position.as_value() == 0):
                        with m.If(shift_cmd.tdo):
                            m.next = "SHIFT-REPORT"
                        with m.Else():
                            m.next = "IDLE"
                    with m.Else():
                        m.d.sync += position.as_value().eq(position.as_value() - 1)

            # XXX: I believe this never worked for >= 1 byte of data
            with m.State("SHIFT-REPORT"):
                with _produce_output(bits_out):
                    m.next = "IDLE"

            # GPIO commands

            with m.State("GPIO-READ-I"):
                with _produce_output(Mux(gpio_cmd_adr == 0, self.pads_i[0:8], self.pads_i[8:16])):
                    m.next = "IDLE"

            with m.State("GPIO-WRITE-O"), _consume_input():
                with m.If(gpio_cmd_adr == 0):
                    m.d.sync += self.pads_o[1:8].eq(self.in_stream.payload[1:8]) # pads_o[0] belongs to tck forever!
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
from amaranth.sim import Tick

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

    def dut_state(self):
        state = {v: k for k, v in self.dut.fsm.encoding.items()}
        return state[(yield self.dut.fsm.state)]

    def write(self, byte):
        yield self.dut.in_stream.payload.eq(byte)
        yield self.dut.in_stream.valid.eq(1)
        for _ in range(32 * self.clkdiv):
            if (yield self.dut.in_stream.ready) == 1:
                yield Tick()
                yield self.dut.in_stream.valid.eq(0)
                return
            yield Tick()
        raise Exception("DUT stuck while writing")

    def read(self):
        yield self.dut.out_stream.ready.eq(1)
        for _ in range(32 * self.clkdiv):
            if (yield self.dut.out_stream.valid) == 1:
                byte = (yield self.dut.out_stream.payload)
                yield Tick()
                yield self.dut.out_stream.ready.eq(0)
                return byte
            yield Tick()
        raise Exception("DUT stuck while reading")

    def _wait_for_tck(self, at_setup=None):
        setup = None
        for _ in range(64 * self.clkdiv):
            if at_setup:
                setup = (yield from at_setup())
            tckold = (yield self.tck)
            yield Tick()
            tcknew = (yield self.tck)
            if tckold != tcknew:
                break
        if tckold == tcknew:
            raise Exception("DUT ceased driving TCK")
        return setup

    def recv_tdi(self, nbits, pos):
        bits = 0
        for n in range(nbits * 2):
            yield from self._wait_for_tck()
            if (yield self.tck) == pos:
                bits = (bits << 1) | (yield self.tdi)
        return bits

    def recv_tms(self, nbits, pos):
        bits = 0
        for n in range(nbits * 2):
            yield from self._wait_for_tck()
            if (yield self.tck) == pos:
                bits = (bits << 1) | (yield self.tms)
        return bits

    def xfer(self, nbits, out_bits, in_bits, out_pos, in_pos):
        for n in range(nbits * 2):
            tdiold = (yield from self._wait_for_tck(
                at_setup=lambda: (yield self.tdi)))
            tcknew = (yield self.tck)

            if in_pos == tcknew:
                if (yield self.tdi) != tdiold:
                    yield Tick(); yield Tick(); yield Tick(); yield Tick()
                    raise Exception("DUT violated setup/hold timings")

                in_bit  = in_bits  & (1 << (nbits - n // 2) - 1)
                # print(f"{in_bits:0{nbits}b} {in_bit:0{nbits}b} ")
                if (yield self.tdi) != (in_bit != 0):
                    yield Tick(); yield Tick(); yield Tick(); yield Tick()
                    raise Exception("DUT clocked out bit {} as {} (expected {})"
                                    .format(n // 2, (yield self.tdi), 1 if in_bit else 0))
            if out_pos == tcknew:
                out_bit = out_bits & (1 << (nbits - n // 2) - 1)
                yield self.tdo.eq(out_bit)

        for _ in range(16 * self.clkdiv):
            tckold = (yield self.tck)
            yield Tick()
            tcknew = (yield self.tck)
            if tckold != tcknew and in_pos == tcknew:
                raise Exception("DUT spuriously drives TCK")

        return True

    def out_xfer(self, nbits, bits, pos):
        return self.xfer(nbits, bits, 0, pos, False)

    def in_xfer(self, nbits, bits, pos):
        return self.xfer(nbits, 0, bits, False, pos)


class MPSSETestCase(unittest.TestCase):
    def setUp(self):
        self.tb = MPSSETestbench()

    def configure(self, tb):
        # speed up tests
        yield tb.dut.legacy_divisor_en.eq(0)

    @simulation_test
    def test_error(self, tb):
        yield from tb.write(0xFF)
        self.assertEqual((yield from tb.read()), 0xFA)
        self.assertEqual((yield from tb.read()), 0xFF)
        self.assertEqual((yield from tb.dut_state()), "IDLE")

    @simulation_test
    def test_gpio_read(self, tb):
        yield tb.dut.pads_i.eq(0xAA55)
        yield Tick()
        yield from tb.write(0x81)
        self.assertEqual((yield from tb.read()), 0x55)
        yield from tb.write(0x83)
        self.assertEqual((yield from tb.read()), 0xAA)
        self.assertEqual((yield from tb.dut_state()), "IDLE")

    @simulation_test
    def test_gpio_write(self, tb):
        yield from tb.write(0x80)
        yield from tb.write(0xA1)
        yield from tb.write(0x52)
        self.assertEqual((yield tb.dut.pads_o),  0x00A1)
        self.assertEqual((yield tb.dut.pads_oe), 0x0052)
        yield from tb.write(0x82)
        yield from tb.write(0x7E)
        yield from tb.write(0x81)
        self.assertEqual((yield tb.dut.pads_o),  0x7EA1)
        self.assertEqual((yield tb.dut.pads_oe), 0x8152)
        self.assertEqual((yield from tb.dut_state()), "IDLE")

    @simulation_test
    def test_bits_write(self, tb):
        yield from tb.write(0x12)
        yield from tb.write(5)
        self.assertEqual((yield tb.dut.position.bit), 5)
        yield from tb.write(0x55)
        self.assertEqual((yield from tb.recv_tdi(5, pos=True)), 0x0A)

    @simulation_test
    def test_clk_bits(self, tb):
        yield from tb.write(0x8E)
        yield from tb.write(5)
        self.assertEqual((yield tb.dut.position.bit), 5)
        self.assertEqual((yield from tb.recv_tdi(6, pos=True)), 0x00)
        self.assertEqual((yield tb.tck), 0)
        self.assertEqual((yield from tb.dut_state()), "IDLE")

    @simulation_test
    def test_clk_bytes(self, tb):
        yield from tb.write(0x8F)
        yield from tb.write(5)
        yield from tb.write(0)
        self.assertEqual((yield tb.dut.position.lobyte), 5)
        self.assertEqual((yield tb.dut.position.hibyte), 0)
        self.assertEqual((yield from tb.recv_tdi(48, pos=True)), 0x00)
        self.assertEqual((yield tb.tck), 0)
        self.assertEqual((yield from tb.dut_state()), "IDLE")

    @simulation_test
    def test_bits_read(self, tb):
        yield from tb.write(0x22)
        yield from tb.write(5)
        self.assertEqual((yield tb.dut.position.bit), 5)
        self.assertEqual((yield from tb.read()), 0x00)

    @simulation_test
    def test_legacy_dividor(self, tb):
        # restore pristine MPSSE state
        yield tb.dut.legacy_divisor_en.eq(0)
        self.tb.clkdiv = 5

        # works
        yield from tb.write(0x22)
        yield from tb.write(5)
        self.assertEqual((yield tb.dut.position.bit), 5)
        self.assertEqual((yield from tb.read()), 0x00)

        # works
        yield from tb.write(0x8A)
        self.tb.clkdiv = 1
        yield from tb.write(0x22)
        yield from tb.write(5)
        self.assertEqual((yield tb.dut.position.bit), 5)
        self.assertEqual((yield from tb.read()), 0x00)

        # fails - timeout
        yield from tb.write(0x8B)
        self.tb.clkdiv = 1
        yield from tb.write(0x22)
        yield from tb.write(5)
        self.assertEqual((yield tb.dut.position.bit), 5)
        with self.assertRaises(Exception):
            yield from tb.read()

    @simulation_test
    def test_bits_read_write(self, tb):
        yield from tb.write(0x84)
        yield from tb.write(0x33)
        yield from tb.write(5)
        self.assertEqual((yield tb.dut.position.bit), 5)
        yield from tb.write(0x55)
        self.assertEqual((yield from tb.recv_tdi(5, pos=True)), 0x0A)
        self.assertEqual((yield from tb.read()), 0x15) # non-negative read clock

    @simulation_test
    def test_tms_write(self, tb):
        yield from tb.write(0x4A)
        yield from tb.write(5)
        self.assertEqual((yield tb.dut.position.bit), 5)
        yield from tb.write(0x55)
        self.assertEqual((yield from tb.recv_tms(5, pos=True)), 0x15)

    @simulation_test
    def test_invalid_tms_commands(self, tb):
        yield from tb.write(0x5A)
        self.assertEqual((yield from tb.dut_state()), "ERROR")
        yield from tb.read()
        yield from tb.read()
        self.assertEqual((yield from tb.dut_state()), "IDLE")
        yield from tb.write(0x42)
        self.assertEqual((yield from tb.dut_state()), "ERROR")
        yield from tb.read()
        yield from tb.read()
        yield from tb.write(0x68)
        self.assertEqual((yield from tb.dut_state()), "ERROR")
        yield from tb.read()
        yield from tb.read()

    @simulation_test
    def test_hibyte_lobyte_write(self, tb):
        yield from tb.write(0x10)
        yield from tb.write(0x05)
        self.assertEqual((yield tb.dut.position.lobyte), 0x05)
        yield from tb.write(0x11)
        self.assertEqual((yield tb.dut.position.hibyte), 0x11)

    @simulation_test
    def test_divisor_write(self, tb):
        yield from tb.write(0x86)
        yield from tb.write(0x34)
        yield from tb.write(0x12)
        self.assertEqual((yield tb.dut.divisor), 0x1234)
        self.assertEqual((yield from tb.dut_state()), "IDLE")

    def write_single_byte(self, tb, pos):
        yield from tb.write(0x80)
        if pos:
            yield from tb.write(0b0001)
        else:
            yield from tb.write(0b0000)
        yield from tb.write(0b1101)

        if pos:
            yield from tb.write(0x10)
        else:
            yield from tb.write(0x11)
        yield from tb.write(0x00)
        yield from tb.write(0x00)
        yield from tb.write(0xA5)
        self.assertTrue((yield from tb.in_xfer(8, 0b10100101, not pos)))

    @simulation_test
    def test_write_single_byte_clkpos(self, tb):
        yield tb.dut.divisor.eq(1)
        yield from self.write_single_byte(tb, pos=True)

    @simulation_test
    def test_write_single_byte_clkneg(self, tb):
        yield tb.dut.divisor.eq(1)
        yield from self.write_single_byte(tb, pos=False)

    @simulation_test
    def test_write_single_byte_clkneg_fast(self, tb):
        yield from self.write_single_byte(tb, pos=False)

    @simulation_test
    def test_write_single_byte_clkwrong(self, tb):
        yield from tb.write(0x10) # +ve, but we start from tck=0
        yield from tb.write(0x00)
        yield from tb.write(0x00)
        yield from tb.write(0xA5)
        self.assertEqual((yield from tb.recv_tdi(8, pos=False)), 0xA5)
        yield Tick()
        self.assertEqual((yield tb.tck), 0)
