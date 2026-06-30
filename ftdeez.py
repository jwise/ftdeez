import asyncio
import struct
import logging
import time
from enum import IntEnum

import usb1
from usb_construct.emitters.descriptors import DeviceDescriptorCollection

import fakeusb1
import pyusbip

_logger = logging.getLogger("ftdeez")

class BaseD2xxChannel:
    def __init__(self, channel):
        self.channel = channel
        self.latency_timer = 0x10
        self._logger = logging.getLogger(f"ftdeez.BaseD2xxChannel.{id(self)}")
        self._in_lock = asyncio.Lock()
        self._in_cvar = asyncio.Condition()
        self._in_latency_timer_start = None
        self._in_event = False
        self._in_buf = bytearray()
    
    def get_modem_status(self):
        return 0x0140
    
    def get_latency_timer(self):
        return self.latency_timer

    def set_latency_timer(self, wValue):
        self.latency_timer = wValue
    
    def get_pin_state(self):
        # return DBUS pin state
        return 0x00
    
    def reset(self, wValue):
        pass
    
    def set_modem_ctrl(self, wValue):
        pass
    
    def set_flow_ctrl(self, wValue, wIndex):
        pass
    
    def set_baud_rate(self, divisor):
        pass
    
    def set_data_characteristics(self, wValue):
        pass
    
    def set_event_char(self, wValue):
        pass
    
    def set_error_char(self, wValue):
        pass
    
    def set_bitmode(self, wValue):
        pass

    async def bulk_in(self, wLength):
        # only one bulk IN URB can be ready to return at a time
        async with self._in_lock:
            async with self._in_cvar:
                while True:
                    if self._in_event:
                        self._logger.debug("returning IN packet due to event")
                        break
                    if len(self._in_buf) >= (wLength - 2):
                        self._logger.debug("returning IN packet due to buffer full")
                        break
                    if len(self._in_buf) > 0 and not self._in_latency_timer_start:
                        self._logger.debug("starting latency timer")
                        self._in_latency_timer_start = time.time()
                    
                    timeout_remaining = 2**32
                    if self._in_latency_timer_start:
                        timeout_remaining = (self._in_latency_timer_start + self.latency_timer * 0.001) - time.time()
                    if timeout_remaining < 0:
                        self._logger.debug("returning IN packet due to latency timer expiring")
                        break
                    
                    try:
                        await asyncio.wait_for(self._in_cvar.wait(), timeout_remaining)
                    except TimeoutError:
                        pass
                
                self._in_event = False
                self._in_latency_timer_start = None
                
                bs = self._in_buf[:wLength - 2]
                self._in_buf[:wLength - 2] = b''
                
            return struct.pack(">H", self.get_modem_status()) + bs
    
    async def bulk_out(self, buf):
        await self.put_infifo(buf)
        return len(buf)
    
    async def put_infifo(self, buf):
        # handle event char
        async with self._in_cvar:
            self._in_buf.extend(buf)
            self._in_cvar.notify_all()
        

class Ft2232Device(fakeusb1.BaseFakeUSBDevice):
    def __init__(self,
                 channels = None,
                 idVendor = 0x0403,
                 idProduct = 0x6010,
                 iManufacturer = 'Emarhavil Heavy Industries',
                 iProduct = 'FTDEEZ',
                 iSerialNumber = '123456'):
        if channels is None:
            channels = [BaseD2xxChannel(0), BaseD2xxChannel(1)]
        self.channels = channels
        self.idVendor = idVendor
        self.idProduct = idProduct
        self.iManufacturer = iManufacturer
        self.iProduct = iProduct
        self.iSerialNumber = iSerialNumber
        super().__init__()
    
    def __setattr__(self, k, v):
        super().__setattr__(k, v)
        if (k == "bus_number" or k == "device_address") and getattr(self, "bus_number", None) and getattr(self, "device_address", None):
            self._logger = logging.getLogger(f"ftdeez.Ft2232Device.{self.bus_number}-{self.device_address}")
    
    def build_descriptors(self, collection):
        with collection.DeviceDescriptor() as d:
            d.idVendor           = self.idVendor
            d.idProduct          = self.idProduct
            d.bNumConfigurations = 1
            d.bcdDevice          = 7.00 # pretend to be a FT2232H

            d.iManufacturer      = self.iManufacturer
            d.iProduct           = self.iProduct
            d.iSerialNumber      = self.iSerialNumber
        
        with collection.ConfigurationDescriptor() as c:
            for chn,ch in enumerate(self.channels):
                with c.InterfaceDescriptor() as i:
                    i.bInterfaceNumber = chn
                    with i.EndpointDescriptor() as e:
                        e.bEndpointAddress = 0x01 + chn
                        e.wMaxPacketSize   = 512
                
                    with i.EndpointDescriptor() as e:
                        e.bEndpointAddress = 0x81 + chn
                        e.wMaxPacketSize   = 512
    
    def controlRead(self, bRequestType, bRequest, wValue, wIndex, wLength):
        match (bRequestType, bRequest):
            case (0xC0, 0x05): # GET_MODEM_STATUS
                self._logger.debug(f"GET_MODEM_STATUS channel {wIndex - 1}")
                return struct.pack(">H", self.channels[wIndex - 1].get_modem_status())
            case (0xC0, 0x0A): # GET_LATENCY_TIMER
                self._logger.debug(f"GET_LATENCY_TIMER channel {wIndex - 1}")
                return struct.pack(">B", self.channels[wIndex - 1].get_latency_timer())
            case (0xC0, 0x0C): # GET_PIN_STATE
                self._logger.debug(f"GET_PIN_STATE channel {wIndex - 1}")
                return struct.pack(">B", self.channels[wIndex - 1].get_pin_state())
            case (0xC0, 0x20): # VENDOR_CMD_GET
                self._logger.error(f"VENDOR_CMD_GET")
                raise usb1.USBErrorPipe
            case (0xC0, 0x90): # READ_EEPROM
                self._logger.debug(f"READ_EEPROM address {wIndex:04x}")
                return b'\x00\x00'
        return super().controlRead(bRequestType, bRequest, wValue, wIndex, wLength)
    
    def controlWrite(self, bRequestType, bRequest, wValue, wIndex, buf):
        match (bRequestType, bRequest):
            case (0x40, 0x00): # RESET
                self.channels[wIndex - 1].reset(wValue)
                self._logger.debug(f"RESET channel {wIndex - 1}")
                return 0
            case (0x40, 0x01): # SET_MODEM_CTRL
                self._logger.debug(f"SET_MODEM_CTRL channel {wIndex} -> 0x{wValue:04x}")
                self.channels[wIndex - 1].set_modem_ctrl(wValue)
                return 0
            case (0x40, 0x02): # SET_FLOW_CTRL
                self._logger.debug(f"SET_FLOW_CTRL channel {(wIndex & 0xFF) - 1} -> 0x{wValue:04x}, 0x{wIndex:04x}")
                self.channels[(wIndex & 0xFF) - 1].set_flow_ctrl(wValue, wIndex)
                return 0
            case (0x40, 0x03): # SET_BAUD_RATE
                self._logger.debug(f"SET_BAUD_RATE channel {(wIndex & 0xFF) - 1} -> 0x{((wIndex & 0xFF00) << 8) | wValue:05x}")
                self.channels[(wIndex & 0xFF) - 1].set_baud_rate(((wIndex & 0xFF00) << 8) | wValue)
                return 0
            case (0x40, 0x04): # SET_DATA_CHARACTERISTICS
                self._logger.debug(f"SET_DATA_CHARACTERISTICS channel {wIndex} -> 0x{wValue:04x}")
                self.channels[wIndex - 1].set_data_characteristics(wValue)
                return 0
            case (0x40, 0x06): # SET_EVENT_CHAR
                self._logger.debug(f"SET_EVENT_CHAR channel {wIndex} -> 0x{wValue:04x}")
                self.channels[wIndex - 1].set_event_char(wValue)
                return 0
            case (0x40, 0x07): # SET_ERROR_CHAR
                self._logger.debug(f"SET_ERROR_CHAR channel {wIndex} -> 0x{wValue:04x}")
                self.channels[wIndex - 1].set_error_char(wValue)
                return 0
            case (0x40, 0x09): # SET_LATENCY_TIMER
                self._logger.debug(f"SET_LATENCY_TIMER channel {wIndex} -> 0x{wValue:04x}")
                self.channels[wIndex - 1].set_latency_timer(wValue)
                return 0
            case (0x40, 0x0b): # SET_BITMODE
                self._logger.debug(f"SET_BITMODE channel {wIndex} -> 0x{wValue:04x}")
                self.channels[wIndex - 1].set_bitmode(wValue)
                return 0
            case (0x40, 0x21): # VENDOR_CMD_SET
                self._logger.error("VENDOR_CMD_SET")
            case (0x40, 0x91): # WRITE_EEPROM
                self._logger.error("WRITE_EEPROM")
            case (0x40, 0x92): # ERASE_EEPROM
                self._logger.error("ERASE_EEPROM")
        raise usb1.USBErrorPipe
    
    async def bulk_in(self, ep, buflen):
        self._logger.debug(f"bulk IN for channel {(ep & 0x7F) - 1}, len = {buflen} bytes")
        rv = await self.channels[(ep & 0x7F) - 1].bulk_in(buflen)
        self._logger.debug(f"bulk IN for channel {(ep & 0x7F) - 1} complete: {rv}")
        return rv
    
    async def bulk_out(self, ep, buf):
        self._logger.debug(f"bulk OUT for channel {(ep & 0x7F) - 1}: {buf}")
        return await self.channels[ep - 1].bulk_out(buf)

def setup_logging():
    logging.getLogger().setLevel(logging.INFO)
    _logger.setLevel(logging.DEBUG)
    ch = logging.StreamHandler()
    ch.setLevel(logging.DEBUG)
    ch.setFormatter(logging.Formatter("[%(asctime)s] %(name)s: %(levelname)s: %(message)s"))
    logging.getLogger().addHandler(ch)

if __name__ == "__main__":
    setup_logging()

    dev = Ft2232Device()
    usbctx = fakeusb1.FakeUSBContext(devices=[dev])
    pyusbip.main(usbctx, host='0.0.0.0')

