import asyncio
import fakeusb1
import usb1
import pyusbip
from usb_construct.emitters.descriptors import DeviceDescriptorCollection

from enum import IntEnum

class Ft2232Device(fakeusb1.BaseFakeUSBDevice):
    def build_descriptors(self, collection):
        with collection.DeviceDescriptor() as d:
            d.idVendor           = 0x0403
            d.idProduct          = 0x6010
            d.bNumConfigurations = 1
            d.bcdDevice          = 7.00 # pretend to be a FT2232H

            d.iManufacturer      = 'Emarhavil Heavy Industries'
            d.iProduct           = 'FTDEEZ'
            d.iSerialNumber      = '123456'
        
        with collection.ConfigurationDescriptor() as c:
            with c.InterfaceDescriptor() as i:
                i.bInterfaceNumber = 0
                with i.EndpointDescriptor() as e:
                    e.bEndpointAddress = 0x02
                    e.wMaxPacketSize   = 512
                
                with i.EndpointDescriptor() as e:
                    e.bEndpointAddress = 0x82
                    e.wMaxPacketSize   = 512

            with c.InterfaceDescriptor() as i:
                i.bInterfaceNumber = 1
                with i.EndpointDescriptor() as e:
                    e.bEndpointAddress = 0x01
                    e.wMaxPacketSize   = 512
                
                with i.EndpointDescriptor() as e:
                    e.bEndpointAddress = 0x81
                    e.wMaxPacketSize   = 512
    
    def controlRead(self, bRequestType, bRequest, wValue, wIndex, wLength):
        match (bRequestType, bRequest):
            case (0xC0, 0x05): # GET_MODEM_STATUS
                print(f"GET_MODEM_STATUS channel {wIndex}")
                return b'\x01\x40'
            case (0xC0, 0x0A): # GET_LATENCY_TIMER
                print(f"GET_LATENCY_TIMER channel {wIndex}")
                return b'\x10'
            case (0xC0, 0x0C): # GET_PIN_STATE
                print(f"GET_PIN_STATE channel {wIndex}")
                return b'\x00'
            case (0xC0, 0x20): # VENDOR_CMD_GET
                print(f"VENDOR_CMD_GET")
                raise usb1.USBErrorPipe
            case (0xC0, 0x90): # READ_EEPROM
                print(f"READ_EEPROM address {wIndex:04x}")
                return b'\x00\x00'
        return super().controlRead(bRequestType, bRequest, wValue, wIndex, wLength)
    
    def controlWrite(self, bRequestType, bRequest, wValue, wIndex, buf):
        match (bRequestType, bRequest):
            case (0x40, 0x00): # RESET
                print("RESET")
                return 0
            case (0x40, 0x01): # SET_MODEM_CTRL
                print("SET_MODEM_CTRL")
            case (0x40, 0x02): # SET_FLOW_CTRL
                print("SET_FLOW_CTRL")
                return 0
            case (0x40, 0x03): # SET_BAUD_RATE
                print("SET_BAUD_RATE")
                return 0
            case (0x40, 0x04): # SET_DATA_CHARACTERISTICS
                print("SET_DATA_CHARACTERISTICS")
                return 0
            case (0x40, 0x06): # SET_EVENT_CHAR
                print("SET_EVENT_CHAR")
            case (0x40, 0x07): # SET_ERROR_CHAR
                print("SET_ERROR_CHAR")
            case (0x40, 0x09): # SET_LATENCY_TIMER
                print("SET_LATENCY_TIMER")
                return 0
            case (0x40, 0x0b): # SET_BITMODE
                print("SET_BITMODE")
            case (0x40, 0x21): # VENDOR_CMD_SET
                print("VENDOR_CMD_SET")
            case (0x40, 0x91): # WRITE_EEPROM
                print("WRITE_EEPROM")
            case (0x40, 0x92): # ERASE_EEPROM
                print("ERASE_EEPROM")
        raise usb1.USBErrorPipe
    
    async def bulk_in(self, ep, buflen):
        print(f"BULK IN ON EP {ep}")
        await asyncio.sleep(0.1)
        return b'\x01\x40'
    
    async def bulk_out(self, ep, buf):
        print(f"BULK OUT ON EP {ep}: {buf}")
        return len(buf)

dev = Ft2232Device()
usbctx = fakeusb1.FakeUSBContext(devices=[dev])
pyusbip.main(usbctx, host='0.0.0.0')

