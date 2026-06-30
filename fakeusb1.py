import logging
import asyncio
import usb1
import traceback
from usb_construct.emitters.descriptors import DeviceDescriptorCollection
from usb_construct.types.descriptors import DeviceDescriptor, ConfigurationDescriptor, InterfaceDescriptor, EndpointDescriptor
from usb_construct.types.descriptors.standard import StandardDescriptorNumbers

from abc import *

from enum import IntEnum

USB_EPIPE = 32

class USB_bRequestType_Type(IntEnum):
    STANDARD = 0
    CLASS = 1
    VENDOR = 2
    RESERVED = 3

class USB_bRequestType_Recipient(IntEnum):
    DEVICE = 0
    INTERFACE = 1
    ENDPOINT = 2
    OTHER = 3

class USB_bRequest_device(IntEnum):
    GET_STATUS = 0x00
    CLEAR_FEATURE = 0x01
    GET_DESCRIPTOR = 0x06

class FakeUSBContext:
    # Sort of like a libusb1 usbctx, but it contains fake devices.
    def __init__(self, devices = []):
        self.devices = devices
        for n,d in enumerate(self.devices):
            d.bus_number = 1
            d.device_address = n+1
    
    def getDeviceList(self):
        return self.devices

class AbstractFakeUSBDevice(ABC):
    def __init__(self):
        self.bus_number = 0
        self.device_address = 0
    
    def getBusNumber(self):
        return self.bus_number

    def getDeviceAddress(self):
        return self.device_address
    
    def getDeviceSpeed(self):
        return usb1.SPEED_HIGH
    
    @abstractmethod
    def getVendorID(self):
        pass
    
    @abstractmethod
    def getProductID(self):
        pass
    
    @abstractmethod
    def getbcdDevice(self):
        pass
    
    @abstractmethod
    def getDeviceClass(self):
        pass
    
    @abstractmethod
    def getDeviceSubClass(self):
        pass
    
    @abstractmethod
    def getDeviceProtocol(self):
        pass
    
    class Interface:
        def __init__(self, clazz, subclass, protocol):
            self.clazz = clazz
            self.subclass = subclass
            self.protocol = protocol
        
        def getClass(self):
            return self.clazz
        
        def getSubClass(self):
            return self.subclass
        
        def getProtocol(self):
            return self.protocol
    
    class Configuration:
        def __init__(self):
            self.interfaces = []
            self.configuration_value = 1
        
        def getNumInterfaces(self):
            return len(self.interfaces)
        
        def getConfigurationValue(self):
            return self.configuration_value
        
        def iterInterfaces(self):
            return self.interfaces
    
    @abstractmethod
    def iterConfigurations(self):
        pass
    
    @abstractmethod
    def getNumConfigurations(self):
        pass
    
    def open(self):
        # return a handle, which for us is just the same as the device
        return self

    # HANDLES
    @abstractmethod
    def getConfiguration(self):
        pass
    
    @abstractmethod
    def setConfiguration(self):
        pass
    
    def close(self):
        pass
    
    def getDevice(self):
        return self
    
    def claimInterface(self, i):
        pass
    
    @abstractmethod
    def setInterfaceAltSetting(self, wIndex, wValue):
        pass
    
    @abstractmethod
    def controlRead(self, bRequestType, bRequest, wValue, wIndex, wLength):
        pass
    
    @abstractmethod
    def controlWrite(self, bRequestType, bRequest, wValue, wIndex, buf):
        pass
    
    class Transfer(ABC):
        @abstractmethod
        def getStatus(self):
            pass
        
        @abstractmethod
        def getActualLength(self):
            pass
        
        @abstractmethod
        def getBuffer(self):
            pass
        
        @abstractmethod
        def setBulk(self, ep, buf_or_buflen, callback):
            pass
        
        @abstractmethod
        def submit(self):
            pass
        
    @abstractmethod
    def getTransfer(self) -> Transfer:
        pass

class BaseFakeUSBDevice(AbstractFakeUSBDevice):
    # Handles much of what you need to be a USB device.
    def __init__(self):
        self._descriptors = DeviceDescriptorCollection()
        self.build_descriptors(self._descriptors)
        self.configuration = 0
        super().__init__()

    def __setattr__(self, k, v):
        super().__setattr__(k, v)
        if (k == "bus_number" or k == "device_address") and getattr(self, "bus_number", None) and getattr(self, "device_address", None):
            self._fakeusbdevice_logger = logging.getLogger(f"fakeusb1.BaseFakeUSBDevice.{self.bus_number}-{self.device_address}")
    
    @abstractmethod
    def build_descriptors(self, collection):
        pass
    
    @property
    def _device_descriptor(self):
        return DeviceDescriptor.parse(self._descriptors.get_descriptor_bytes(StandardDescriptorNumbers.DEVICE, 0))
    
    def getVendorID(self):
        return self._device_descriptor.idVendor
    
    def getProductID(self):
        return self._device_descriptor.idProduct
    
    def getbcdDevice(self):
        return int(self._device_descriptor.bcdDevice * 256)
    
    def getDeviceClass(self):
        return self._device_descriptor.bDeviceClass
    
    def getDeviceSubClass(self):
        return self._device_descriptor.bDeviceSubclass
    
    def getDeviceProtocol(self):
        return self._device_descriptor.bDeviceProtocol

    def iterConfigurations(self):
        configs = []
        for c in range(self.getNumConfigurations()):
            config = AbstractFakeUSBDevice.Configuration()
        
            bs = self._descriptors.get_descriptor_bytes(StandardDescriptorNumbers.CONFIGURATION, c)
            config_d = ConfigurationDescriptor.parse(bs)
            bs = bs[config_d.bLength:]
            
            config.configuration_value = config_d.bConfigurationValue
            
            for i in range(config_d.bNumInterfaces):
                interf_d = InterfaceDescriptor.parse(bs)
                bs = bs[interf_d.bLength:]
                
                for _ in range(interf_d.bNumEndpoints):
                    ep_d = EndpointDescriptor.parse(bs)
                    bs = bs[ep_d.bLength:]
                
                interf = AbstractFakeUSBDevice.Interface(interf_d.bInterfaceClass, interf_d.bInterfaceSubclass, interf_d.bInterfaceProtocol)
                config.interfaces.append(interf)
            
            configs.append(config)
        
        return configs
    
    def getNumConfigurations(self):
        return self._device_descriptor.bNumConfigurations
    
    def getConfiguration(self):
        return self.configuration
    
    def setConfiguration(self, config):
        self.configuration = config
    
    def setInterfaceAltSetting(self, wIndex, wValue):
        pass
    
    def controlRead(self, bRequestType, bRequest, wValue, wIndex, wLength):
        bRequestType_type = (bRequestType >> 5) & 3
        bRequestType_recipient = bRequestType & 0x1F
        
        self._fakeusbdevice_logger.debug(f"controlRead(bRequestType = {bRequestType:02x}, bRequest = {bRequest:02x}, wValue = {wValue:04x}, wIndex = {wIndex:04x}, wLength = {wLength:04x})")
        match ((bRequestType >> 5) & 3, bRequestType & 0x1F, bRequest):
            case (USB_bRequestType_Type.STANDARD, USB_bRequestType_Recipient.DEVICE, USB_bRequest_device.GET_DESCRIPTOR):
                try:
                    # ignore language ID for now
                    bs = self._descriptors.get_descriptor_bytes(wValue >> 8, wValue & 0xFF)
                except:
                    self._fakeusbdevice_logger.error("no descriptor?")
                    raise usb1.USBErrorPipe
                return bs[:wLength]
            case (USB_bRequestType_Type.STANDARD, USB_bRequestType_Recipient.DEVICE, _):
                self._fakeusbdevice_logger.error("unknown bRequest for device")
                raise usb1.USBErrorPipe
            case _:
                self._fakeusbdevice_logger.error("unknown target for request")

    def controlWrite(self, bRequestType, bRequest, wValue, wIndex, buf):
        self._fakeusbdevice_logger.error("unsupported controlWrite")
        raise usb1.USBErrorPipe

    class Transfer(AbstractFakeUSBDevice.Transfer):
        def __init__(self, parent):
            self.parent = parent
            self.status = 0
            self.actual_length = 0
            self.buffer = b''
            self.ep = None
            self.buflen = None
            self.callback = None
            self.task = None

        def getStatus(self):
            return self.status
        
        def getActualLength(self):
            return self.actual_length
        
        def getBuffer(self):
            return self.buffer
        
        def setBulk(self, ep, buf_or_buflen, callback):
            self.ep = ep
            self.buffer = buf_or_buflen
            self.callback = callback
        
        def submit(self):
            async def run_xfer():
                try:
                    if self.ep & 0x80:
                        rv = await self.parent.bulk_in(self.ep, self.buffer)
                        if type(rv) == bytes:
                            self.actual_length = len(rv)
                            self.status = 0
                            self.buffer = rv
                        else:
                            self.actual_length = 0
                            self.status = -rv
                            self.buffer = b''
                    else:
                        rv = await self.parent.bulk_out(self.ep, self.buffer)
                        if rv >= 0:
                            self.actual_length = rv
                            self.status = 0
                        else:
                            self.status = -rv
                            self.actual_length = 0
                except Exception as e:
                    traceback.print_exc()
                    self.parent._fakeusbdevice_logger.error("URB exception")
                    self.status = -USB_EPIPE
                    self.actual_length = 0
                self.callback(self)
            self.task = asyncio.create_task(run_xfer())
    
        def cancel(self):
            self.parent._fakeusbdevice_logger.error("URB cancelled")
            self.task.cancel()
            self.status = -USB_EPIPE
            self.actual_length = 0
            self.buffer = b''
            self.callback(self)
        
    def getTransfer(self):
        return self.Transfer(self)

    @abstractmethod
    async def bulk_in(self, ep, buflen):
        pass

    @abstractmethod
    async def bulk_out(self, ep, buf):
        pass
