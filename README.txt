ftdeez: FTD2xx Emulation, easy.  Nuts.

Emulate a FT2232H using USB/IP.  (Or at least a very limited subset of it --
like a UART with no flow control.)

Try:

$ python3 ftdeez.py

Or if you have a Glasgow, connect it, and connect 3.3V peripherals to
{tx,rx} = {A0,A1} and {tx,rx} = {B0,B1}.  (Or cross-connect it externally). 
Then try:

$ python3 ftdeez_glasgow.py 

After doing either of them, do something like this (hopefully, from a VM,
given the stability of Linux USBIP):

# usbip attach -r 192.168.122.1 -d 1-1
# picocom /dev/ttyUSB0

(where 192.168.122.1 is the IP of the machine that is running `ftdeez`)
