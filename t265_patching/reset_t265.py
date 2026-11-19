import ctypes
import ctypes.util
import time

libusb = ctypes.CDLL(ctypes.util.find_library("usb-1.0"))

class timeval(ctypes.Structure):
    _fields_ = [("tv_sec", ctypes.c_long), ("tv_usec", ctypes.c_long)]

# Function signatures
libusb.libusb_init.argtypes = [ctypes.POINTER(ctypes.c_void_p)]
libusb.libusb_init.restype = ctypes.c_int

libusb.libusb_exit.argtypes = [ctypes.c_void_p]
libusb.libusb_exit.restype = None

libusb.libusb_open_device_with_vid_pid.argtypes = [ctypes.c_void_p, ctypes.c_uint16, ctypes.c_uint16]
libusb.libusb_open_device_with_vid_pid.restype = ctypes.c_void_p

libusb.libusb_close.argtypes = [ctypes.c_void_p]
libusb.libusb_close.restype = None

libusb.libusb_control_transfer.argtypes = [
    ctypes.c_void_p, ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint16, ctypes.c_uint16,
    ctypes.c_char_p, ctypes.c_uint16, ctypes.c_uint
]
libusb.libusb_control_transfer.restype = ctypes.c_int

def reset_t265_to_bootloader():
    ctx = ctypes.c_void_p()
    r = libusb.libusb_init(ctypes.byref(ctx))
    if r != 0:
        print(f"libusb_init failed: {r}")
        return False
    
    # Try open booted T265: VID 0x8087, PID 0x0B37
    handle = libusb.libusb_open_device_with_vid_pid(ctx, 0x8087, 0x0b37)
    if not handle:
        print("T265 is not currently in booted state (8087:0b37). Checking unbooted state...")
        unbooted = libusb.libusb_open_device_with_vid_pid(ctx, 0x03e7, 0x2150)
        if unbooted:
            print("T265 is already in unbooted ROM bootloader state (03e7:2150)!")
            libusb.libusb_close(unbooted)
            libusb.libusb_exit(ctx)
            return True
        else:
            print("No T265 device found in either booted or unbooted state.")
            libusb.libusb_exit(ctx)
            return False
            
    print("Found booted T265 (8087:0b37). Sending USB reset control transfer (bmRequestType=0x40, bRequest=0x10)...")
    # control transfer: bmRequestType=0x40, bRequest=0x10 (CONTROL_USB_RESET)
    res = libusb.libusb_control_transfer(handle, 0x40, 0x10, 0, 0, None, 0, 1000)
    print(f"Control transfer result: {res}")
    libusb.libusb_close(handle)
    libusb.libusb_exit(ctx)
    return True

if __name__ == "__main__":
    reset_t265_to_bootloader()
