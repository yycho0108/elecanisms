
import usb.core

class hellousb:

    def __init__(self):
        self.HELLO = 0
        self.SET_VALS = 1
        self.GET_VALS = 2
        self.PRINT_VALS = 3
        self.TOGGLE_LED = 4
        self.SET_DUTY = 5
        self.GET_DUTY = 6
        self.dev = usb.core.find(idVendor = 0x6666, idProduct = 0x0003)
        if self.dev is None:
            raise ValueError('no USB device found matching idVendor = 0x6666 and idProduct = 0x0003')
        self.dev.set_configuration()

    def close(self):
        self.dev = None
    def xfer(self,flag,wValue=0,wIndex=0,data_or_wLength=None):
        try:
            ret = self.dev.ctrl_transfer(0x40,flag,wValue,wIndex,data_or_wLength)
        except usb.core.USBError:
            print "Could not send vendoer request."
        return ret
    def hello(self):
        self.xfer(self.HELLO)
    def set_vals(self, val1, val2):
        self.xfer(self.SET_VALS, int(val1), int(val2))
    def get_vals(self):
        try:
            ret = self.dev.ctrl_transfer(0xC0, self.GET_VALS, 0, 0, 4)
        except usb.core.USBError:
            print "Could not send GET_VALS vendor request."
        else:
            return [int(ret[0])+int(ret[1])*256, int(ret[2])+int(ret[3])*256]
    def toggle_led(self):
        try:
            self.dev.ctrl_transfer(0x40, self.TOGGLE_LED)
        except usb.core.USBError:
            print "Could not send HELLO vendor request."
    def set_duty(self):
        pass
    def get_duty(self):
        pass
    def print_vals(self):
        try:
            self.dev.ctrl_transfer(0x40, self.PRINT_VALS)
        except usb.core.USBError:
            print "Could not send PRINT_VALS vendor request."
