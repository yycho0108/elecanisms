
import usb.core

class miniproj2:

    def __init__(self):
        self.SET_SERVOS = 1
        self.GET_SERVOS = 2
        self.PING = 3
        self.dev = usb.core.find(idVendor = 0x6666, idProduct = 0x0003)
        if self.dev is None:
            raise ValueError('no USB device found matching idVendor = 0x6666 and idProduct = 0x0003')
        self.dev.set_configuration()

    def close(self):
        self.dev = None

    def set_servos(self, servo1, servo2):
        try:
            self.dev.ctrl_transfer(0x40, self.SET_SERVOS, servo1, servo2)
        except usb.core.USBError:
            print "Could not send SET_SERVOS vendor request."

    def get_servos(self):
        try:
            ret = self.dev.ctrl_transfer(0xC0, self.GET_SERVOS, 0, 0, 4)
        except usb.core.USBError:
            print "Could not send GET_SERVOS vendor request."
        else:
            return [int(ret[0])+int(ret[1])*256, int(ret[2])+int(ret[3])*256]

    def ping(self):
        try:
            self.dev.ctrl_transfer(0x40, self.PING)
        except usb.core.USBError:
            print "Could not send PING vendor request."

