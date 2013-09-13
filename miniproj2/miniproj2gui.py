
import Tkinter as tk
import miniproj2

class miniproj2gui:

    def __init__(self):
        self.dev = miniproj2.miniproj2()
        if self.dev.dev>=0:
            self.root = tk.Tk()
            self.root.title('Miniproject 2 GUI')
            [servo1, servo2] = self.dev.get_servos()
            self.servo1slider = tk.Scale(self.root, from_ = 0, to = 65535, orient = tk.HORIZONTAL, showvalue = tk.FALSE, command = self.update_servo1)
            self.servo1slider.set(servo1)
            self.servo1slider.pack(side = tk.TOP)
            self.servo2slider = tk.Scale(self.root, from_ = 0, to = 65535, orient = tk.HORIZONTAL, showvalue = tk.FALSE, command = self.update_servo2)
            self.servo2slider.set(servo2)
            self.servo2slider.pack(side = tk.TOP)
            tk.Button(self.root, text = 'Ping!', command = self.dev.ping).pack(side = tk.TOP)

    def update_servo1(self, servo1):
        servo2 = self.servo2slider.get()
        self.dev.set_servos(int(servo1), int(servo2))

    def update_servo2(self, servo2):
        servo1 = self.servo1slider.get()
        self.dev.set_servos(int(servo1), int(servo2))

if __name__=='__main__':
    gui = miniproj2gui()
    gui.root.mainloop()
