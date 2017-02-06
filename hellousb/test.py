import hellousb
import time

h = hellousb.hellousb()
h.hello()

while (True):
    h.set_vals(1,2)
    a,b = h.get_vals()
    print 'a,b', a,b
    h.print_vals()
    time.sleep(.02)
