#include <p24FJ128GB206.h>
#include "config.h"
#include "common.h"
#include "usb.h"
#include "pin.h"
#include "timer.h"
#include "oc.h"

#define SET_SERVOS      1
#define GET_SERVOS      2
#define PING            3 

//void ClassRequests(void) {
//    switch (USB_setup.bRequest) {
//        default:
//            USB_error_flags |= 0x01;                    // set Request Error Flag
//    }
//}

void VendorRequests(void) {
    WORD temp;

    switch (USB_setup.bRequest) {
        case SET_SERVOS:
            pin_write(&D[2], USB_setup.wValue.w);
            pin_write(&D[3], USB_setup.wIndex.w);
            BD[EP0IN].bytecount = 0;    // set EP0 IN byte count to 0 
            BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
            break;
        case GET_SERVOS:
            temp.w = pin_read(&D[2]);
            BD[EP0IN].address[0] = temp.b[0];
            BD[EP0IN].address[1] = temp.b[1];
            temp.w = pin_read(&D[3]);
            BD[EP0IN].address[2] = temp.b[0];
            BD[EP0IN].address[3] = temp.b[1];
            BD[EP0IN].bytecount = 4;    // set EP0 IN byte count to 4
            BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
            break;            
        case PING:
            pin_write(&D[4], 0x8000);
            timer_start(&timer2);
            while (timer_flag(&timer2)==0) {}
            pin_write(&D[4], 0);
            timer_stop(&timer2);
            BD[EP0IN].bytecount = 0;    // set EP0 IN byte count to 0
            BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
            break;
        default:
            USB_error_flags |= 0x01;    // set Request Error Flag
    }
}

void VendorRequestsIn(void) {
    switch (USB_request.setup.bRequest) {
        default:
            USB_error_flags |= 0x01;                    // set Request Error Flag
    }
}

void VendorRequestsOut(void) {
    switch (USB_request.setup.bRequest) {
        default:
            USB_error_flags |= 0x01;                    // set Request Error Flag
    }
}

int16_t main(void) {
    init_clock();
    init_pin();
    init_timer();
    init_oc();

    oc_servo(&oc1, &D[2], &timer1, 20e-3, 0.8e-3, 2.2e-3, 0x8000);
    oc_servo(&oc2, &D[3], &timer1, 20e-3, 0.8e-3, 2.2e-3, 0x8000);

    oc_pwm(&oc3, &D[4], NULL, 40e3, 0);
    timer_setPeriod(&timer2, 0.5e-3);

    InitUSB();                              // initialize the USB registers and serial interface engine
    while (USB_USWSTAT!=CONFIG_STATE) {     // while the peripheral is not configured...
        ServiceUSB();                       // ...service USB requests
    }
    while (1) {
        ServiceUSB();                       // service any pending USB requests
    }
}

